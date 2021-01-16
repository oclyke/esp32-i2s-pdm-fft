/* 
// This file is subject to the terms and conditions defined in
// file 'LICENSE.md', which is part of this source code package.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/message_buffer.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include <math.h>

#include "fft.h" // github.com/fakufaku/esp32-fft

/////////////
// USER SETUP

// #define DEBUG_PRINT_RAW
// #define DEBUG_PRINT_AVG
// #define DEBUG_FFT_FULL_BIN_INFO
#define DEBUG_FFT_STRONGEST

#define PIN_CLK (5)
#define PIN_DATA (35)

#define SAMPLE_BIT_SIZE             (I2S_BITS_PER_SAMPLE_16BIT)
#define SAMPLE_RATE_HZ              (44100) // not sure if decimation is considered?

#define FFT_BUF_SAMPLES             (2048)   // (4096), <-- does not trigger watchdog ====== triggers watchdog --> (1024) (512)
#define FFT_BUFFERS                 (2)     // double buffering

#define DMA_BUF_BYTES               (1024)  // maximum allowable size is 1024 (by i2s driver implemnetation)

// END USER SETUP
/////////////////

// determine overall buffer size
#if SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_32BIT
#define FFT_BYTES_PER_SAMPLE        (4)
#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_24BIT
#error "24 bit depth not supported currently"
#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_16BIT
#define FFT_BYTES_PER_SAMPLE        (2)
#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_8BIT
#define FFT_BYTES_PER_SAMPLE        (1)
#else
#error "sample bit size incorrectly configured"
#endif

#define FFT_BUF_BYTES               (FFT_BUF_SAMPLES * FFT_BYTES_PER_SAMPLE)
#define PDM_BUF_BYTES               (FFT_BUF_BYTES * FFT_BUFFERS)
#define DMA_BUFFERS                 ((FFT_BUF_BYTES * FFT_BUFFERS) / DMA_BUF_BYTES)

#define BIN_WIDTH_HZ                ((float)SAMPLE_RATE_HZ / (float)FFT_BUF_SAMPLES) // just a total guess lahmaoh

// note: though presented as an unsigned buffer this is really a signed value - functions that interpret 
// these values should use proper casting (to a signed type)
uint8_t PDMDataBuffer[PDM_BUF_BYTES];

MessageBufferHandle_t buf_idx_msg_handle;
const size_t buf_idx_msg_bytes = sizeof(size_t) + sizeof(size_t); // one size_t for buffer index, another size_t for MessageBuffer overhead

// fwd declarations
void disp_buf(uint8_t* buf, size_t length);
void disp_avg_buf(uint8_t* buf, size_t length);
void i2s_init();


// //*****************************************************************************
// //
// // Analyze and print frequency data.
// //
// //*****************************************************************************
// void
// pcm_fft_print(void)
// {
//     float fMaxValue;
//     uint32_t ui32MaxIndex;
//     int16_t *pi16PDMData = (int16_t *) g_ui32PDMDataBuffer;
//     uint32_t ui32LoudestFrequency;

//     //
//     // Convert the PDM samples to floats, and arrange them in the format
//     // required by the FFT function.
//     //
//     for (uint32_t i = 0; i < PDM_FFT_SIZE; i++)
//     {
//         if (PRINT_PDM_DATA)
//         {
//             am_util_stdio_printf("%d\n", pi16PDMData[i]);
//         }

//         g_fPDMTimeDomain[2 * i] = pi16PDMData[i] / 1.0;
//         g_fPDMTimeDomain[2 * i + 1] = 0.0;
//     }

//     if (PRINT_PDM_DATA)
//     {
//         am_util_stdio_printf("END\n");
//     }

//     //
//     // Perform the FFT.
//     //
//     arm_cfft_radix4_instance_f32 S;
//     arm_cfft_radix4_init_f32(&S, PDM_FFT_SIZE, 0, 1);
//     arm_cfft_radix4_f32(&S, g_fPDMTimeDomain);
//     arm_cmplx_mag_f32(g_fPDMTimeDomain, g_fPDMMagnitudes, PDM_FFT_SIZE);

//     if (PRINT_FFT_DATA)
//     {
//         for (uint32_t i = 0; i < PDM_FFT_SIZE / 2; i++)
//         {
//             am_util_stdio_printf("%f\n", g_fPDMMagnitudes[i]);
//         }

//         am_util_stdio_printf("END\n");
//     }

//     //
//     // Find the frequency bin with the largest magnitude.
//     //
//     arm_max_f32(g_fPDMMagnitudes, PDM_FFT_SIZE / 2, &fMaxValue, &ui32MaxIndex);

//     ui32LoudestFrequency = (g_ui32SampleFreq * ui32MaxIndex) / PDM_FFT_SIZE;

//     if (PRINT_FFT_DATA)
//     {
//         am_util_stdio_printf("Loudest frequency bin: %d\n", ui32MaxIndex);
//     }

//     am_util_stdio_printf("Loudest frequency: %d         \n", ui32LoudestFrequency);
// }


void tsk_process (void* arg) {
    size_t buf_idx = 0;
    uint8_t* buf = NULL;

    while (1) {
        size_t rx_bytes = xMessageBufferReceive( buf_idx_msg_handle, (void*)(&buf_idx), sizeof(buf_idx), portMAX_DELAY );
        assert(rx_bytes == sizeof(buf_idx));
        buf = &PDMDataBuffer[buf_idx * FFT_BUF_BYTES]; // get the proper buffer space
        // ESP_LOGI("processor", "received %d bytes, buffer index %d", rx_bytes, buf_idx);

#ifndef DEBUG_PRINT_RAW
        
#else
        disp_buf(buf, FFT_BUF_BYTES); // print the buffer contents
#endif

#ifdef DEBUG_PRINT_AVG
        disp_avg_buf(buf, FFT_BUF_BYTES); // print avg value
#endif

#if SAMPLE_BIT_SIZE != I2S_BITS_PER_SAMPLE_16BIT
#error "fft code is currently assuming 16 bit samples - need work to change the sample bit size"
#endif
        // init FFT w/ dynamic memory allocation
        fft_config_t *real_fft_plan = fft_init(FFT_BUF_SAMPLES, FFT_REAL, FFT_FORWARD, NULL, NULL);
        assert(real_fft_plan);
        assert(real_fft_plan->size == FFT_BUF_SAMPLES);

        // prepare input to fft
        int16_t* i16buf = (int16_t*)buf;
        for(size_t k = 0; k < real_fft_plan->size; k++){
            real_fft_plan->input[k] = (float)i16buf[k];
        }

        // perform fft
        fft_execute(real_fft_plan);

        // use the results:
#ifdef DEBUG_FFT_FULL_BIN_INFO
        printf("DC component : %f\n", real_fft_plan->output[0]);  // DC is at [0]
        for (size_t k = 1 ; k < real_fft_plan->size / 2 ; k++){
            printf("%d-th freq : %f+j%f\n", k, real_fft_plan->output[2*k], real_fft_plan->output[2*k+1]);
        }  
        printf("Middle component : %f\n", real_fft_plan->output[1]);  // N/2 is real and stored at [1]
#endif

#ifdef DEBUG_FFT_STRONGEST
        size_t strongest_k = 0;
        float  strongest_val = 0.0;
        for (size_t k = 1 ; k < real_fft_plan->size / 2 ; k++){
            float val = real_fft_plan->output[2*k];
            if(fabs(strongest_val) < fabs(val)){
                strongest_val = val;
                strongest_k = k;
            }
        }
        printf("%f\n", (float)strongest_k*BIN_WIDTH_HZ);
#endif

        // clean up output
        fft_destroy(real_fft_plan);
	}
}

void tsk_record (void* arg) {
    size_t buf_idx = 0;
    size_t bytes_read = 0;
    uint8_t* buf = NULL;

    buf_idx_msg_handle = xMessageBufferCreate( buf_idx_msg_bytes );
    assert(buf_idx_msg_handle);
    xTaskCreate(tsk_process, "process task", 1024 * 2, NULL, 5, NULL);

    while (1) {
        bytes_read = 0;
        buf = &PDMDataBuffer[buf_idx * FFT_BUF_BYTES]; // get the proper buffer space
	    i2s_read(I2S_NUM_0, buf, FFT_BUF_BYTES, &bytes_read, portMAX_DELAY);
        assert(bytes_read == FFT_BUF_BYTES);

        // signal the processing task which buffer to handle
        size_t tx_bytes = xMessageBufferSend(buf_idx_msg_handle, &buf_idx, sizeof(size_t), portMAX_DELAY);

        // increment the buffer to use
        buf_idx++;
        if(buf_idx >= FFT_BUFFERS){
            buf_idx = 0;
        }
	}
}


void app_main() {
    i2s_init();
    vTaskDelay(1000 / portTICK_RATE_MS);
    xTaskCreate(tsk_record, "record task", 1024 * 2, NULL, 5, NULL);
    
}


// debug display functions
void disp_buf(uint8_t* buf, size_t length) {
#if SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_32BIT
    int32_t* b = (int32_t*)buf;
#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_24BIT
#error "24 bit depth not supported currently"
#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_16BIT
    int16_t* b = (int16_t*)buf;
#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_8BIT
    int8_t* b = (int8_t*)buf;
#else
#error "sample bit size incorrectly configured"
#endif
    
    for (size_t i = 0; i < length; i++) {
        printf("%i\n", b[i]);
    }
}

void disp_avg_buf(uint8_t* buf, size_t length) {
    int64_t acc = 0;
    int16_t* b = (int16_t*)buf;
    for (size_t idx = 0; idx < length; idx++) {
        acc += b[idx];   
    }
    printf("%i\n", (int16_t)(acc/length));
}



// i2s initialization
void i2s_init() {
    int i2s_num = 0;
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX |I2S_MODE_PDM,
        .sample_rate =  SAMPLE_RATE_HZ,
        .bits_per_sample = SAMPLE_BIT_SIZE,
        .communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT,
        // .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = 0,
        .dma_buf_count = DMA_BUFFERS,
        .dma_buf_len = DMA_BUF_BYTES,
    };

    i2s_pin_config_t pin_config = {
        .ws_io_num   = PIN_CLK,
        .data_in_num = PIN_DATA,
    };

    //install and start i2s driver
    i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
}
