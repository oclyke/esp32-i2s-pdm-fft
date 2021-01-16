idf_component_register( SRCS 
                          "i2s_example_main.c"
                          "./esp32-fft/components/fft/fft.c"
                        INCLUDE_DIRS 
                          "."
                          "./esp32-fft/components/fft/include"
                      )
