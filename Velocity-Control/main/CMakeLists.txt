idf_component_register(SRCS "bno055.c" "platform_esp32s3.c" "as5600_lib.c" "bldc_pwm.c" "led.c" "uart_console.c" "main.c"
                    INCLUDE_DIRS "." "../main"
                    )
#file(GLOB SRC_FILES "../sensors//.c" "../platform/*.c")