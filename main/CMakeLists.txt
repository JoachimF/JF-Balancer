idf_component_register(SRCS "calibration.c" "servo.c" "jf-balancer.c" "hardware.c" "filtres.c" "uart_console.c"
                       INCLUDE_DIRS "."
                       REQUIRES mpu6050 esp_timer)
