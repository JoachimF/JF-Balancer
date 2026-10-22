idf_component_register(SRCS "servo.c" "jf-balancer.c"
                       INCLUDE_DIRS "."
                       REQUIRES mpu6050 esp_timer)
