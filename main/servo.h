#ifndef __SERVO_H__
#define __SERVO_H__

#include "driver/ledc.h"
#include <stdbool.h>
#include "driver/gpio.h"
#include <stdint.h>

#define SERVO_FREQ_HZ        100
#define SERVO_PERIOD_US      (1000000 / SERVO_FREQ_HZ)   // 10000 µs
#define SERVO_MIN_US         0
#define SERVO_MAX_US         2100
#define SERVO_TIMER          LEDC_TIMER_0
#define SERVO_SPEED_MODE     LEDC_LOW_SPEED_MODE
#define SERVO_DUTY_RES       LEDC_TIMER_14_BIT
#define SERVO_DUTY_MAX       ((1U << 14) - 1)

typedef struct {
    gpio_num_t     gpio;
    ledc_channel_t channel;
    bool           initialized;
} servo_t;

esp_err_t servo_init(servo_t *servo, gpio_num_t gpio, ledc_channel_t channel) ;
esp_err_t servo_set_pulse_us(const servo_t *servo, uint32_t pulse_us) ;

#endif