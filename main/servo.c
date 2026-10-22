#include <stdint.h>
#include <stdbool.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "servo.h"

static const char *TAG = "servo";

static bool s_timer_ready = false;

esp_err_t servo_init(servo_t *servo, gpio_num_t gpio, ledc_channel_t channel)
{
    if (servo == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
        ESP_LOGE(TAG, "GPIO %d invalide en sortie", gpio);
        return ESP_ERR_INVALID_ARG;
    }
    if (channel >= LEDC_CHANNEL_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    // Le timer est partagé entre toutes les servos (même fréquence)
    if (!s_timer_ready) {
        const ledc_timer_config_t timer_cfg = {
            .speed_mode      = SERVO_SPEED_MODE,
            .duty_resolution = SERVO_DUTY_RES,
            .timer_num       = SERVO_TIMER,
            .freq_hz         = SERVO_FREQ_HZ,
            .clk_cfg         = LEDC_AUTO_CLK,
        };
        esp_err_t err = ledc_timer_config(&timer_cfg);
        if (err != ESP_OK) {
            return err;
        }
        s_timer_ready = true;
    }

    const ledc_channel_config_t ch_cfg = {
        .gpio_num   = gpio,
        .speed_mode = SERVO_SPEED_MODE,
        .channel    = channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    esp_err_t err = ledc_channel_config(&ch_cfg);
    if (err != ESP_OK) {
        return err;
    }

    servo->gpio = gpio;
    servo->channel = channel;
    servo->initialized = true;
    return ESP_OK;
}

esp_err_t servo_set_pulse_us(const servo_t *servo, uint32_t pulse_us)
{
    if (servo == NULL || !servo->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pulse_us < SERVO_MIN_US || pulse_us > SERVO_MAX_US) {
        ESP_LOGW(TAG, "Impulsion %lu µs hors limites [%d..%d]",
                 (unsigned long)pulse_us, SERVO_MIN_US, SERVO_MAX_US);
        return ESP_ERR_INVALID_ARG;
    }

    // duty = pulse / période * 2^14 (arrondi au plus proche)
    uint32_t duty = ((uint64_t)pulse_us * (1U << 14) + SERVO_PERIOD_US / 2) / SERVO_PERIOD_US;
    if (duty > SERVO_DUTY_MAX) {
        duty = SERVO_DUTY_MAX;
    }

    esp_err_t err = ledc_set_duty(SERVO_SPEED_MODE, servo->channel, duty);
    if (err != ESP_OK) {
        return err;
    }
    return ledc_update_duty(SERVO_SPEED_MODE, servo->channel);
}