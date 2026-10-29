#include <stdio.h>
#include "hardware.h"
#include "esp_log.h"
#include <mpu6050.h>
#include "driver/gpio.h"

#define HIGH_PASS_FILTER MPU6050_DHPF_5
#define DLPF MPU6050_DLPF_1
#define TAG "HARDWARE"
#define GYRO2 1

void config_mpu(mpu6050_dev_t *dev1,mpu6050_dev_t *dev2)
{
   
    ESP_LOGI(TAG, "Configuration du gyro 1") ;
    ESP_ERROR_CHECK(mpu6050_init_desc(dev1, ADDR, I2C_NUM_0, 21, 22));
    while (1)
    {
        //ESP_LOGI(TAG, "avant i2c_dev_probe") ;
        esp_err_t res = i2c_dev_probe(&dev1->i2c_dev, I2C_DEV_WRITE);
        //ESP_LOGI(TAG, "après i2c_dev_probe") ;
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU6050 device 1");
            break;
        }
        ESP_LOGE(TAG, "MPU6050 1 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //ESP_LOGI(TAG, "Avant mpu6050_init") ;
    ESP_ERROR_CHECK(mpu6050_init(dev1));
    //ESP_LOGI(TAG, "Après mpu6050_init") ;

    ESP_ERROR_CHECK(mpu6050_set_rate(dev1, 0)); // Set sample rate to 1 kHz.
    ESP_ERROR_CHECK(mpu6050_set_dlpf_mode(dev1, DLPF));
    ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(dev1, MPU6050_GYRO_RANGE_250));
    mpu6050_set_dhpf_mode(dev1,HIGH_PASS_FILTER) ;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&dev, MPU6050_ACCEL_RANGE_2)) ;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //ESP_LOGI(TAG, "Avant mpu6050_set_interrupt_mode") ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_mode(dev1,MPU6050_INT_LEVEL_HIGH)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_drive(dev1,MPU6050_INT_PUSH_PULL)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_latch(dev1,MPU6050_INT_LATCH_PULSE)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_latch_clear(dev1,1)) ;
    ESP_ERROR_CHECK(mpu6050_set_int_enabled(dev1,0x01)) ;
    //ESP_LOGI(TAG, "Après mpu6050_get_interrupt_latch_clear") ;

#ifdef GYRO2
    ESP_LOGI(TAG, "Configuration du gyro 2") ;
    ESP_ERROR_CHECK(mpu6050_init_desc(dev2, ADDR, I2C_NUM_1, 25, 26));
    while (1)
    {
        //ESP_LOGI(TAG, "avant i2c_dev_probe") ;
        esp_err_t res = i2c_dev_probe(&dev2->i2c_dev, I2C_DEV_WRITE);
        //ESP_LOGI(TAG, "après i2c_dev_probe") ;
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU6050 device 2");
            break;
        }
        ESP_LOGE(TAG, "MPU6050 2 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    //ESP_LOGI(TAG, "Avant mpu6050_init") ;
    ESP_ERROR_CHECK(mpu6050_init(dev2));
    //ESP_LOGI(TAG, "Après mpu6050_init") ;
    ESP_ERROR_CHECK(mpu6050_set_rate(dev2, 0)); // Set sample rate to 1 kHz.
    ESP_ERROR_CHECK(mpu6050_set_dlpf_mode(dev2, DLPF));
    ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(dev2, MPU6050_GYRO_RANGE_250));
    mpu6050_set_dhpf_mode(dev2,HIGH_PASS_FILTER) ;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(dev, MPU6050_ACCEL_RANGE_2)) ;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //ESP_LOGI(TAG, "Avant mpu6050_set_interrupt_mode") ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_mode(dev2,MPU6050_INT_LEVEL_HIGH)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_drive(dev2,MPU6050_INT_PUSH_PULL)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_latch(dev2,MPU6050_INT_LATCH_PULSE)) ;
    ESP_ERROR_CHECK(mpu6050_set_interrupt_latch_clear(dev2,1)) ;
    ESP_ERROR_CHECK(mpu6050_set_int_enabled(dev2,0x01)) ;
    //ESP_LOGI(TAG, "Après mpu6050_get_interrupt_latch_clear") ;
#endif

    vTaskDelay(100 / portTICK_PERIOD_MS);

}