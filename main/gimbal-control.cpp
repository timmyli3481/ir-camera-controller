#include "gimbal-control.h"

#include <esp_log.h>
#include <iot_servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "GIMBAL_CONTROL"



void gimbal_control_init() {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "Gimbal control initialized");

    servo_config_t config = {
        .max_angle = 120,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                GPIO_NUM_33,
            },
            .ch = {
                LEDC_CHANNEL_0,
            },
        },
        .channel_number = 1,
    };

    iot_servo_init(LEDC_LOW_SPEED_MODE, &config);
}

void gimbal_control_task(void *arg) {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "Gimbal control task started");

    while (1) {
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE,0, 90);
        ESP_LOGI(TAG, "Set angle to 90 degrees");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE,0, 0);
        ESP_LOGI(TAG, "Set angle to 0 degrees");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

