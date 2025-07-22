//
// Created by Timmy Li on 7/22/25.
//

#include "gimbal-controller.h"
#include "main.h"

#include <esp_log.h>
#include <FreeRTOSConfig.h>
#include <portmacro.h>
#include <freertos/task.h>

#include "iot_servo.h"

#define TAG "GIMBAL_CONTROL"
#define MAX_ANGLE 120


gimbal_controller_input current_input={
    .tracking = false,
    .target_x = 0,
    .target_y = 0
};

gimbal_state current_state={
    .servo_0_angle = 0,
    .servo_1_angle = 0,
    .servo_0_adding = true,
    .servo_1_adding = true
};



void gimbal_control_init() {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "Gimbal control initialized");

    servo_config_t config = {
        .max_angle = MAX_ANGLE,
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

void apply_angle(uint16_t servo_0_angle, uint16_t servo_1_angle) {
    current_state.servo_0_angle = servo_0_angle;
    current_state.servo_1_angle = servo_1_angle;

    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, servo_0_angle);
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, servo_1_angle);
}

void gimbal_control_task(void *pv) {

    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "Gimbal control task started");


    while (1) {
        if (xQueueReceive(gimbal_controller_queue, &current_input, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Received input: tracking=%d, target_x=%d, target_y=%d", current_input.tracking, current_input.target_x, current_input.target_y);
        }

        if (current_input.tracking) {
            apply_angle(current_input.target_x, current_input.target_y);
        }else {
            if (current_state.servo_0_angle >= MAX_ANGLE) {
                current_state.servo_0_angle = MAX_ANGLE;
                current_state.servo_0_adding = false;
            }

            else if (current_state.servo_0_angle <= 0) {
                current_state.servo_0_angle = 0;
                current_state.servo_0_adding = true;
            }

            if (current_state.servo_1_angle >= MAX_ANGLE) {
                current_state.servo_1_angle = MAX_ANGLE;
                current_state.servo_1_adding = false;
            }

            else if (current_state.servo_1_angle <= 0) {
                current_state.servo_1_angle = 0;
                current_state.servo_1_adding = true;
            }

            if (current_state.servo_0_adding) {
                current_state.servo_0_angle+=0.01;
            } else {
                current_state.servo_0_angle-=0.01;
            }

            if (current_state.servo_1_adding) {
                current_state.servo_1_angle+=0.01;
            } else {
                current_state.servo_1_angle-=0.01;
            }


            apply_angle(current_state.servo_0_angle, current_state.servo_1_angle);
        }

    }
}

