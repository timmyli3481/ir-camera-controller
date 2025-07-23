//
// Created by Timmy Li on 7/22/25.
//

#include "gimbal-controller.h"

#include <esp_log.h>
#include <FreeRTOSConfig.h>
#include <ir-camera.h>
#include <main.h>
#include <portmacro.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "iot_servo.h"
#include <math.h> // For fabs if needed for precise control, but maybe not for simple relative adjustment

#define TAG "GIMBAL_CONTROL"
#define MAX_ANGLE 180
#define SWEEP_INCREMENT 10 // Define a constant for the sweep increment

// NOTE: If the camera's angle_x/y output represents pixel offset that needs
// to be converted to degrees, you might need a conversion factor here.
// For example: #define PIXEL_TO_DEGREE_CONVERSION_X 0.1 // degrees per pixel
//              #define PIXEL_TO_DEGREE_CONVERSION_Y 0.1

gimbal_state current_state = {
    .servo_0_angle = 0,
    .servo_1_angle = 0,
    .servo_0_adding = true,
    .servo_1_adding = true,
    .tracking = false
};

void gimbal_controller_init() {
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
                GPIO_NUM_37,
                GPIO_NUM_38
            },
            .ch = {
                LEDC_CHANNEL_0,
                LEDC_CHANNEL_1
            },
        },
        .channel_number = 2, // There are two channels being used (0 and 1)
    };

    iot_servo_init(LEDC_LOW_SPEED_MODE, &config);
}

void apply_angle(float servo_0_angle, float servo_1_angle) {
    // Clamp angles to be within the valid range [0, MAX_ANGLE]
    if (servo_0_angle < 0) {
        servo_0_angle = 0;
    } else if (servo_0_angle > MAX_ANGLE) {
        servo_0_angle = MAX_ANGLE;
    }

    if (servo_1_angle < 0) {
        servo_1_angle = 0;
    } else if (servo_1_angle > MAX_ANGLE) {
        servo_1_angle = MAX_ANGLE;
    }

    current_state.servo_0_angle = servo_0_angle;
    current_state.servo_1_angle = servo_1_angle;

    ESP_LOGI(TAG, "Applying angle: servo_0_angle=%f, servo_1_angle=%f", current_state.servo_0_angle, current_state.servo_1_angle);

    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, current_state.servo_0_angle);
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, current_state.servo_1_angle);
}

void gimbal_controller_task(void *pv) {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "Gimbal control task started");

    while (1) {
        ESP_LOGD(TAG, "Waiting for input (with timeout)");
        camera_data_t current_input;
        bool newDataAvailable = false;

        if (xSemaphoreTake(camera_data_mutex, pdMS_TO_TICKS(100)) == pdPASS) {
            current_input = g_camera_data;
            xSemaphoreGive(camera_data_mutex);
            newDataAvailable = true;
            ESP_LOGD(TAG, "Received input");
        } else {
            ESP_LOGD(TAG, "No new camera data. Continuing current action.");
        }

        if (newDataAvailable && current_input.found_fire) {
            ESP_LOGI(TAG, "Fire found! Relative adjustment: x=%f, y=%f", current_input.angle_x, current_input.angle_y);
            current_state.tracking = true;

            // Apply relative adjustments to current angles
            float new_servo_0_angle = current_state.servo_0_angle + current_input.angle_x;
            float new_servo_1_angle = current_state.servo_1_angle + current_input.angle_y;

            // Apply the new calculated angles (which will also clamp them)
            apply_angle(new_servo_0_angle, new_servo_1_angle);

            ESP_LOGD(TAG, "Tracking fire: current_x=%f (adjusted by %f), current_y=%f (adjusted by %f)",
                     current_state.servo_0_angle, current_input.angle_x,
                     current_state.servo_1_angle, current_input.angle_y);

        } else if (newDataAvailable && !current_input.found_fire) {
            ESP_LOGD(TAG, "No fire found in new data. Switching to sweep.");
            current_state.tracking = false;
        }
        // If no new data and was tracking, it will just stay in its last position
        // until new data (with or without fire) comes in.
        // If no new data and was sweeping, it will continue sweeping in the 'else' block below.

        if (!current_state.tracking) { // Only sweep if not tracking fire
            ESP_LOGD(TAG, "Sweeping...");
            // Servo 0 (horizontal) movement
            if (current_state.servo_0_adding) {
                current_state.servo_0_angle += SWEEP_INCREMENT;
                if (current_state.servo_0_angle >= MAX_ANGLE) {
                    current_state.servo_0_angle = MAX_ANGLE;
                    current_state.servo_0_adding = false; // Change direction
                }
            } else {
                current_state.servo_0_angle -= SWEEP_INCREMENT;
                if (current_state.servo_0_angle <= 0) {
                    current_state.servo_0_angle = 0;
                    current_state.servo_0_adding = true; // Change direction
                }
            }

            // Servo 1 (vertical) movement - simplified for a basic sweep
            if (current_state.servo_1_adding) {
                current_state.servo_1_angle += SWEEP_INCREMENT;
                if (current_state.servo_1_angle >= MAX_ANGLE) {
                    current_state.servo_1_angle = MAX_ANGLE;
                    current_state.servo_1_adding = false; // Change direction
                }
            } else {
                current_state.servo_1_angle -= SWEEP_INCREMENT;
                if (current_state.servo_1_angle <= 0) {
                    current_state.servo_1_angle = 0;
                    current_state.servo_1_adding = true; // Change direction
                }
            }
            // Apply the new, incrementally updated angle for sweeping
            apply_angle(current_state.servo_0_angle, current_state.servo_1_angle);
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}