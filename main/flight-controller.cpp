#include "flight-controller.h"
#include <esp_log.h>
#include "gimbal-controller.h"
#include "main.h"
#include "crsf-controller.h"

#define TAG "FLIGHT_CONTROLLER"
#define PID_KP 0.1
#define PID_KI 0.01 // Added a small integral gain
#define PID_KD 0.005 // Added a small derivative gain
#define YAW_SETPOINT 0.0 // Assuming a desired yaw angle of 0

// Global variables for PID control
static float prev_yaw_error = 0.0;
static float integral_yaw_error = 0.0;
static const float INTEGRAL_WINDUP_LIMIT = 10.0; // Limit for integral windup

crsf_command_t current_command = {
    .throttle = {.value = 0, .is_null = true},
    .yaw = {.value = 0, .is_null = true},
    .pitch = {.value = 0, .is_null = true},
    .roll = {.value = 0, .is_null = true},
};

void yaw_pid() {
    float yaw_error = YAW_SETPOINT - gimbal_state_g.servo_0_angle;

    // Proportional term
    float proportional_term = PID_KP * yaw_error;

    // Integral term
    integral_yaw_error += yaw_error;
    // Apply integral windup limit
    if (integral_yaw_error > INTEGRAL_WINDUP_LIMIT) {
        integral_yaw_error = INTEGRAL_WINDUP_LIMIT;
    } else if (integral_yaw_error < -INTEGRAL_WINDUP_LIMIT) {
        integral_yaw_error = -INTEGRAL_WINDUP_LIMIT;
    }
    float integral_term = PID_KI * integral_yaw_error;

    // Derivative term
    float derivative_term = PID_KD * (yaw_error - prev_yaw_error);

    float yaw_output = proportional_term + integral_term + derivative_term;

    // Update previous error for the next iteration
    prev_yaw_error = yaw_error;

    // Apply the PID output to the yaw command
    current_command.yaw.value = yaw_output + 1500;
    current_command.yaw.is_null = false; // Set to false since we are actively controlling yaw

    ESP_LOGD(TAG, "Yaw Error: %.2f, Yaw Output: %.2f", yaw_error, yaw_output);
}

void set_current_command_to_empty() {
    current_command.throttle = {.value = 0, .is_null = true};
    current_command.yaw = {.value = 0, .is_null = true};
    current_command.pitch = {.value = 0, .is_null = true};
    current_command.roll = {.value = 0, .is_null = true};
}

void flight_controller_task(void *pvParameters) {
    ESP_LOGI(TAG, "Flight controller task started");
    while (1) {
        set_current_command_to_empty();
        if (!gimbal_state_g.tracking) {
            xQueueSend(crsf_commands_queue,&current_command ,0);
        } else {
            yaw_pid(); // Call the yaw PID function when tracking
            xQueueSend(crsf_commands_queue, &current_command, 0); // Send the updated command
        }

        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}