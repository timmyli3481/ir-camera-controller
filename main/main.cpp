#include <crsf-controller.h>
#include <esp_log.h>
#include <stdio.h>
#include "ir-camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "init.h"
#include "flight-controller.h"

#include <gimbal-controller.h>
#define MAIN_APP_TAG "MAIN_APP" // Renamed TAG to avoid conflict with ir-camera.c

extern "C" void app_main(void)
{
    // Set log levels for both tags
    esp_log_level_set(MAIN_APP_TAG, ESP_LOG_DEBUG);
    esp_log_level_set("CAMERA_PARSER", ESP_LOG_DEBUG); // Set log level for UART camera task

    ESP_LOGI(MAIN_APP_TAG, "Hello world!");
    ESP_LOGI(MAIN_APP_TAG, "Running Init Program");
    init();
    ESP_LOGI(MAIN_APP_TAG, "Finished Running Init Program");

    BaseType_t CameraTaskxReturned = xTaskCreatePinnedToCore(uart_camera_task, "uart_task", 4096, NULL, 10, NULL,1);
    if (CameraTaskxReturned != pdPASS) {
        ESP_LOGE(MAIN_APP_TAG, "Failed to create UART camera task");
    } else {
        ESP_LOGI(MAIN_APP_TAG, "UART camera task created successfully");
    }

    BaseType_t GimbalTaskxReturned = xTaskCreatePinnedToCore(gimbal_controller_task, "gimbal_task", 4096, NULL, 10, NULL,1);
    if (GimbalTaskxReturned != pdPASS) {
        ESP_LOGE(MAIN_APP_TAG, "Failed to create Gimbal control task");
    } else {
        ESP_LOGI(MAIN_APP_TAG, "Gimbal control task created successfully");
    }

    BaseType_t CrsfTaskxReturned = xTaskCreatePinnedToCore(crsf_controller_task, "gimbal_task", 4096, NULL, 10, NULL,0);
    if (CrsfTaskxReturned != pdPASS) {
        ESP_LOGE(MAIN_APP_TAG, "Failed to create Crsf control task");
    } else {
        ESP_LOGI(MAIN_APP_TAG, "Crsf control task created successfully");
    }

    // Option 2: Delete app_main task itself (if no further main_task activity is needed)
    ESP_LOGI(MAIN_APP_TAG, "Setup complete, deleting main task.");
    vTaskDelete(NULL); // Deletes the current task (main_task)
}