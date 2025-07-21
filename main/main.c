#include <esp_log.h>
#include <stdio.h>
#include "ir-camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define MAIN_APP_TAG "MAIN_APP" // Renamed TAG to avoid conflict with ir-camera.c

void app_main(void)
{
    // Set log levels for both tags
    esp_log_level_set(MAIN_APP_TAG, ESP_LOG_DEBUG);
    esp_log_level_set("CAMERA_PARSER", ESP_LOG_DEBUG); // Set log level for UART camera task

    ESP_LOGI(MAIN_APP_TAG, "Hello world!");
    ESP_LOGD(MAIN_APP_TAG, "Running Uart Camera Task Init");

    uart_camera_init(); // This will log with "UART_CAMERA_TASK" TAG

    ESP_LOGD(MAIN_APP_TAG, "Uart Camera Task Init Finished");

    BaseType_t xReturned = xTaskCreate(uart_camera_task, "uart_task", 4096, NULL, 10, NULL);
    if (xReturned != pdPASS) {
        ESP_LOGE(MAIN_APP_TAG, "Failed to create UART camera task");
    } else {
        ESP_LOGI(MAIN_APP_TAG, "UART camera task created successfully");
    }

    // Prevent app_main from returning immediately
    // Option 1: Infinite loop (common for simple applications)
    // while (1) {
    //     ESP_LOGD(MAIN_APP_TAG, "app_main idle loop...");
    //     vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 5 seconds
    // }

    // Option 2: Delete app_main task itself (if no further main_task activity is needed)
    ESP_LOGI(MAIN_APP_TAG, "Setup complete, deleting main task.");
    vTaskDelete(NULL); // Deletes the current task (main_task)
}