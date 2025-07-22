#include "init.h"

#include <esp_log.h>
#include <main.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "ir-camera.h"
#include "flight-controller.h"
#include "crsf-controller.h"
#include "gimbal-controller.h"

#define TAG "INIT"

QueueHandle_t ir_camera_queue;

void init(void) {
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Running Init Programs");
    ESP_LOGI(TAG, "Initializing IR camera Uart Communication");
    uart_camera_init();
    ESP_LOGI(TAG, "Finished Initializing IR camera Uart Communication");
    ESP_LOGI(TAG, "Initializing Flight Controller");
    crsf_controller_init();
    ESP_LOGI(TAG, "Finished Initializing Flight Controller");
    ESP_LOGI(TAG, "Initializing Gimbal Control");
    gimbal_controller_init();
    ESP_LOGI(TAG, "Finished Initializing Gimbal Control");
    ESP_LOGI(TAG, "Finished Running Init Programs");

    ESP_LOGI(TAG, "Creating IR Camera Queue");
    ir_camera_queue = xQueueCreate(10, sizeof(camera_data_t));
    ESP_LOGI(TAG, "Finished Creating IR Camera Queue");
    ESP_LOGI(TAG, "Creating Gimbal Controller Queue");
    gimbal_controller_queue=xQueueCreate(10,sizeof(gimbal_controller_input));
    ESP_LOGI(TAG, "Finished Creating Gimbal Controller Queue");
    ESP_LOGI(TAG, "Finished Running Queue Creation Programs");

    ESP_LOGI(TAG, "Setup Complete");
}
