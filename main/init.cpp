#include "init.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_log.h>
#include <main.h>


#include "ir-camera.h"
#include "flight-controller.h"
#include "crsf-controller.h"
#include "gimbal-controller.h"
#include "main.h"

#define TAG "INIT"

SemaphoreHandle_t camera_data_mutex;

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
    ESP_LOGI(TAG, "Running Mutex Creation Programs");
    camera_data_mutex=xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "Finished Running Mutex Creation Programs");

    ESP_LOGI(TAG, "Setup Complete");
}
