#include "init.h"

#include <esp_log.h>

#include "ir-camera.h"
#include "flight-controller.h"

#define TAG "INIT"

void init(void) {
    esp_log_level_set(TAG,ESP_LOG_INFO);

    ESP_LOGI(TAG,"Running Init Programs");
    ESP_LOGI(TAG,"Initializing IR camera Uart Communication");
    uart_camera_init();
    ESP_LOGI(TAG,"Finished Initializing IR camera Uart Communication");
    ESP_LOGI(TAG,"Initializing Flight Controller");
    flight_controller_init();
    ESP_LOGI(TAG,"Finished Initializing Flight Controller");
    ESP_LOGI(TAG,"Finished Running Init Programs");
}