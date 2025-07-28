#include "water-controller.h"

#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include <FreeRTOSConfig.h>
#include <portmacro.h>
#include <driver/gpio.h>
#include <freertos/task.h>
#include <hal/gpio_types.h>

#define TAG "WATER_CONTROLLER"

void water_controller_task(void *pvParameters) {
    gpio_set_direction(GPIO_NUM_19, GPIO_MODE_OUTPUT);
    while (1) {
        ESP_LOGI(TAG, "open water valve");
        gpio_set_level(GPIO_NUM_19, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "close water valve");

        gpio_set_level(GPIO_NUM_19, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
