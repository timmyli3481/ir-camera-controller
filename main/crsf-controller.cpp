#include "crsf-controller.h"

#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "crsf.h"
#include "main.h"
#include "hal/gpio_types.h"


static CRSF* crsf = nullptr;
static CRSF* crsf2 = nullptr;

static crsf_command_t last_command = {
    .throttle = {.value = 0, .is_null = true},
    .yaw = {.value = 0, .is_null = true},
    .pitch = {.value = 0, .is_null = true},
    .roll = {.value = 0, .is_null = true},
};

bool should_send = false;


void crsf_controller_init(void) {

    if (crsf) {
        delete crsf;
    }

    if (crsf2) {
        delete crsf2;
    }

    crsf =new CRSF(UART_NUM_1, 15, 7, 420000); // RX=16, TX=17, 420k baud
    crsf2 =new CRSF(UART_NUM_2, 18, 8, 420000); // RX=16, TX=17, 420k baud

    if (!crsf->isInitialized()) {
        ESP_LOGE("MAIN", "Failed to initialize CRSF");
        return;
    }

    if (!crsf2->isInitialized()) {
        ESP_LOGE("MAIN", "Failed to initialize CRSF");
        return;
    }
}

void crsf_controller_task(void *pvParameters) {
    crsf_channels_t channels;

    ESP_LOGI("CRSF_CONTROLLER", "CRSF controller task started");

    while (1) {
        // Poll both CRSF instances for incoming data
        if (crsf) {
            crsf->poll(true);
        }

        if (crsf && crsf->getChannels(&channels)) {
            should_send = true;
            ESP_LOGD("CRSF_CONTROLLER", "Channels received successfully");
        }

        if (xQueueReceive(crsf_commands_queue, &last_command, 0) == pdPASS) {
            should_send = true;
            ESP_LOGD("CRSF_CONTROLLER", "Command received successfully");
        }

        if (!last_command.throttle.is_null) {
            channels.channels[3] = last_command.throttle.value;
        }

        if (!last_command.yaw.is_null) {
            channels.channels[1] = last_command.yaw.value;
        }

        if (!last_command.pitch.is_null) {
            channels.channels[0] = last_command.pitch.value;
        }

        if (!last_command.roll.is_null) {
            channels.channels[2] = last_command.roll.value;
        }

        // Check if we have fresh channels from the first CRSF instance
        if (should_send) {
            // Log the channel values
            // ESP_LOGI("CRSF_CONTROLLER", "Channels: [%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u] (timestamp: %llu)",
            //     channels.channels[0], channels.channels[1], channels.channels[2], channels.channels[3],
            //     channels.channels[4], channels.channels[5], channels.channels[6], channels.channels[7],
            //     channels.channels[8], channels.channels[9], channels.channels[10], channels.channels[11],
            //     channels.channels[12], channels.channels[13], channels.channels[14], channels.channels[15],
            //     channels.timestamp_us);

            // Forward the channels to the second CRSF instance
            if (crsf2) {
                if (crsf2->sendChannels(&channels)) {
                    ESP_LOGD("CRSF_CONTROLLER", "Channels forwarded successfully");
                } else {
                    ESP_LOGW("CRSF_CONTROLLER", "Failed to forward channels to crsf2");
                }
            }
        }

        // Small delay to prevent task from consuming too much CPU
        vTaskDelay(pdMS_TO_TICKS(1)); // 1ms delay
    }
}