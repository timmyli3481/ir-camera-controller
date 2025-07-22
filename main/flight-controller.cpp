#include "flight-controller.h"

#include <esp_log.h>

#include "crsf.h"
#include "hal/gpio_types.h"

static const int TX_PIN = GPIO_NUM_13;
static const int RX_PIN = GPIO_NUM_15;

static CRSF* crsf = nullptr;
static CRSF* crsf2 = nullptr;


void flight_controller_init(void) {

    if (crsf) {
        delete crsf;
    }

    if (crsf2) {
        delete crsf2;
    }

    crsf =new CRSF(UART_NUM_1, 23, 19, 420000); // RX=16, TX=17, 420k baud
    crsf2 =new CRSF(UART_NUM_2, 17, 16, 420000); // RX=16, TX=17, 420k baud

    if (!crsf->isInitialized()) {
        ESP_LOGE("MAIN", "Failed to initialize CRSF");
        return;
    }

    if (!crsf2->isInitialized()) {
        ESP_LOGE("MAIN", "Failed to initialize CRSF");
        return;
    }
}

void flight_controller_task(void *pvParameters) {

}