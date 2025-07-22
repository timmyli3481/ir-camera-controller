#include "ir-camera.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>        // for atof, atoi

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "hal/gpio_types.h"
#include "esp_log.h"
#include "softuart.h"

static const char *TAG = "CAMERA_PARSER";

// choose any free soft-UART port (0..UART_NUM_MAX-1)
#define SOFT_UART_PORT    UART_NUM_1
#define TX_PIN            GPIO_NUM_18
#define RX_PIN            GPIO_NUM_5
#define BAUD_RATE         9600

// global camera data
camera_data_t g_camera_data = {
    .found_fire = false,
    .angle_x    = 0.0f,
    .angle_y    = 0.0f,
    .pixels     = 0,
    .cx         = 0,
    .cy         = 0,
};

// RX line buffer
static char  s_rx_buffer[CAMERA_RX_BUFFER_SIZE];
static int   s_rx_buffer_idx = 0;

void uart_camera_init(void)
{
    uart_config_t cfg = {
        .baud_rate    = BAUD_RATE,
        .data_bits    = UART_DATA_8_BITS,
        .parity       = UART_PARITY_DISABLE,
        .stop_bits    = UART_STOP_BITS_1,
        .flow_ctrl    = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0
    };

    ESP_LOGI(TAG,
             "SoftUART init: port=%d TX=%d RX=%d @%dbps",
             SOFT_UART_PORT, TX_PIN, RX_PIN, BAUD_RATE);

    // install driver: give TX & RX queues room for CAMERA_RX_BUFFER_SIZE bytes
    ESP_ERROR_CHECK(softuart_driver_install(
        SOFT_UART_PORT,
        CAMERA_RX_BUFFER_SIZE,
        CAMERA_RX_BUFFER_SIZE,
        0,
        NULL,
        0));

    ESP_ERROR_CHECK(softuart_param_config(SOFT_UART_PORT, &cfg));
    ESP_ERROR_CHECK(softuart_set_pin(
        SOFT_UART_PORT,
        TX_PIN,
        RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "SoftUART ready");
}

void decodeCameraData(const char *line)
{
    if (strncmp(line, "NO_FIRE", 7) == 0) {
        g_camera_data.found_fire = false;
        g_camera_data.angle_x    = 0.0f;
        g_camera_data.angle_y    = 0.0f;
        g_camera_data.pixels     = 0;
        g_camera_data.cx         = 0;
        g_camera_data.cy         = 0;
        ESP_LOGI(TAG, "Decoded: NO_FIRE");
        return;
    }

    const char *p = strstr(line, "FIRE,");
    if (!p) {
        ESP_LOGW(TAG, "Unknown format: '%s'", line);
        return;
    }

    p += 5; // skip "FIRE,"
    float fx, fy;
    int   pix, cx, cy;
    int scanned = sscanf(p, "%f,%f,%d,%d,%d", &fx, &fy, &pix, &cx, &cy);
    if (scanned == 5) {
        g_camera_data.found_fire = true;
        g_camera_data.angle_x    = fx;
        g_camera_data.angle_y    = fy;
        g_camera_data.pixels     = pix;
        g_camera_data.cx         = cx;
        g_camera_data.cy         = cy;
        ESP_LOGI(TAG,
                 "Decoded FIRE: X=%.2f Y=%.2f Pix=%d CX=%d CY=%d",
                 fx, fy, pix, cx, cy);
    } else {
        ESP_LOGW(TAG,
                 "Parse failed (%d): '%s'",
                 scanned, p);
    }
}

void uart_camera_task(void *pv)
{
    ESP_LOGI(TAG, "Camera task started");
    uint8_t ch;
    while (1) {
        int len = softuart_read_bytes(
            SOFT_UART_PORT, &ch, 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            // guard overflow
            if (s_rx_buffer_idx >= CAMERA_RX_BUFFER_SIZE - 1) {
                ESP_LOGE(TAG,
                         "RX buf overflow, discarding '%s'",
                         s_rx_buffer);
                s_rx_buffer_idx = 0;
                memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
            }
            s_rx_buffer[s_rx_buffer_idx++] = (char)ch;

            // check for "\r\n"
            if (s_rx_buffer_idx >= 2 &&
                s_rx_buffer[s_rx_buffer_idx - 2] == '\r' &&
                s_rx_buffer[s_rx_buffer_idx - 1] == '\n') {

                // terminate at '\r'
                s_rx_buffer[s_rx_buffer_idx - 2] = '\0';
                decodeCameraData(s_rx_buffer);

                // reset for next line
                s_rx_buffer_idx = 0;
                memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
            }
        } else {
            // No data received within the timeout. Task will yield.
            // ESP_LOGI(TAG, "No UART data."); // Uncomment for very verbose debugging
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
        
        // Add periodic heartbeat to show task is running (every 5 seconds)
        static int heartbeat_counter = 0;
        heartbeat_counter++;
        if (heartbeat_counter >= 5000) { // 5000 * 1ms = 5 seconds
            ESP_LOGI(TAG, "UART camera task heartbeat - still running");
            heartbeat_counter = 0;
        }
    }
}