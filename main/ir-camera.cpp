#include <ir-camera.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For atof, atoi

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "hal/gpio_types.h"
#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "CAMERA_PARSER";
static const int TX_PIN = GPIO_NUM_18;
static const int RX_PIN = GPIO_NUM_5;
static const int BAUD_RATE = 9600;

// Global instance of camera data, accessible from other modules
camera_data_t g_camera_data = {
    .found_fire = false,
    .angle_x = 0.0f,
    .angle_y = 0.0f,
    .pixels = 0,
    .cx = 0,
    .cy = 0,
};

// Internal buffer for receiving UART data
static char s_rx_buffer[CAMERA_RX_BUFFER_SIZE];
static int s_rx_buffer_idx = 0;

void uart_camera_init() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0, // Not used when flow_ctrl is disabled
    };
    
    ESP_LOGI(TAG, "UART configuration: Baud Rate = %d, Data Bits = %d, Parity = %d, Stop Bits = %d",
             uart_config.baud_rate, uart_config.data_bits, uart_config.parity, uart_config.stop_bits);
    
    esp_err_t err;
    
    err = uart_param_config(UART_PORT, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(err));
        ESP_ERROR_CHECK(err); // Consider handling this more gracefully if not a critical error
    } else {
        ESP_LOGI(TAG, "UART parameters configured successfully for port %d.", UART_PORT);
    }
    
    err = uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins (TX: %d, RX: %d): %s", TX_PIN, RX_PIN, esp_err_to_name(err));
        ESP_ERROR_CHECK(err);
    } else {
        ESP_LOGI(TAG, "UART pins set successfully for port %d (TX: %d, RX: %d).", UART_PORT, TX_PIN, RX_PIN);
    }
    
    err = uart_driver_install(UART_PORT, CAMERA_RX_BUFFER_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver for port %d with RX buffer size %d: %s", UART_PORT, CAMERA_RX_BUFFER_SIZE, esp_err_to_name(err));
        ESP_ERROR_CHECK(err);
    } else {
        ESP_LOGI(TAG, "UART driver installed successfully for port %d.", UART_PORT);
    }
    
    ESP_LOGI(TAG, "UART camera task initialization complete.");
}

void decodeCameraData(const char *data_to_decode) {

    if (strncmp(data_to_decode, "NO_FIRE", strlen("NO_FIRE")) == 0) {
        g_camera_data.found_fire = false;
        // Reset other values if no fire
        g_camera_data.angle_x = 0.0f;
        g_camera_data.angle_y = 0.0f;
        g_camera_data.pixels = 0;
        g_camera_data.cx = 0;
        g_camera_data.cy = 0;
        ESP_LOGI(TAG, "Decoded: NO_FIRE");
    } else {
        // Look for "FIRE," anywhere in the string (handles cases like "5FIRE,")
        const char *fire_pos = strstr(data_to_decode, "FIRE,");
        if (fire_pos != NULL) {
            // Expected format: "FIRE,X.XX,Y.YY,PIXELS,CX,CY \r\n"
            // Use sscanf to parse the string
            float angle_x, angle_y;
            int pixels, cx, cy;

            // Skip "FIRE," and parse the rest
            const char *payload = fire_pos + strlen("FIRE,");
            int scanned_items = sscanf(payload, "%f,%f,%d,%d,%d",
                                       &angle_x, &angle_y, &pixels, &cx, &cy);

            if (scanned_items == 5) {
                g_camera_data.found_fire = true;
                g_camera_data.angle_x = angle_x;
                g_camera_data.angle_y = angle_y;
                g_camera_data.pixels = pixels;
                g_camera_data.cx = cx;
                g_camera_data.cy = cy;
                ESP_LOGI(TAG, "Decoded: FIRE X=%.2f, Y=%.2f, Pixels=%d, CX=%d, CY=%d",
                         g_camera_data.angle_x, g_camera_data.angle_y,
                         g_camera_data.pixels, g_camera_data.cx, g_camera_data.cy);
            } else {
                ESP_LOGW(TAG, "Failed to parse FIRE data: '%s'. Scanned items: %d", payload, scanned_items);
            }
        } else {
            ESP_LOGW(TAG, "Unknown camera data format: '%s'", data_to_decode);
        }
    }
}

void uart_camera_task(void *pvParameters) {
    esp_log_level_set(TAG, ESP_LOG_DEBUG); // Set log level for this task/file
    uint8_t temp_buffer[1]; // Read one character at a time

    ESP_LOGI(TAG, "UART camera task started.");

    while (1) {
        // Read one character at a time
        // The timeout (20ms) allows the task to yield if no data is present,
        // preventing a busy-wait and allowing other tasks to run.
        int len = uart_read_bytes(UART_PORT, temp_buffer, 1, 20 / portTICK_PERIOD_MS);

        if (len > 0) {
            char received_char = (char)temp_buffer[0];
            // ESP_LOGI(TAG, "Received character: '%c'", received_char);

            // Check for buffer overflow before adding character
            if (s_rx_buffer_idx >= (CAMERA_RX_BUFFER_SIZE - 1)) {
                ESP_LOGE(TAG, "UART RX buffer overflow! Data: '%s'", s_rx_buffer);
                // Clear buffer and reset index to prevent further issues
                s_rx_buffer_idx = 0;
                memset(s_rx_buffer, 0, CAMERA_RX_BUFFER_SIZE);
            }

            s_rx_buffer[s_rx_buffer_idx++] = received_char;

            // Check for line ending (\r\n)
            if (s_rx_buffer_idx >= 2 &&
                s_rx_buffer[s_rx_buffer_idx - 2] == '\r' &&
                s_rx_buffer[s_rx_buffer_idx - 1] == '\n') {

                // Null-terminate the string before passing to decode function
                s_rx_buffer[s_rx_buffer_idx - 2] = '\0'; // Overwrite \r
                // s_rx_buffer[s_rx_buffer_idx - 1] = '\0'; // Overwrite \n (already overwritten by '\0')

                // ESP_LOGI(TAG, "Received complete line: '%s'", s_rx_buffer);

                // Call the decode function with the complete line
                decodeCameraData(s_rx_buffer);

                // Reset buffer index for the next message
                s_rx_buffer_idx = 0;
                memset(s_rx_buffer, 0, CAMERA_RX_BUFFER_SIZE); // Clear buffer
            }
        } else {
            // No data received within the timeout. Task will yield.
            ESP_LOGI(TAG, "No UART data."); // Uncomment for very verbose debugging
        }

        // A small delay is still useful if the timeout is short and data is very sparse,
        // but `uart_read_bytes` timeout already handles yielding.
        // It's often good practice to have some form of `vTaskDelay` in a loop,
        // but removing it here if `uart_read_bytes` has a good timeout is fine.
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