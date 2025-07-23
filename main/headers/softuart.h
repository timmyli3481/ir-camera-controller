#ifndef SOFTUART_H
#define SOFTUART_H

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Install a software UART driver.
 * @param uart_num        Software‐UART port number (0..UART_NUM_MAX-1)
 * @param rx_buffer_size  Depth of the RX queue (bytes)
 * @param tx_buffer_size  Depth of the TX queue (bytes)
 * @param queue_size      Unused (no event queue)
 * @param uart_queue      Unused (will be set to NULL)
 * @param intr_alloc_flags Passed to gpio_install_isr_service()
 */
esp_err_t softuart_driver_install(uart_port_t uart_num,
                                  int rx_buffer_size,
                                  int tx_buffer_size,
                                  int queue_size,
                                  QueueHandle_t *uart_queue,
                                  int intr_alloc_flags);

/**
 * @brief Delete a software UART driver and free resources.
 */
esp_err_t softuart_driver_delete(uart_port_t uart_num);

/**
 * @brief Returns true if driver is installed.
 */
bool softuart_is_driver_installed(uart_port_t uart_num);

/**
 * @brief Configure UART frame parameters (only 8-N-1 supported).
 */
esp_err_t softuart_param_config(uart_port_t uart_num,
                                const uart_config_t *uart_config);

/**
 * @brief Assign TX/RX pins (rts/cts ignored).
 */
esp_err_t softuart_set_pin(uart_port_t uart_num,
                           int tx_io_num,
                           int rx_io_num,
                           int rts_io_num,
                           int cts_io_num);

/**
 * @brief Write bytes to software UART (blocks until queued).
 * @return number of bytes queued, or -1 on error.
 */
int softuart_write_bytes(uart_port_t uart_num,
                         const void *src,
                         size_t size);

/**
 * @brief Read bytes from software UART.
 * @return number of bytes read (≤ length), or -1 on error.
 */
int softuart_read_bytes(uart_port_t uart_num,
                        void *buf,
                        uint32_t length,
                        TickType_t ticks_to_wait);

/**
 * @brief Discard any pending RX data.
 */
esp_err_t softuart_flush_input(uart_port_t uart_num);

/**
 * @brief Get number of bytes waiting in RX queue.
 */
esp_err_t softuart_get_buffered_data_len(uart_port_t uart_num,
                                         size_t *size);

#ifdef __cplusplus
}
#endif

#endif // SOFTUART_H