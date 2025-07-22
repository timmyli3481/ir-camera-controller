#include "softuart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <rom/ets_sys.h>   // for ets_delay_us()
#include <string.h>

typedef struct {
    bool            installed;
    int             rx_buffer_size;
    int             tx_buffer_size;
    QueueHandle_t   rx_queue;
    QueueHandle_t   tx_queue;
    uart_config_t   config;
    int             intr_alloc_flags;
    gpio_num_t      tx_io_num;
    gpio_num_t      rx_io_num;
    TaskHandle_t    tx_task_handle;
} softuart_t;

static softuart_t s_softuarts[UART_NUM_MAX] = {0};

// TX task: dequeues bytes and bit-bangs them on TX pin
static void softuart_tx_task(void *arg)
{
    uart_port_t   u = (uart_port_t)(uintptr_t)arg;
    softuart_t  *su = &s_softuarts[u];
    uint8_t       data;
    int           bit_time_us;

    while (1) {
        if (xQueueReceive(su->tx_queue, &data, portMAX_DELAY) == pdTRUE) {
            bit_time_us = 1000000 / su->config.baud_rate;
            // start bit (low)
            gpio_set_level(su->tx_io_num, 0);
            ets_delay_us(bit_time_us);
            // 8 data bits, LSB first
            for (int i = 0; i < 8; i++) {
                gpio_set_level(su->tx_io_num, (data >> i) & 1);
                ets_delay_us(bit_time_us);
            }
            // stop bit (high)
            gpio_set_level(su->tx_io_num, 1);
            ets_delay_us(bit_time_us *
                         ((su->config.stop_bits == UART_STOP_BITS_2) ? 2 : 1));
        }
    }
    vTaskDelete(NULL);
}

// RX ISR: on falling edge (start bit), sample at mid-bit intervals
static void IRAM_ATTR softuart_rx_isr(void *arg)
{
    uart_port_t   u  = (uart_port_t)(uintptr_t)arg;
    softuart_t  *su = &s_softuarts[u];
    uint32_t      bt = 1000000 / su->config.baud_rate;
    uint8_t       data = 0;

    // disable further IRQs until done sampling
    gpio_intr_disable(su->rx_io_num);

    // wait 1.5 bits to center of bit0
    ets_delay_us(bt + bt/2);
    for (int i = 0; i < 8; i++) {
        if (gpio_get_level(su->rx_io_num)) {
            data |= (1 << i);
        }
        ets_delay_us(bt);
    }
    // skip stop bit
    ets_delay_us(bt);

    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(su->rx_queue, &data, &hp);

    // re-enable IRQ on start bit
    gpio_intr_enable(su->rx_io_num);
    if (hp) portYIELD_FROM_ISR();
}

esp_err_t softuart_driver_install(uart_port_t     uart_num,
                                  int             rx_buffer_size,
                                  int             tx_buffer_size,
                                  int             queue_size,
                                  QueueHandle_t  *uart_queue,
                                  int             intr_alloc_flags)
{
    if (uart_num >= UART_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    softuart_t *su = &s_softuarts[uart_num];
    if (su->installed) {
        return ESP_ERR_INVALID_STATE;
    }
    su->rx_buffer_size    = rx_buffer_size;
    su->tx_buffer_size    = tx_buffer_size;
    su->intr_alloc_flags  = intr_alloc_flags;

    su->rx_queue = xQueueCreate(rx_buffer_size, sizeof(uint8_t));
    if (!su->rx_queue) {
        return ESP_ERR_NO_MEM;
    }
    su->tx_queue = xQueueCreate(tx_buffer_size, sizeof(uint8_t));
    if (!su->tx_queue) {
        vQueueDelete(su->rx_queue);
        return ESP_ERR_NO_MEM;
    }

    // we do not support the UART event queue here
    if (uart_queue) {
        *uart_queue = NULL;
    }

    su->installed = true;
    return ESP_OK;
}

esp_err_t softuart_driver_delete(uart_port_t uart_num)
{
    if (uart_num >= UART_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    softuart_t *su = &s_softuarts[uart_num];
    if (!su->installed) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_isr_handler_remove(su->rx_io_num);
    vTaskDelete(su->tx_task_handle);
    vQueueDelete(su->rx_queue);
    vQueueDelete(su->tx_queue);
    memset(su, 0, sizeof(*su));

    return ESP_OK;
}

bool softuart_is_driver_installed(uart_port_t uart_num)
{
    if (uart_num >= UART_NUM_MAX) {
        return false;
    }
    return s_softuarts[uart_num].installed;
}

esp_err_t softuart_param_config(uart_port_t           uart_num,
                                const uart_config_t  *cfg)
{
    if (uart_num >= UART_NUM_MAX || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    softuart_t *su = &s_softuarts[uart_num];
    if (!su->installed) {
        return ESP_ERR_INVALID_STATE;
    }
    // only 8 data bits, no parity, 1 or 2 stop bits
    if (cfg->data_bits != UART_DATA_8_BITS ||
        cfg->parity    != UART_PARITY_DISABLE ||
       (cfg->stop_bits != UART_STOP_BITS_1 &&
        cfg->stop_bits != UART_STOP_BITS_2)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    su->config = *cfg;
    return ESP_OK;
}

esp_err_t softuart_set_pin(uart_port_t uart_num,
                           int         tx_io_num,
                           int         rx_io_num,
                           int         rts_io_num,
                           int         cts_io_num)
{
    if (uart_num >= UART_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    softuart_t *su = &s_softuarts[uart_num];
    if (!su->installed) {
        return ESP_ERR_INVALID_STATE;
    }

    su->tx_io_num = tx_io_num;
    su->rx_io_num = rx_io_num;

    // configure TX pin as output, idle high
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << tx_io_num,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(tx_io_num, 1);

    // configure RX pin as input, pull-up, falling-edge IRQ
    io_conf.intr_type    = GPIO_INTR_NEGEDGE;
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << rx_io_num;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // install GPIO ISR service once
    gpio_install_isr_service(su->intr_alloc_flags);
    gpio_isr_handler_add(rx_io_num, softuart_rx_isr, (void*)(uintptr_t)uart_num);

    // start TX task
    xTaskCreate(softuart_tx_task,
                "softuart_tx",
                1024,
                (void*)(uintptr_t)uart_num,
                tskIDLE_PRIORITY + 1,
                &su->tx_task_handle);

    return ESP_OK;
}

int softuart_write_bytes(uart_port_t      uart_num,
                         const void      *src,
                         size_t           size)
{
    if (uart_num >= UART_NUM_MAX || src == NULL) {
        return -1;
    }
    softuart_t *su = &s_softuarts[uart_num];
    if (!su->installed) {
        return -1;
    }
    const uint8_t *d = src;
    size_t written = 0;
    for (size_t i = 0; i < size; i++) {
        if (xQueueSend(su->tx_queue, &d[i], portMAX_DELAY) == pdTRUE) {
            written++;
        }
    }
    return (int)written;
}

int softuart_read_bytes(uart_port_t  uart_num,
                        void        *buf,
                        uint32_t     length,
                        TickType_t   ticks_to_wait)
{
    if (uart_num >= UART_NUM_MAX || buf == NULL) {
        return -1;
    }
    softuart_t *su = &s_softuarts[uart_num];
    if (!su->installed) {
        return -1;
    }
    uint8_t *d = buf;
    uint32_t r = 0;
    TickType_t start = xTaskGetTickCount();
    for (; r < length; r++) {
        TickType_t dt = xTaskGetTickCount() - start;
        TickType_t to = (ticks_to_wait > dt) ? (ticks_to_wait - dt) : 0;
        if (xQueueReceive(su->rx_queue, &d[r], to) != pdTRUE) {
            break;
        }
    }
    return (int)r;
}

esp_err_t softuart_flush_input(uart_port_t uart_num)
{
    if (uart_num >= UART_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    softuart_t *su = &s_softuarts[uart_num];
    if (!su->installed) {
        return ESP_ERR_INVALID_STATE;
    }
    xQueueReset(su->rx_queue);
    return ESP_OK;
}

esp_err_t softuart_get_buffered_data_len(uart_port_t uart_num, size_t *size)
{
    if (uart_num >= UART_NUM_MAX || size == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    softuart_t *su = &s_softuarts[uart_num];
    if (!su->installed) {
        return ESP_ERR_INVALID_STATE;
    }
    *size = uxQueueMessagesWaiting(su->rx_queue);
    return ESP_OK;
}