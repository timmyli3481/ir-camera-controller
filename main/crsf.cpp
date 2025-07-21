#include "crsf.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 3
#endif

#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL
#endif

#ifndef ESP_LOG_LEVEL
#define ESP_LOG_LEVEL LOG_LOCAL_LEVEL
#endif

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_err.h"

static const char *TAG = "CRSF";

/* ---------------- CRSF Class Implementation ---------------- */

// Static registry for UART->CRSF mapping
CRSF* CRSF::uart_registry[UART_NUM_MAX] = {nullptr};

// Constructor with full UART initialization
CRSF::CRSF(uart_port_t uart_num, int rx_pin, int tx_pin, uint32_t baud) 
    : uart(uart_num), initialized(false), idx(0), expected_len(0), 
      fresh_channels(false), fresh_link_stats(false), fresh_battery(false), fresh_attitude(false)
{
    // Clear buffers and structures
    memset(buffer, 0, sizeof(buffer));
    memset(&channels, 0, sizeof(channels));
    memset(&link_stats, 0, sizeof(link_stats));
    memset(&battery, 0, sizeof(battery));
    memset(&attitude, 0, sizeof(attitude));

    if (baud == 0) {
        baud = CRSF_UART_BAUD_DEFAULT;
    }

    uart_config_t cfg = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;
    err = uart_driver_install(uart_num, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
        return;
    }

    err = uart_param_config(uart_num, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(err));
        uart_driver_delete(uart_num);
        return;
    }

    err = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
        uart_driver_delete(uart_num);
        return;
    }

    // Register this instance in the static registry
    uart_registry[uart_num] = this;
    initialized = true;

    ESP_LOGI(TAG, "CRSF UART initialized on port %d @ %u baud (RX:%d, TX:%d)", 
             uart_num, baud, rx_pin, tx_pin);
}

// Constructor using existing UART configuration
CRSF::CRSF(uart_port_t uart_num) 
    : uart(uart_num), initialized(false), idx(0), expected_len(0),
      fresh_channels(false), fresh_link_stats(false), fresh_battery(false), fresh_attitude(false)
{
    // Clear buffers and structures
    memset(buffer, 0, sizeof(buffer));
    memset(&channels, 0, sizeof(channels));
    memset(&link_stats, 0, sizeof(link_stats));
    memset(&battery, 0, sizeof(battery));
    memset(&attitude, 0, sizeof(attitude));

    // Check if UART was already initialized
    if (uart_num >= UART_NUM_MAX || uart_registry[uart_num] == nullptr) {
        ESP_LOGE(TAG, "UART %d not previously initialized. Use full constructor first.", uart_num);
        return;
    }

    initialized = true;
    ESP_LOGI(TAG, "CRSF object created for existing UART %d", uart_num);
}

// Destructor
CRSF::~CRSF()
{
    if (initialized && uart < UART_NUM_MAX) {
        // Only delete UART driver if this was the initializing instance
        if (uart_registry[uart] == this) {
            uart_driver_delete(uart);
            uart_registry[uart] = nullptr;
            ESP_LOGI(TAG, "CRSF UART %d driver deleted", uart);
        }
    }
}

/* ---------------- Internal Helper Methods ---------------- */

uint8_t CRSF::crc8_dvb_s2_byte(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

uint8_t CRSF::crc8_dvb_s2(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i) {
        crc = crc8_dvb_s2_byte(crc, data[i]);
    }
    return crc;
}

void CRSF::unpack_channels(const uint8_t *payload, uint16_t *dest)
{
    const uint32_t mask = (1u << 11) - 1u; // 0x7FF
    uint32_t acc = 0;
    uint8_t bits_in_acc = 0;

    for (int ch = 0, i = 0; ch < 16; ++ch) {
        while (bits_in_acc < 11) {
            acc |= (uint32_t)payload[i++] << bits_in_acc;
            bits_in_acc += 8;
        }
        dest[ch] = acc & mask;
        acc >>= 11;
        bits_in_acc -= 11;
    }
}

void CRSF::pack_channels(const uint16_t *src, uint8_t *payload)
{
    const uint32_t mask = (1u << 11) - 1u; // 0x7FF
    uint32_t acc = 0;
    uint8_t bits_in_acc = 0;
    uint8_t byte_idx = 0;

    for (int ch = 0; ch < 16; ++ch) {
        uint32_t value = src[ch] & mask; // Ensure 11-bit value
        acc |= value << bits_in_acc;
        bits_in_acc += 11;

        while (bits_in_acc >= 8) {
            payload[byte_idx++] = (uint8_t)(acc & 0xFF);
            acc >>= 8;
            bits_in_acc -= 8;
        }
    }

    // Write any remaining bits
    if (bits_in_acc > 0) {
        payload[byte_idx] = (uint8_t)(acc & 0xFF);
    }
}

bool CRSF::send_frame(uint8_t frame_type, const uint8_t *payload, uint8_t payload_len)
{
    if (!initialized) {
        ESP_LOGW(TAG, "CRSF not initialized");
        return false;
    }

    if (payload_len > CRSF_MAX_PAYLOAD_LEN) {
        ESP_LOGW(TAG, "Payload too large: %d bytes", payload_len);
        return false;
    }

    uint8_t frame[CRSF_MAX_FRAME_LEN];
    uint8_t frame_len = 0;

    // Build frame: SYNC + LEN + TYPE + PAYLOAD + CRC
    frame[frame_len++] = CRSF_SYNC_BYTE;
    frame[frame_len++] = payload_len + 2; // LEN = TYPE + PAYLOAD + CRC
    frame[frame_len++] = frame_type;

    // Copy payload
    if (payload && payload_len > 0) {
        memcpy(&frame[frame_len], payload, payload_len);
        frame_len += payload_len;
    }

    // Calculate CRC over TYPE + PAYLOAD
    uint8_t crc = crc8_dvb_s2(&frame[2], payload_len + 1);
    frame[frame_len++] = crc;

    // Send frame
    int written = uart_write_bytes(uart, frame, frame_len);
    if (written == frame_len) {
        ESP_LOGD(TAG, "Sent frame type 0x%02X, len %d", frame_type, frame_len);
        return true;
    } else {
        ESP_LOGW(TAG, "Failed to send frame (wrote %d/%d bytes)", written, frame_len);
        return false;
    }
}

void CRSF::process_link_statistics(const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len < 10) {
        ESP_LOGW(TAG, "Link statistics payload too short: %d bytes", payload_len);
        return;
    }

    link_stats.rssi1 = (int8_t)payload[0];
    link_stats.rssi2 = (int8_t)payload[1];
    link_stats.link_quality = payload[2];
    link_stats.snr = (int8_t)payload[3];
    link_stats.antenna = payload[4] & 0x01;
    link_stats.rf_mode = payload[5];
    link_stats.tx_power = payload[6];
    link_stats.downlink_rssi = (int8_t)payload[7];
    link_stats.downlink_lq = payload[8];
    link_stats.downlink_snr = (int8_t)payload[9];
    
    fresh_link_stats = true;
    ESP_LOGD(TAG, "Link stats: RSSI %d/%d, LQ %d%%, SNR %d", 
             link_stats.rssi1, link_stats.rssi2, link_stats.link_quality, link_stats.snr);
}

void CRSF::process_battery_sensor(const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len < 8) {
        ESP_LOGW(TAG, "Battery sensor payload too short: %d bytes", payload_len);
        return;
    }

    // Parse battery frame: voltage (16-bit), current (16-bit), capacity (24-bit), percentage (8-bit)
    uint16_t voltage_raw = (payload[1] << 8) | payload[0];
    uint16_t current_raw = (payload[3] << 8) | payload[2];
    uint32_t capacity_raw = (payload[6] << 16) | (payload[5] << 8) | payload[4];
    uint8_t percentage = payload[7];

    battery.voltage = voltage_raw / 100.0f; // Convert from centi-volts
    battery.current = current_raw / 100.0f; // Convert from centi-amps
    battery.capacity = capacity_raw;
    battery.percentage = percentage;
    
    fresh_battery = true;
    ESP_LOGD(TAG, "Battery: %.2fV, %.2fA, %dmAh, %d%%", 
             battery.voltage, battery.current, 
             (int)battery.capacity, battery.percentage);
}

void CRSF::process_attitude(const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len < 6) {
        ESP_LOGW(TAG, "Attitude payload too short: %d bytes", payload_len);
        return;
    }

    // Parse attitude frame: pitch (16-bit), roll (16-bit), yaw (16-bit) in radians * 10000
    int16_t pitch_raw = (payload[1] << 8) | payload[0];
    int16_t roll_raw = (payload[3] << 8) | payload[2];
    int16_t yaw_raw = (payload[5] << 8) | payload[4];

    attitude.pitch = pitch_raw / 10000.0f;
    attitude.roll = roll_raw / 10000.0f;
    attitude.yaw = yaw_raw / 10000.0f;
    
    fresh_attitude = true;
    ESP_LOGD(TAG, "Attitude: P=%.3f, R=%.3f, Y=%.3f", 
             attitude.pitch, attitude.roll, attitude.yaw);
}

void CRSF::process_frame(void)
{
    const uint8_t *frame = buffer;
    uint8_t len_field = frame[1];
    uint8_t type = frame[2];

    // Validate minimum frame length
    if (len_field < 2) {
        ESP_LOGW(TAG, "Frame too short: len_field=%d", len_field);
        return;
    }

    // Check CRC (covers TYPE..PAYLOAD)
    uint8_t crc_calc = crc8_dvb_s2(&frame[2], len_field - 1); // minus CRC byte itself
    uint8_t crc_frame = frame[len_field + 1];

    if (crc_calc != crc_frame) {
        ESP_LOGW(TAG, "CRC mismatch (type 0x%02X): calc=0x%02X, frame=0x%02X", 
                 type, crc_calc, crc_frame);
        return;
    }

    const uint8_t *payload = &frame[3];
    uint8_t payload_len = len_field - 2; // minus TYPE and CRC

    // Process frame based on type
    switch (type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            if (payload_len == 22) {
                unpack_channels(payload, channels.channels);
                channels.timestamp_us = esp_timer_get_time();
                fresh_channels = true;
                ESP_LOGD(TAG, "Received RC channels");
            } else {
                ESP_LOGW(TAG, "Invalid RC channels payload length: %d", payload_len);
            }
            break;

        case CRSF_FRAMETYPE_LINK_STATISTICS:
            process_link_statistics(payload, payload_len);
            break;

        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            process_battery_sensor(payload, payload_len);
            break;

        case CRSF_FRAMETYPE_ATTITUDE:
            process_attitude(payload, payload_len);
            break;

        case CRSF_FRAMETYPE_HEARTBEAT:
            ESP_LOGD(TAG, "Received heartbeat");
            break;

        default:
            ESP_LOGD(TAG, "Unhandled frame type: 0x%02X", type);
            break;
    }
}

/* ---------------- Public API Methods ---------------- */

void CRSF::poll(void)
{
    if (!initialized) {
        return;
    }

    // Read all pending bytes from UART and feed parser
    uint8_t rx_buf[128];
    int rx_len = 0;
    do {
        rx_len = uart_read_bytes(uart, rx_buf, sizeof(rx_buf), 0);
        for (int i = 0; i < rx_len; ++i) {
            uint8_t byte = rx_buf[i];

            if (idx == 0) {
                // Waiting for SYNC
                if (byte == CRSF_SYNC_BYTE) {
                    buffer[idx++] = byte;
                }
                continue;
            }

            // Save byte
            buffer[idx++] = byte;

            if (idx == 2) {
                // We just stored LEN byte
                uint8_t len_field = byte;
                if (len_field < 2 || len_field > CRSF_MAX_PAYLOAD_LEN) {
                    // Invalid length, reset
                    ESP_LOGD(TAG, "Invalid length field: %d", len_field);
                    idx = 0;
                    continue;
                }
                expected_len = len_field + 2; // total bytes (SYNC+LEN) + len_field
            }

            if (expected_len && idx >= expected_len + 1) {
                // +1 for CRC byte
                process_frame();
                // Reset for next frame
                idx = 0;
                expected_len = 0;
            }

            if (idx >= sizeof(buffer)) {
                // Buffer overflow protection
                ESP_LOGW(TAG, "Buffer overflow, resetting");
                idx = 0;
                expected_len = 0;
            }
        }
    } while (rx_len > 0);
}

bool CRSF::getChannels(crsf_channels_t *out)
{
    if (!out || !initialized) {
        return false;
    }
    if (!fresh_channels) {
        return false;
    }
    memcpy(out, &channels, sizeof(crsf_channels_t));
    fresh_channels = false;
    return true;
}

bool CRSF::sendChannels(const crsf_channels_t *channels)
{
    if (!channels) {
        return false;
    }
    return sendChannelsRaw(channels->channels);
}

bool CRSF::sendChannelsRaw(const uint16_t channels[16])
{
    if (!channels || !initialized) {
        return false;
    }

    // Validate channel values are within valid range
    for (int i = 0; i < 16; i++) {
        if (channels[i] > 1984) { // 11-bit max value
            ESP_LOGW(TAG, "Channel %d value %d exceeds 11-bit range", i, channels[i]);
            return false;
        }
    }

    // Pack channels into 22-byte payload
    uint8_t payload[22] = {0};
    pack_channels(channels, payload);

    // Send RC channels frame
    return send_frame(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, payload, sizeof(payload));
}

bool CRSF::getLinkStatistics(crsf_link_statistics_t *out)
{
    if (!out || !initialized) {
        return false;
    }
    if (!fresh_link_stats) {
        return false;
    }
    memcpy(out, &link_stats, sizeof(crsf_link_statistics_t));
    fresh_link_stats = false;
    return true;
}

bool CRSF::getBattery(crsf_battery_t *out)
{
    if (!out || !initialized) {
        return false;
    }
    if (!fresh_battery) {
        return false;
    }
    memcpy(out, &battery, sizeof(crsf_battery_t));
    fresh_battery = false;
    return true;
}

bool CRSF::getAttitude(crsf_attitude_t *out)
{
    if (!out || !initialized) {
        return false;
    }
    if (!fresh_attitude) {
        return false;
    }
    memcpy(out, &attitude, sizeof(crsf_attitude_t));
    fresh_attitude = false;
    return true;
}

uint16_t CRSF::toMicroseconds(uint16_t crsf_value)
{
    // Convert CRSF value (172-1811) to microseconds (988-2012)
    if (crsf_value < CRSF_CHANNEL_MIN) {
        crsf_value = CRSF_CHANNEL_MIN;
    } else if (crsf_value > CRSF_CHANNEL_MAX) {
        crsf_value = CRSF_CHANNEL_MAX;
    }
    
    // Linear interpolation: 172-1811 -> 988-2012
    return 988 + ((crsf_value - CRSF_CHANNEL_MIN) * 1024) / CRSF_CHANNEL_RANGE;
}

uint16_t CRSF::fromMicroseconds(uint16_t us_value)
{
    // Convert microseconds (988-2012) to CRSF value (172-1811)
    if (us_value < 988) {
        us_value = 988;
    } else if (us_value > 2012) {
        us_value = 2012;
    }
    
    // Linear interpolation: 988-2012 -> 172-1811
    return CRSF_CHANNEL_MIN + ((us_value - 988) * CRSF_CHANNEL_RANGE) / 1024;
}

/* ---------------- Legacy C API Implementation ---------------- */

// Global instance for legacy C API
static CRSF* g_legacy_crsf = nullptr;

extern "C" {

void crsf_init(uart_port_t uart_num, int rx_pin, int tx_pin, uint32_t baud)
{
    if (g_legacy_crsf) {
        delete g_legacy_crsf;
    }
    g_legacy_crsf = new CRSF(uart_num, rx_pin, tx_pin, baud);
}

void crsf_poll(void)
{
    if (g_legacy_crsf) {
        g_legacy_crsf->poll();
    }
}

bool crsf_get_channels(crsf_channels_t *out)
{
    if (g_legacy_crsf) {
        return g_legacy_crsf->getChannels(out);
    }
    return false;
}

bool crsf_send_channels(const crsf_channels_t *channels)
{
    if (g_legacy_crsf) {
        return g_legacy_crsf->sendChannels(channels);
    }
    return false;
}

bool crsf_send_channels_raw(const uint16_t channels[16])
{
    if (g_legacy_crsf) {
        return g_legacy_crsf->sendChannelsRaw(channels);
    }
    return false;
}

bool crsf_get_link_statistics(crsf_link_statistics_t *out)
{
    if (g_legacy_crsf) {
        return g_legacy_crsf->getLinkStatistics(out);
    }
    return false;
}

bool crsf_get_battery(crsf_battery_t *out)
{
    if (g_legacy_crsf) {
        return g_legacy_crsf->getBattery(out);
    }
    return false;
}

bool crsf_get_attitude(crsf_attitude_t *out)
{
    if (g_legacy_crsf) {
        return g_legacy_crsf->getAttitude(out);
    }
    return false;
}

uint16_t crsf_to_us(uint16_t crsf_value)
{
    return CRSF::toMicroseconds(crsf_value);
}

uint16_t us_to_crsf(uint16_t us_value)
{
    return CRSF::fromMicroseconds(us_value);
}

} // extern "C"

