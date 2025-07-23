#include "crsf.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 3
#endif

#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL
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
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {}
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

    ESP_LOGI(TAG, "CRSF UART initialized on port %d @ %" PRIu32 " baud (RX:%d, TX:%d)",
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

void CRSF::process_frame(bool channels_only)
{
    const uint8_t *frame = buffer;
    uint8_t len_field = frame[1];
    uint8_t type = frame[2];

    // Validate minimum frame length
    if (len_field < 2) {
        ESP_LOGW(TAG, "Frame too short: len_field=%d", len_field);
        return;
    }

    // If channels_only mode is enabled, skip non-channel frames
    if (channels_only && type != CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        ESP_LOGD(TAG, "Skipping non-channel frame type: 0x%02X", type);
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
                ESP_LOGW(TAG, "Invalid RC channels payload length: %d bytes", payload_len);
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

        case CRSF_FRAMETYPE_GPS:
        case CRSF_FRAMETYPE_VARIO:
        case CRSF_FRAMETYPE_BARO_ALT:
        case CRSF_FRAMETYPE_VIDEO_TRANSMITTER:
        case CRSF_FRAMETYPE_FLIGHT_MODE:
        case CRSF_FRAMETYPE_DEVICE_INFO:
        case CRSF_FRAMETYPE_CONFIG_READ:
        case CRSF_FRAMETYPE_CONFIG_WRITE:
        case CRSF_FRAMETYPE_RADIO_ID:
            ESP_LOGD(TAG, "Received frame type 0x%02X (not implemented)", type);
            break;

        default:
            ESP_LOGW(TAG, "Unknown frame type: 0x%02X", type);
            break;
    }
}

/* ---------------- Public Methods ---------------- */

void CRSF::poll(bool channels_only)
{
    if (!initialized) {
        ESP_LOGW(TAG, "CRSF not initialized");
        return;
    }

    // Read available bytes from UART
    size_t available = 0;
    uart_get_buffered_data_len(uart, &available);

    if (available == 0) {
        return;
    }

    uint8_t data[64];
    int read_len = uart_read_bytes(uart, data, available < sizeof(data) ? available : sizeof(data), 0);

    if (read_len <= 0) {
        return;
    }

    // Process each received byte
    for (int i = 0; i < read_len; i++) {
        uint8_t byte = data[i];

        if (idx == 0) {
            // Looking for sync byte
            if (byte == CRSF_SYNC_BYTE) {
                buffer[idx++] = byte;
            }
        } else if (idx == 1) {
            // Length field
            if (byte >= 2 && byte <= CRSF_MAX_PAYLOAD_LEN + 2) {
                buffer[idx++] = byte;
                expected_len = byte + 2; // +2 for SYNC and LEN bytes
            } else {
                // Invalid length, reset
                idx = 0;
                expected_len = 0;
            }
        } else {
            // Payload and CRC
            buffer[idx++] = byte;

            if (idx >= expected_len) {
                // Complete frame received
                process_frame(channels_only);
                idx = 0;
                expected_len = 0;
            }
        }

        // Safety check to prevent buffer overflow
        if (idx >= CRSF_MAX_FRAME_LEN) {
            ESP_LOGW(TAG, "Buffer overflow, resetting");
            idx = 0;
            expected_len = 0;
        }
    }
}

bool CRSF::getChannels(crsf_channels_t *out)
{
    if (!initialized || !out) {
        return false;
    }

    if (fresh_channels) {
        memcpy(out, &channels, sizeof(crsf_channels_t));
        fresh_channels = false;
        return true;
    }

    return false;
}

bool CRSF::sendChannels(const crsf_channels_t *channels)
{
    if (!initialized || !channels) {
        ESP_LOGW(TAG, "CRSF not initialized or null channels");
        return false;
    }

    return sendChannelsRaw(channels->channels);
}

bool CRSF::sendChannelsRaw(const uint16_t channels[16])
{
    if (!initialized || !channels) {
        ESP_LOGW(TAG, "CRSF not initialized or null channels");
        return false;
    }

    uint8_t payload[22]; // 16 channels * 11 bits = 176 bits = 22 bytes
    pack_channels(channels, payload);

    return send_frame(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, payload, sizeof(payload));
}

bool CRSF::getLinkStatistics(crsf_link_statistics_t *out)
{
    if (!initialized || !out) {
        return false;
    }

    if (fresh_link_stats) {
        memcpy(out, &link_stats, sizeof(crsf_link_statistics_t));
        fresh_link_stats = false;
        return true;
    }

    return false;
}

bool CRSF::getBattery(crsf_battery_t *out)
{
    if (!initialized || !out) {
        return false;
    }

    if (fresh_battery) {
        memcpy(out, &battery, sizeof(crsf_battery_t));
        fresh_battery = false;
        return true;
    }

    return false;
}

bool CRSF::getAttitude(crsf_attitude_t *out)
{
    if (!initialized || !out) {
        return false;
    }

    if (fresh_attitude) {
        memcpy(out, &attitude, sizeof(crsf_attitude_t));
        fresh_attitude = false;
        return true;
    }

    return false;
}

uint16_t CRSF::toMicroseconds(uint16_t crsf_value)
{
    // Convert CRSF value (172-1811) to microseconds (988-2012)
    // Linear mapping: us = 988 + (crsf - 172) * (2012 - 988) / (1811 - 172)
    if (crsf_value < CRSF_CHANNEL_MIN) {
        return 988;
    }
    if (crsf_value > CRSF_CHANNEL_MAX) {
        return 2012;
    }

    return 988 + ((crsf_value - CRSF_CHANNEL_MIN) * 1024) / CRSF_CHANNEL_RANGE;
}

uint16_t CRSF::fromMicroseconds(uint16_t us_value)
{
    // Convert microseconds (988-2012) to CRSF value (172-1811)
    if (us_value < 988) {
        return CRSF_CHANNEL_MIN;
    }
    if (us_value > 2012) {
        return CRSF_CHANNEL_MAX;
    }

    return CRSF_CHANNEL_MIN + ((us_value - 988) * CRSF_CHANNEL_RANGE) / 1024;
}

/* ---------------- Legacy C API Implementation ---------------- */

// Global CRSF instance for legacy C API
static CRSF* legacy_crsf = nullptr;

void crsf_init(uart_port_t uart_num, int rx_pin, int tx_pin, uint32_t baud)
{
    if (legacy_crsf) {
        delete legacy_crsf;
    }
    legacy_crsf = new CRSF(uart_num, rx_pin, tx_pin, baud);
}

void crsf_poll(void)
{
    if (legacy_crsf) {
        legacy_crsf->poll();
    }
}

bool crsf_get_channels(crsf_channels_t *out)
{
    if (legacy_crsf) {
        return legacy_crsf->getChannels(out);
    }
    return false;
}

bool crsf_send_channels(const crsf_channels_t *channels)
{
    if (legacy_crsf) {
        return legacy_crsf->sendChannels(channels);
    }
    return false;
}
