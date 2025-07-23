#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CRSF Protocol Constants */
#define CRSF_SYNC_BYTE                  0xC8
#define CRSF_MAX_FRAME_LEN              64
#define CRSF_MAX_PAYLOAD_LEN            60
#define CRSF_UART_BAUD_DEFAULT          420000

/* CRSF Frame Types */
#define CRSF_FRAMETYPE_GPS              0x02
#define CRSF_FRAMETYPE_VARIO            0x07
#define CRSF_FRAMETYPE_BATTERY_SENSOR   0x08
#define CRSF_FRAMETYPE_BARO_ALT         0x09
#define CRSF_FRAMETYPE_HEARTBEAT        0x0B
#define CRSF_FRAMETYPE_VIDEO_TRANSMITTER 0x0F
#define CRSF_FRAMETYPE_LINK_STATISTICS  0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_ATTITUDE         0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE      0x21
#define CRSF_FRAMETYPE_DEVICE_INFO      0x29
#define CRSF_FRAMETYPE_CONFIG_READ      0x2C
#define CRSF_FRAMETYPE_CONFIG_WRITE     0x2D
#define CRSF_FRAMETYPE_RADIO_ID         0x3A

/* Channel constants */
#define CRSF_CHANNEL_COUNT              16
#define CRSF_CHANNEL_MIN                172   // Min channel value (microseconds: 988us)
#define CRSF_CHANNEL_CENTER             992   // Center channel value (microseconds: 1500us)
#define CRSF_CHANNEL_MAX                1811  // Max channel value (microseconds: 2012us)
#define CRSF_CHANNEL_RANGE              1639  // (1811 - 172)

/* Data Structures */
typedef struct {
    uint16_t channels[CRSF_CHANNEL_COUNT];  // Raw CRSF channel values (172-1811)
    uint64_t timestamp_us;                  // Timestamp when channels were received
} crsf_channels_t;

typedef struct {
    int8_t rssi1;
    int8_t rssi2;
    uint8_t link_quality;
    int8_t snr;
    uint8_t antenna;
    uint8_t rf_mode;
    uint8_t tx_power;
    int8_t downlink_rssi;
    uint8_t downlink_lq;
    int8_t downlink_snr;
} crsf_link_statistics_t;

typedef struct {
    float voltage;      // Battery voltage in volts
    float current;      // Current in amps
    uint32_t capacity;  // Consumed capacity in mAh
    uint8_t percentage; // Battery percentage
} crsf_battery_t;

typedef struct {
    float pitch;        // Pitch in radians
    float roll;         // Roll in radians
    float yaw;          // Yaw in radians
} crsf_attitude_t;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * CRSF Communication Class
 * Handles CrossFire Serial Protocol communication
 */
class CRSF {
private:
    uart_port_t uart;
    bool initialized;

    uint8_t buffer[CRSF_MAX_FRAME_LEN];
    uint8_t idx;          // current write index
    uint8_t expected_len; // total length incl SYNC+LEN (0 when unknown)

    crsf_channels_t channels;
    bool fresh_channels;

    crsf_link_statistics_t link_stats;
    bool fresh_link_stats;

    crsf_battery_t battery;
    bool fresh_battery;

    crsf_attitude_t attitude;
    bool fresh_attitude;

    // Internal helper methods
    uint8_t crc8_dvb_s2_byte(uint8_t crc, uint8_t data);
    uint8_t crc8_dvb_s2(const uint8_t *data, uint8_t len);
    void unpack_channels(const uint8_t *payload, uint16_t *dest);
    void pack_channels(const uint16_t *src, uint8_t *payload);
    bool send_frame(uint8_t frame_type, const uint8_t *payload, uint8_t payload_len);
    void process_link_statistics(const uint8_t *payload, uint8_t payload_len);
    void process_battery_sensor(const uint8_t *payload, uint8_t payload_len);
    void process_attitude(const uint8_t *payload, uint8_t payload_len);
    void process_frame(bool channels_only = false);

    // Static registry for UART->CRSF mapping
    static CRSF* uart_registry[UART_NUM_MAX];

public:
    /**
     * Constructor with full UART initialization
     * @param uart_num UART port number
     * @param rx_pin RX pin number  
     * @param tx_pin TX pin number
     * @param baud Baud rate (0 for default 420000)
     */
    CRSF(uart_port_t uart_num, int rx_pin, int tx_pin, uint32_t baud = 0);

    /**
     * Constructor using existing UART configuration
     * Note: UART must have been previously initialized with the full constructor
     * @param uart_num UART port number (must be already initialized)
     */
    CRSF(uart_port_t uart_num);

    /**
     * Destructor
     */
    ~CRSF();

    /**
     * Check if CRSF object is properly initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const { return initialized; }

    /**
     * Poll for incoming CRSF data and process frames
     * Call this regularly in your main loop or task
     * @param channels_only If true, only process RC channel frames, ignore other frame types
     */
    void poll(bool channels_only = false);

    /**
     * Get the latest received channels
     * @param out Pointer to store channel data
     * @return true if fresh channels available, false otherwise
     */
    bool getChannels(crsf_channels_t *out);

    /**
     * Send channel data
     * @param channels Pointer to channel data to send
     * @return true if sent successfully, false otherwise
     */
    bool sendChannels(const crsf_channels_t *channels);

    /**
     * Send raw channel values
     * @param channels Array of 16 channel values (172-1811)
     * @return true if sent successfully, false otherwise
     */
    bool sendChannelsRaw(const uint16_t channels[16]);

    /**
     * Get the latest link statistics
     * @param out Pointer to store link statistics
     * @return true if fresh data available, false otherwise
     */
    bool getLinkStatistics(crsf_link_statistics_t *out);

    /**
     * Get the latest battery data
     * @param out Pointer to store battery data
     * @return true if fresh data available, false otherwise
     */
    bool getBattery(crsf_battery_t *out);

    /**
     * Get the latest attitude data
     * @param out Pointer to store attitude data
     * @return true if fresh data available, false otherwise
     */
    bool getAttitude(crsf_attitude_t *out);

    /**
     * Convert CRSF channel value to microseconds
     * @param crsf_value Raw CRSF channel value (172-1811)
     * @return Microsecond value (988-2012)
     */
    static uint16_t toMicroseconds(uint16_t crsf_value);

    /**
     * Convert microseconds to CRSF channel value
     * @param us_value Microsecond value (988-2012)
     * @return CRSF channel value (172-1811)
     */
    static uint16_t fromMicroseconds(uint16_t us_value);

    /**
     * Get UART port number
     * @return UART port number
     */
    uart_port_t getUartPort() const { return uart; }
};

// Legacy C-style function wrappers for backward compatibility
extern "C" {
#endif

/**
 * Legacy C API - Initialize CRSF communication
 * @param uart_num UART port number
 * @param rx_pin RX pin number
 * @param tx_pin TX pin number
 * @param baud Baud rate (0 for default 420000)
 */
void crsf_init(uart_port_t uart_num, int rx_pin, int tx_pin, uint32_t baud);

/**
 * Legacy C API - Poll for incoming CRSF data and process frames
 * Call this regularly in your main loop or task
 */
void crsf_poll(void);

/**
 * Legacy C API - Get the latest received channels
 * @param out Pointer to store channel data
 * @return true if fresh channels available, false otherwise
 */
bool crsf_get_channels(crsf_channels_t *out);

/**
 * Legacy C API - Send channel data
 * @param channels Pointer to channel data to send
 * @return true if sent successfully, false otherwise
 */
bool crsf_send_channels(const crsf_channels_t *channels);

/**
 * Legacy C API - Send raw channel values
 * @param channels Array of 16 channel values (172-1811)
 * @return true if sent successfully, false otherwise
 */
bool crsf_send_channels_raw(const uint16_t channels[16]);

/**
 * Legacy C API - Get the latest link statistics
 * @param out Pointer to store link statistics
 * @return true if fresh data available, false otherwise
 */
bool crsf_get_link_statistics(crsf_link_statistics_t *out);

/**
 * Legacy C API - Get the latest battery data
 * @param out Pointer to store battery data
 * @return true if fresh data available, false otherwise
 */
bool crsf_get_battery(crsf_battery_t *out);

/**
 * Legacy C API - Get the latest attitude data
 * @param out Pointer to store attitude data
 * @return true if fresh data available, false otherwise
 */
bool crsf_get_attitude(crsf_attitude_t *out);

/**
 * Legacy C API - Convert CRSF channel value to microseconds
 * @param crsf_value Raw CRSF channel value (172-1811)
 * @return Microsecond value (988-2012)
 */
uint16_t crsf_to_us(uint16_t crsf_value);

/**
 * Legacy C API - Convert microseconds to CRSF channel value
 * @param us_value Microsecond value (988-2012)
 * @return CRSF channel value (172-1811)
 */
uint16_t us_to_crsf(uint16_t us_value);

#ifdef __cplusplus
}
#endif

#endif //CRSF_H
