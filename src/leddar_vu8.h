#ifndef SRC_LEDDAR_VU8_H
#define SRC_LEDDAR_VU8_H

#include <condition_variable>
#include <mutex>
#include <thread>
#include <string>
#include <vector>

namespace leddar_vu8 {

enum LogLevel {
    kLogLevelDebug = 0,
    kLogLevelInfo = 10,
    kLogLevelWarn = 20,
    kLogLevelError = 30
};

/**
 * Log message. This should be defined **externally**.
 */
void log(
    LogLevel level,
    const char* file,
    int line,
    const char* function,
    const std::stringstream &message
);

/**
 * Represents a single detection reported by `Sensor`.
 */
struct Echo {
    void from_data(const uint8_t (&data)[8]);

    unsigned int distance;      //< Distance of detection (scale by `1.0/Config::distance_resolution()`).
    double amplitude;           //< Amplitude of detection.
    unsigned segment_number;    //< Number of segment of detection
    bool object_demerging;      //< Whether detection was the result of an object de-merging operation.
    bool saturated;             //< Whether detection exceeds saturation threshold (see `Config::saturation_count()`).
};

class Sensor;

/**
 * `Sensor` configuration.
 */
class Config {
public:
    explicit Config(Sensor &sensor);

    bool Load(unsigned int retry = 0);

    bool Save(unsigned int retry = 0);

    uint16_t segment_count() const;

    uint8_t accumulation_exponent() const;

    void accumulation_exponent(uint8_t value);

    uint8_t oversampling_exponent() const;

    void oversampling_exponent(uint8_t value);

    uint8_t base_samples() const;

    void base_samples(uint8_t value);

    int8_t smoothing() const;

    void smoothing(int8_t value);

    void detection_threshold(uint32_t value);

    uint32_t detection_threshold() const;

    void light_source_power_percent(uint16_t value);

    uint16_t light_source_power_percent() const;

    void saturation_count(uint8_t value);

    uint8_t saturation_count() const;

    void auto_light_source_power(uint16_t value);

    uint16_t auto_light_source_power() const;

    void distance_resolution(uint16_t value);

    uint16_t distance_resolution() const;

    bool auto_light_source_power_enabled() const;

    void auto_light_source_power_enabled(bool value);

    bool demerge_object_enabled() const;

    void demerge_object_enabled(bool value);

    bool static_noise_removal_enabled() const;

    void static_noise_removal_enabled(bool value);

    bool precision_enabled() const;

    void precision_enabled(bool value);

    bool saturation_compenstation_enabled() const;

    void saturation_compenstation_enabled(bool value);

    bool overshoot_management_enabled() const;

    void overshoot_management_enabled(bool value);

    enum BaudRate {
        kBaudRate1000 = 0,
        kBaudRate500,
        kBaudRate250,
        kBaudRate125,
        kBaudRate100,
        kBaudRate50,
        kBaudRate20,
        kBaudRate10
    };

    void baud_rate(BaudRate value);

    BaudRate baud_rate() const;

    enum FrameFormat {
        kFrameFormatStandard = 0,
        kFrameFormatExtended
    };

    void frame_format(FrameFormat value);

    FrameFormat frame_format() const;

    uint32_t base_tx_message_id() const;

    void base_tx_message_id(uint32_t value);

    uint32_t base_rx_message_id() const;

    void base_rx_message_id(uint32_t value);

    uint8_t max_detections() const;

    void max_detections(uint8_t value);

    uint16_t inter_message_delay() const;

    void inter_message_delay(uint16_t value);

    uint16_t inter_cycle_delay() const;

    void inter_cycle_delay(uint16_t value);

    uint32_t enabled_segments() const;

    void enabled_segments(uint32_t value);

private:
    Sensor &sensor_;

    uint16_t segment_count_ = 8;
    uint8_t accumulation_exponent_ = 0;
    uint8_t oversampling_exponent_ = 0;
    uint8_t base_samples_ = 0;
    int8_t smoothing_ = 0;
    uint32_t detection_threshold_ = 0;
    uint16_t light_source_power_percent_ = 0;
    uint8_t saturation_count_ = 0;
    uint16_t auto_light_source_power_ = 0;
    uint16_t distance_resolution_ = 0;
    uint16_t acquisition_options_ = 0;
    BaudRate baud_rate_ = kBaudRate1000;
    FrameFormat frame_format_ = kFrameFormatStandard;
    uint32_t base_rx_message_id_ = 0x740;
    uint32_t base_tx_message_id_ = 0x750;
    uint8_t max_detections_ = 0;
    uint16_t inter_message_delay_ = 0;
    uint16_t inter_cycle_delay_ = 0;
    uint32_t enabled_segments_ = 0;
};

class Stream;

/**
 * Connection to a Leddar Vu8 sensor.
 */
class Sensor {
public:
    Sensor();

    const Config &config() const;

    Config &config();

    bool Connect();

    bool is_connected() const;

    void Disconnect();

    int sock() const;

    void interface(const std::string &value);

    const std::string &interface() const;

    void send_timeout(double value);

    double send_timeout() const;

    void receive_timeout(double value);

    double receive_timeout() const;

    /**
     * Start streaming detections continuously as they become available.
     */
    bool StartStream(unsigned int retry = 0);

    /**
     * Stop streaming detections continuously.
     */
    bool StopStream(unsigned int retry = 0);

    /**
     * Listen for streaming detections.
     */
    Stream Listen(double timeout);

private:
    int sock_ = -1;
    std::string interface_ = "can0";
    double send_timeout_ = 0;
    double recv_timeout_ = 0;
    Config config_;
};

/**
 * Stream that listens for continuous detections from `Sensor`.
 */
class Stream {
public:
    Stream(
        const std::string &interface,
        double timeout,
        unsigned int base_tx_message_id,
        unsigned int max_detections);

    Stream(const Stream &stream);

    ~Stream();

    bool Start();

    bool is_listening() const;

    void Stop();

    unsigned int sequence() const;

    void last(unsigned int &seq, std::vector<Echo> &echos) const;

private:
    void Listen();

    std::thread thd_;
    std::condition_variable listening_cv_;
    std::mutex listening_mtx_;
    bool listening_  = false;
    std::string interface_;
    double timeout_;
    unsigned int base_tx_message_id_;
    unsigned int max_detections_;
    int sock_ = -1;
    int notify_ = -1;
    mutable std::mutex echos_mtx_;
    unsigned int seq_ = 0;
    std::vector<Echo> echos_;
};

}  // namespace leddar_vu8

#endif  // SRC_LEDDAR_VU8_H
