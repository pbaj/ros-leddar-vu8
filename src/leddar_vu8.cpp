#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "./leddar_vu8.h"

#define LOG(level, args) \
    { \
        std::stringstream ss; \
        ss << args; \
        leddar_vu8::log((level), __FILE__, __LINE__, __FUNCTION__, ss); \
    }

#define LOG_ERROR(args) \
    LOG(leddar_vu8::kLogLevelError, args)

#define LOG_INFO(args) \
    LOG(leddar_vu8::kLogLevelInfo, args)

class LockGuard {
public:
    explicit LockGuard(std::mutex &mutex) : mutex(mutex) {
        mutex.lock();
    }

    ~LockGuard() {
        mutex.unlock();
    }

    std::mutex &mutex;
};

#define TO_UINT32(f1, f2, f3, f4) (\
    ((f1) <<  0) | \
    ((f2) <<  8) | \
    ((f3) << 16) | \
    ((f4) << 24) \
)

#define TO_UINT16(f1, f2) (\
    ((f1) <<  0) | \
    ((f2) <<  8) \
)

#define FROM_UINT32(value, f1, f2, f3, f4) \
    (f1) = ((value >>  0) & 0xff); \
    (f2) = ((value >>  8) & 0xff); \
    (f3) = ((value >> 16) & 0xff); \
    (f4) = ((value >> 24) & 0xff);

#define FROM_UINT16(value, f1, f2) \
    (f1) = ((value >>  0) & 0xff); \
    (f2) = ((value >>  8) & 0xff);


static bool send_can_data(int sock, uint32_t message_id, const uint8_t (&data)[8]) {
    struct can_frame frame;
    frame.can_id = message_id;
    frame.can_dlc = 8;
    memcpy(frame.data, data, 8);

    int rc = send(sock, &frame, sizeof(frame), 0);
    if (rc != sizeof(frame)) {
        LOG_ERROR("send can frame failed w/ errno " << errno << " - " << strerror(errno));
        return false;
    }
    return true;
}

static bool recv_can_data(int sock, uint32_t &message_id, uint8_t (&data)[8]) {
    struct can_frame frame;
    int rc = read(sock, &frame, sizeof(frame));
    if (rc != sizeof(frame)) {
        LOG_ERROR("recv can frame failed w/ errno " << errno << " - " << strerror(errno));
        return false;
    }
    message_id = frame.can_id;
    if (frame.can_dlc != 8) {
        LOG_ERROR("can frame data size " << frame.can_dlc << " != " << 8);
        return false;
    }
    memcpy(data, frame.data, 8);
    return true;
}

static bool send_and_recv_can_data(
        int sock,
        uint32_t rx_message_id,
        const uint8_t (&request)[8],
        uint32_t tx_message_id,
        uint8_t (&answer)[8],
        unsigned int retries = 0) {
    static uint8_t error[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    uint32_t message_id;
    unsigned int remaining = 1 + retries;

    while (remaining != 0) {
        if (retries != 0) {
            LOG_INFO("retry " << retries - remaining + 1 << "/" << retries << " rx_message_id=" << rx_message_id <<
                    " tx_message_id=" << tx_message_id);
        }

        // send request
        if (!send_can_data(sock, rx_message_id, request)) {
            remaining -= 1;
            continue;
        }

        // recv answer
        if (!recv_can_data(sock, message_id, answer)) {
            remaining -= 1;
            continue;
        }

        // validate answer
        if (message_id != tx_message_id) {
            LOG_INFO("recv'd answer can message id " << message_id << " != expected " << tx_message_id);
            remaining -= 1;
            continue;
        }
        if (answer[0] != request[0]) {
            LOG_ERROR("sensor answer[0] " << answer[0] << " != request[1] " << request[1]);
            remaining -= 1;
            continue;
        }
        if (memcmp(answer + 2, error, 6) == 0) {
            LOG_ERROR("sensor answer = error");
            remaining -= 1;
            continue;
        }

        // success
        break;
    }
    return false;
}


static bool send_and_echo_can_data(
        int sock,
        uint32_t rx_message_id,
        const uint8_t (&request)[8],
        uint32_t tx_message_id,
        unsigned int retries = 0) {
    uint32_t message_id;
    uint8_t answer[8] = {0};
    unsigned int remaining = 1 + retries;

    while (remaining != 0) {
        if (retries != 0) {
            LOG_INFO("retry " << retries - remaining - 1 << "/" << retries << " rx_message_id=" << rx_message_id <<
                    " tx_message_id=" << tx_message_id);
        }

        // send request
        if (!send_can_data(sock, rx_message_id, request)) {
            remaining -= 1;
            continue;
        }

        // recv answer
        if (!recv_can_data(sock, message_id, answer)) {
            remaining -= 1;
            continue;
        }

        // validate answer
        if (message_id != tx_message_id) {
            LOG_INFO("recv'd answer can message id " << message_id << " != expected " << tx_message_id);
            remaining -= 1;
            continue;
        }
        if (memcmp(answer, request, 8) != 0) {
            LOG_ERROR("sensor answer != request");
            remaining -= 1;
            continue;
        }

        // success
        break;
    }
    return false;
}

static canid_t can_mask(canid_t b, canid_t e) {
    const size_t w = (sizeof(canid_t) * 8);
    // https://stackoverflow.com/a/3314434
    canid_t prefix = (b ^ e) ? w - 1 - static_cast<int>(log2(b ^ e)) : w;
    canid_t mask = ((~(canid_t)0) << (w - prefix)) & b & CAN_SFF_MASK;
    return mask;
}

namespace leddar_vu8 {

// Echo

void  Echo::from_data(const uint8_t (&data)[8]) {
    distance = TO_UINT16(data[0], data[1]);
    amplitude = static_cast<double>(TO_UINT16(data[2], data[3])) / 64.0;
    uint16_t flags = TO_UINT16(data[4], data[5]);
    object_demerging = (flags & (1 << 1)) != 0;
    saturated = (flags & (1 << 3)) != 0;
    segment_number = TO_UINT16(data[6], data[7]);
}


// Stream

Stream::Stream(
    const std::string &interface,
    double timeout,
    unsigned int base_tx_message_id,
    unsigned int max_detections
) :
    interface_(interface),
    timeout_(timeout),
    base_tx_message_id_(base_tx_message_id),
    max_detections_(max_detections) {
}


Stream::Stream(const Stream &other) :
    interface_(other.interface_),
    timeout_(other.timeout_),
    base_tx_message_id_(other.base_tx_message_id_),
    max_detections_(other.max_detections_) {
}

Stream::~Stream() {
    Stop();
}

bool Stream::Start() {
    int sock = -1;
    int notify = -1;
    int rc = 0;
    bool result = false;
    struct can_filter filter[1];

    // create the socket
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if ( sock < 0 ) {
        LOG_ERROR("failed to create socket w/ errno " << errno << " - " << strerror(errno));
        goto cleanup;
    }

    // receive timeout
    if (timeout_ != 0) {
        struct timeval tv = {
            static_cast<int>(timeout_),
            static_cast<int>((timeout_ - static_cast<int>(timeout_)) * 1e6)  // micro-seconds
        };
        rc = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        if (rc < 0) {
            LOG_ERROR("set socket recv timeout " << timeout_ << " failed w/ errno " << errno << " - " <<
                    strerror(errno));
            goto cleanup;
        }
        LOG_INFO("set socket recv timeout to " << timeout_);
    }

    // bind the socket;
    struct ifreq ifr;
    if (interface_.size() + 1 > IF_NAMESIZE) {
        LOG_ERROR("interface " << interface_ << " length > " << IF_NAMESIZE);
    }
    snprintf(ifr.ifr_name, IF_NAMESIZE, "%s", interface_.c_str());
    rc = ioctl(sock, SIOCGIFINDEX, &ifr);
    if (rc == -1) {
        LOG_ERROR("find interface " << interface_ << " index failed w/ errno " << errno << " - " << strerror(errno));
        goto cleanup;
    }
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    rc = bind(sock, (struct sockaddr*)&addr, sizeof(addr));
    if (rc == -1) {
        LOG_ERROR("bind socket to interface " << interface_ << " failed w/ errno " << errno << " - " <<
                strerror(errno));
        goto cleanup;
    }

    // filter
    filter[0].can_id = base_tx_message_id_ + 1;
    filter[0].can_mask = can_mask(base_tx_message_id_ + 1, base_tx_message_id_ + 1 + max_detections_);
    rc = setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
    if (rc < 0) {
        LOG_ERROR("set socket filter failed w/ errno " << errno << " - " << strerror(errno));
        goto cleanup;
    }
    LOG_INFO("set socket filter w/ can_id=0x" << std::hex << filter[0].can_id << ", mask=0x" << std::hex <<
            filter[0].can_mask);

    // notify event
    notify = eventfd(0, 0);
    if (notify == -1) {
        LOG_ERROR("notify eventfd failed w/ errno " << errno << " - " << strerror(errno));
        goto cleanup;
    }

    // yay!
    sock_ = sock;
    sock = -1;
    notify_ = notify;
    notify = -1;
    result = true;

cleanup:
    if (sock != -1) {
        close(sock);
        sock = -1;
    }
    if (notify != -1) {
        close(notify);
        notify = -1;
    }

    thd_ = std::thread(&Stream::Listen, this);
    std::unique_lock<std::mutex> lk(listening_mtx_);
    listening_cv_.wait_for(lk, std::chrono::seconds(1), [&]{ return listening_; });
    if (!listening_) {
        Stop();
        result = false;
    }

    return result;
}

bool Stream::is_listening() const {
    return listening_;
}

void Stream::Stop() {
    if (is_listening()) {
        LOG_INFO("notify stream thread");
        eventfd_write(notify_, 1);
    }

    if (thd_.joinable()) {
        LOG_INFO("join stream thread " << thd_.get_id());
        thd_.join();
    }

    if (sock_ != -1) {
        LOG_INFO("closing stream socket 0x" << std::hex << sock_);
        close(sock_);
        sock_ = -1;
    }

    if (notify_ != -1) {
        LOG_INFO("closing notify event 0x" << std::hex << notify_);
        close(notify_);
        notify_ = -1;
    }
}

unsigned int Stream::sequence() const {
    return seq_;
}

void Stream::last(unsigned int &seq, std::vector<Echo> &echos) const {
    LockGuard guard(echos_mtx_);
    seq = seq_;
    echos = echos_;
}

void Stream::Listen() {
    struct ListeningGuard {
       explicit ListeningGuard(bool &listening) : listening_(listening) {
           listening_ = true;
       }

       ~ListeningGuard() {
           listening_ = false;
      }

      bool &listening_;
    };

    ListeningGuard listening_guard(listening_);
    listening_cv_.notify_all();

    LOG_INFO("streaming to 0x" << std::hex << sock_);

    int rc;
    uint32_t message_id;
    uint8_t data[8];
    uint8_t detection_count = 0;
    uint8_t detection_i = 0;
    std::vector<Echo> echos;
    uint32_t min_message_id = base_tx_message_id_ + 1;
    uint32_t max_message_id = base_tx_message_id_ + 1 + max_detections_;

    if (sock_ > FD_SETSIZE) {
        LOG_INFO("streaming to 0x" << std::hex << sock_);
        return;
    }

    if (notify_ > FD_SETSIZE) {
        LOG_INFO("streaming to 0x" << std::hex << sock_);
        return;
    }

    int nfds = std::max(sock_, notify_) + 1;
    fd_set rfds;
    while (true) {
        FD_ZERO(&rfds);
        FD_SET(sock_, &rfds);
        FD_SET(notify_, &rfds);
        rc = select(nfds, &rfds, NULL, NULL, NULL);
        if (rc == -1) {
            LOG_ERROR("failed select w/ errno " << errno << " - " << strerror(errno));
            break;
        }

        // message
        if (FD_ISSET(sock_, &rfds)) {
            if (!recv_can_data(sock_, message_id, data)) {
                break;
            }
            if (message_id < min_message_id || max_message_id < message_id) {
                // not ours
                continue;
            }
            if (message_id == base_tx_message_id_ + 1) {
                // number of detections
                detection_count = data[0];
                detection_i = 0;
                echos.resize(detection_count);
            } else if (message_id > base_tx_message_id_ + 1){
                // detection
                if (detection_i < detection_count) {
                    echos[detection_i].from_data(data);
                    detection_i += 1;
                    if (detection_i == detection_count) {
                        LockGuard guard(echos_mtx_);
                        std::swap(echos_, echos);
                        seq_ += 1;
                    }
                }
            }
        }

        // notify
        if (FD_ISSET(notify_, &rfds)) {
            eventfd_t value = 0;
            rc = eventfd_read(notify_, &value);
            if (rc == -1) {
                LOG_ERROR("failed to read notify event w/ errno " << errno << " - " << strerror(errno));
                break;
            }
            break;
        }
    }
}

// Sensor

Sensor::Sensor() :
    config_(*this) {
}

const Config &Sensor::config() const {
    return config_;
}

Config &Sensor::config() {
    return config_;
}

bool Sensor::Connect() {
    int sock = -1;
    int rc = 0;
    bool result = false;

    // create the socket
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if ( sock < 0 ) {
        LOG_ERROR("failed to create socket w/ errno " << errno << " - " << strerror(errno));
        goto cleanup;
    }

    // send timeout
    if (send_timeout_ == 0) {
        struct timeval lTimeVal = {
            static_cast<int>(send_timeout_),
            static_cast<int>((send_timeout_ - static_cast<int>(send_timeout_)) * 1e6)  // micro-seconds
        };
        rc = setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &lTimeVal, sizeof( lTimeVal ));
        if (rc < 0) {
            LOG_ERROR("failed to set socket send timeout w/ errno " << errno << " - " << strerror(errno));
            goto cleanup;
        }
        LOG_INFO("set socket send timeout to " << send_timeout_);
    }

    // receive timeout
    if (recv_timeout_ != 0) {
        struct timeval tv = {
            static_cast<int>(recv_timeout_),
            static_cast<int>((recv_timeout_ - static_cast<int>(recv_timeout_)) * 1e6)  // micro-seconds
        };
        rc = setsockopt( sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv) );
        if (rc < 0) {
            LOG_ERROR("set socket recv timeout " << recv_timeout_ << " failed w/ errno " << errno << " - " <<
                    strerror(errno));
            goto cleanup;
        }
        LOG_INFO("set socket recv timeout to " << recv_timeout_);
    }

    // bind the socket;
    struct ifreq ifr;
    if (interface_.size() + 1 > IF_NAMESIZE) {
        LOG_ERROR("interface " << interface_ << " length > " << IF_NAMESIZE);
    }
    snprintf(ifr.ifr_name, IF_NAMESIZE, "%s", interface_.c_str());
    rc = ioctl(sock, SIOCGIFINDEX, &ifr);
    if (rc == -1) {
        LOG_ERROR("find interface " << interface_ << " index failed w/ errno " << errno << " - " << strerror(errno));
        goto cleanup;
    }
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    rc = bind(sock, (struct sockaddr*)&addr, sizeof(addr));
    if (rc == -1) {
        LOG_ERROR("bind socket to interface " << interface_ << " failed w/ errno " << errno << " - " <<
                strerror(errno));
        goto cleanup;
    }

    // filter
    struct can_filter filter[1];
    filter[0].can_id = config_.base_tx_message_id();
    filter[0].can_mask = CAN_SFF_MASK;
    rc = setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
    if (rc < 0) {
        LOG_ERROR("set socket filter failed w/ errno " << errno << " - " << strerror(errno));
        goto cleanup;
    }
    LOG_INFO("set socket filter w/ can_id=0x" << std::hex << filter[0].can_id << ", mask=0x" << std::hex <<
            filter[0].can_mask);

    // yay!
    sock_ = sock;
    sock = -1;
    result = true;

cleanup:
    if (sock != -1) {
        close(sock);
    }

    return result;
}

bool Sensor::is_connected() const {
    return sock_ != -1;
}

void Sensor::Disconnect() {
    if (sock_ != -1) {
        close(sock_);
        sock_ = -1;
    }
}

int Sensor::sock() const {
    return sock_;
}

void Sensor::interface(const std::string &value) {
    interface_ = value;
}

const std::string &Sensor::interface() const {
    return interface_;
}


void Sensor::send_timeout(double value) {
    send_timeout_ = value;
}

double Sensor::send_timeout() const {
    return send_timeout_;
}

void Sensor::receive_timeout(double value) {
    recv_timeout_ = value;
}

double Sensor::receive_timeout() const {
    return send_timeout_;
}

Stream Sensor::Listen(double timeout) {
    return Stream(
        interface_,
        timeout,
        config_.base_tx_message_id(),
        config_.max_detections());
}

bool Sensor::StartStream(unsigned int retry) {
    uint8_t data[8] = {0};
    data[0] = 3;
    data[1] = 1;  // multi message mode
    if (!send_and_echo_can_data(
            sock_,
            config_.base_rx_message_id(),
            data,
            config_.base_tx_message_id(),
            retry)) {
        return false;
    }
    LOG_INFO("started streaming detections");
    return true;
}

bool Sensor::StopStream(unsigned int retry) {
    uint8_t data[8] = {0};
    data[0] = 1;
    if (!send_and_echo_can_data(
            sock_,
            config_.base_rx_message_id(),
            data,
            config_.base_tx_message_id(),
            retry)) {
        return false;
    }
    LOG_INFO("stopped streaming detections");
    return true;
}

// Config

Config::Config(Sensor &sensor) :
    sensor_(sensor) {
}

bool Config::Load(unsigned int retry) {
    uint8_t req[8];
    uint8_t ans[8];

    // acquisition configuration
    memset(req, 0, 8);
    req[0] = 5;
    req[1] = 0;
    if (!send_and_recv_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            ans,
            retry)) {
        return false;
    }
    accumulation_exponent_ = ans[2];
    LOG_INFO("loaded accumulation_exponent=" << unsigned(accumulation_exponent_));
    oversampling_exponent_ = ans[3];
    LOG_INFO("loaded oversampling_exponent=" << unsigned(oversampling_exponent_));
    base_samples_ = ans[4];
    LOG_INFO("loaded base_samples=" << unsigned(base_samples_));

    // smoothing and detection threshold
    memset(req, 0, 8);
    req[0] = 5;
    req[1] = 1;
    if (!send_and_recv_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            ans,
            retry)) {
        return false;
    }
    smoothing_ = ans[2];
    LOG_INFO("loaded smoothing=" << signed(smoothing_));
    detection_threshold_ = TO_UINT32(ans[4], ans[5], ans[6], ans[7]);
    LOG_INFO("loaded detection_threshold=" << detection_threshold_);

    // light source power management
    memset(req, 0, 8);
    req[0] = 5;
    req[1] = 2;
    if (!send_and_recv_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            ans,
            retry)) {
        return false;
    }
    light_source_power_percent_ = ans[2];
    LOG_INFO("loaded light_source_power_percent_=" << light_source_power_percent_);
    saturation_count_ = ans[3];
    LOG_INFO("loaded saturation_count=" << unsigned(saturation_count_));
    auto_light_source_power_ = TO_UINT16(ans[4], ans[5]);
    LOG_INFO("loaded auto_light_source_power=" << auto_light_source_power_);

    // distance resolution and acquisition options
    memset(req, 0, 8);
    req[0] = 5;
    req[1] = 3;
    if (!send_and_recv_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            ans,
            retry)) {
        return false;
    }
    distance_resolution_ = TO_UINT16(ans[2], ans[3]);
    distance_resolution_ = 100;
    LOG_INFO("loaded distance_resolution=" << distance_resolution_);
    acquisition_options_ = TO_UINT16(ans[4], ans[5]);
    LOG_INFO("loaded auto_light_source_power_enabled=" <<auto_light_source_power_enabled());
    LOG_INFO("loaded demerge_object_enabled=" << demerge_object_enabled());
    LOG_INFO("loaded static_noise_removal_enabled=" << static_noise_removal_enabled());
    LOG_INFO("loaded precision_enabled=" << precision_enabled());
    LOG_INFO("loaded saturation_compenstation_enabled=" << saturation_compenstation_enabled());
    LOG_INFO("loaded overshoot_management_enabled=" << overshoot_management_enabled());

    // can port configuration
    memset(req, 0, 8);
    req[0] = 5;
    req[1] = 4;
    if (!send_and_recv_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            ans,
            retry)) {
        return false;
    }
    baud_rate_ = (BaudRate) ans[2];
    LOG_INFO("loaded baud_rate=" << baud_rate_);
    frame_format_ = (FrameFormat) ans[3];
    LOG_INFO("loaded frame_format=" << frame_format_);
    base_tx_message_id_ = TO_UINT32(ans[4], ans[5], ans[6], ans[7]);
    LOG_INFO("loaded base_tx_message_id=0x" << std::hex << base_tx_message_id_);
    memset(req, 0, 8);
    req[0] = 5;
    req[1] = 5;
    if (!send_and_recv_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            ans,
            retry)) {
        return false;
    }
    base_rx_message_id_ = TO_UINT32(ans[4], ans[5], ans[6], ans[7]);
    LOG_INFO("loaded base_rx_message_id=0x" << std::hex << base_rx_message_id_);
    memset(req, 0, 8);
    req[0] = 5;
    req[1] = 6;
    if (!send_and_recv_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            ans,
            retry)) {
        return false;
    }
    max_detections_ = ans[3];
    LOG_INFO("loaded max_detections=" << unsigned(max_detections_));
    inter_message_delay_ = TO_UINT16(ans[4], ans[5]);
    LOG_INFO("loaded inter_message_delay=" << inter_message_delay_);
    inter_cycle_delay_ = TO_UINT16(ans[6], ans[7]);
    LOG_INFO("loaded inter_cycle_delay=" << inter_cycle_delay_);

    // enabled segments
    memset(req, 0, 8);
    req[0] = 5;
    req[1] = 8;
    if (!send_and_recv_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            ans,
            retry)) {
        return false;
    }
    enabled_segments_ = TO_UINT32(ans[4], ans[5], ans[6], ans[7]);
    LOG_INFO("loaded enabled_segments=" << enabled_segments_);

    return true;
}

bool Config::Save(unsigned int retry) {
    uint8_t req[8];

    // acquisition configuration
    memset(req, 0, 8);
    req[0] = 6;
    req[1] = 0;
    req[2] = accumulation_exponent_;
    LOG_INFO("saving accumulation_exponent=" << unsigned(accumulation_exponent_));
    req[3] = oversampling_exponent_;
    LOG_INFO("saving oversampling_exponent=" << unsigned(oversampling_exponent_));
    req[4] = base_samples_;
    LOG_INFO("saving base_samples=" << unsigned(base_samples_));
    if (!send_and_echo_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            retry)) {
        return false;
    }

    // smoothing and detection threshold
    memset(req, 0, 8);
    req[0] = 6;
    req[1] = 1;
    req[2] = smoothing_;
    LOG_INFO("saving smoothing=" << signed(smoothing_));
    FROM_UINT32(detection_threshold_, req[4], req[5], req[6], req[7]);
    LOG_INFO("saving detection_threshold=" << detection_threshold_);
    if (!send_and_echo_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            retry)) {
        return false;
    }

    // light source power management
    memset(req, 0, 8);
    req[0] = 6;
    req[1] = 2;
    req[2] = light_source_power_percent_;
    LOG_INFO("saving light_source_power_percent=" << light_source_power_percent_);
    req[3] = saturation_count_;
    LOG_INFO("saving saturation_count=" << unsigned(saturation_count_));
    FROM_UINT16(auto_light_source_power_, req[4], req[5]);
    if (!send_and_echo_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            retry)) {
        return false;
    }

    // distance resolution and acquisition options
    memset(req, 0, 8);
    req[0] = 6;
    req[1] = 3;
    FROM_UINT16(distance_resolution_, req[2], req[3]);
    LOG_INFO("saving distance_resolution=" << distance_resolution_);
    FROM_UINT16(acquisition_options_, req[4], req[5]);
    LOG_INFO("saving auto_light_source_power_enabled=" << auto_light_source_power_enabled());
    LOG_INFO("saving demerge_object_enabled=" << demerge_object_enabled());
    LOG_INFO("saving static_noise_removal_enabled=" << static_noise_removal_enabled());
    LOG_INFO("saving precision_enabled=" << precision_enabled());
    LOG_INFO("saving saturation_compenstation_enabled=" << saturation_compenstation_enabled());
    LOG_INFO("saving overshoot_management_enabled=" << overshoot_management_enabled());
    if (!send_and_echo_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            retry)) {
        return false;
    }

    // can port
    memset(req, 0, 8);
    req[0] = 6;
    req[1] = 6;
    req[3] = max_detections_;
    LOG_INFO("saving max_detections=" << unsigned(max_detections_));
    FROM_UINT16(inter_message_delay_, req[4], req[5]);
    LOG_INFO("saving inter_message_delay=" << inter_message_delay_);
    FROM_UINT16(inter_cycle_delay_, req[6], req[7]);
    LOG_INFO("saving inter_cycle_delay=" << inter_cycle_delay_);
    if (!send_and_echo_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            retry)) {
        return false;
    }

    // enabled segments
    memset(req, 0, 8);
    req[0] = 6;
    req[1] = 8;
    FROM_UINT32(enabled_segments_, req[4], req[5], req[6], req[7]);
    LOG_INFO("saving enabled_segments=" << enabled_segments_);
    if (!send_and_echo_can_data(
            sensor_.sock(),
            base_rx_message_id_,
            req,
            base_tx_message_id_,
            retry)) {
        return false;
    }

    return true;
}

uint8_t Config::accumulation_exponent() const {
    return accumulation_exponent_;
}

void Config::accumulation_exponent(uint8_t value) {
    accumulation_exponent_ = value;
}

uint8_t Config::oversampling_exponent() const {
    return oversampling_exponent_;
}

void Config::oversampling_exponent(uint8_t value) {
    oversampling_exponent_ = value;
}

uint8_t Config::base_samples() const {
    return base_samples_;
}

void Config::base_samples(uint8_t value) {
    base_samples_ = value;
}

int8_t Config::smoothing() const {
    return smoothing_;
}

void Config::smoothing(int8_t value) {
    smoothing_ = value;
}

void Config::detection_threshold(uint32_t value) {
    detection_threshold_ = value;
}

uint32_t Config::detection_threshold() const {
    return detection_threshold_;
}

void Config::light_source_power_percent(uint16_t value) {
    light_source_power_percent_ = value;
}

uint16_t Config::light_source_power_percent() const {
    return light_source_power_percent_;
}

void Config::saturation_count(uint8_t value) {
    saturation_count_ = value;
}

uint8_t Config::saturation_count() const {
    return saturation_count_;
}

void Config::auto_light_source_power(uint16_t value) {
    auto_light_source_power_ = value;
}

uint16_t Config::auto_light_source_power() const {
    return auto_light_source_power_;
}

void Config::distance_resolution(uint16_t value) {
    distance_resolution_ = value;
}

uint16_t Config::distance_resolution() const {
    return distance_resolution_;
}

bool Config::auto_light_source_power_enabled() const {
    return (acquisition_options_ & (1 << 0)) != 0;
}

void Config::auto_light_source_power_enabled(bool value) {
    if (value) {
        acquisition_options_ |= (1 << 0);
    } else {
        acquisition_options_ &= ~(1 << 0);
    }
}

bool Config::demerge_object_enabled() const {
    return (acquisition_options_ & (1 << 1)) != 0;
}

void Config::demerge_object_enabled(bool value) {
    if (value) {
        acquisition_options_ |= (1 << 1);
    } else {
        acquisition_options_ &= ~(1 << 1);
    }
}

bool Config::static_noise_removal_enabled() const {
    return (acquisition_options_ & (1 << 2)) != 0;
}

void Config::static_noise_removal_enabled(bool value) {
    if (value) {
        acquisition_options_ |= (1 << 2);
    } else {
        acquisition_options_ &= ~(1 << 2);
    }
}

bool Config::precision_enabled() const {
    return (acquisition_options_ & (1 << 3)) != 0;
}

void Config::precision_enabled(bool value) {
    if (value) {
        acquisition_options_ |= (1 << 3);
    } else {
        acquisition_options_ &= ~(1 << 3);
    }
}

bool Config::saturation_compenstation_enabled() const {
    return (acquisition_options_ & (1 << 4)) != 0;
}

void Config::saturation_compenstation_enabled(bool value) {
    if (value) {
        acquisition_options_ |= (1 << 4);
    } else {
        acquisition_options_ &= ~(1 << 4);
    }
}

bool Config::overshoot_management_enabled() const {
    return (acquisition_options_ & (1 << 5)) != 0;
}

void Config::overshoot_management_enabled(bool value) {
    if (value) {
        acquisition_options_ |= (1 << 5);
    } else {
        acquisition_options_ &= ~(1 << 5);
    }
}

void Config::baud_rate(BaudRate value) {
    baud_rate_ = value;
}

Config::BaudRate Config::baud_rate() const {
    return baud_rate_;
}

void Config::frame_format(FrameFormat value) {
    frame_format_ = value;
}

Config::FrameFormat Config::frame_format() const {
    return frame_format_;
}

uint32_t Config::base_tx_message_id() const {
    return base_tx_message_id_;
}

void Config::base_tx_message_id(uint32_t value) {
    base_tx_message_id_ = value;
}

uint32_t Config::base_rx_message_id() const {
    return base_rx_message_id_;
}

void Config::base_rx_message_id(uint32_t value) {
    base_rx_message_id_ = value;
}

uint8_t Config::max_detections() const {
    return max_detections_;
}

void Config::max_detections(uint8_t value) {
    max_detections_ = value;
}

uint16_t Config::inter_message_delay() const {
    return inter_message_delay_;
}

void Config::inter_message_delay(uint16_t value) {
    inter_message_delay_ = value;
}

uint16_t Config::inter_cycle_delay() const {
    return inter_cycle_delay_;
}

void Config::inter_cycle_delay(uint16_t value) {
    inter_cycle_delay_ = value;
}

uint32_t Config::enabled_segments() const {
    return enabled_segments_;
}

void Config::enabled_segments(uint32_t value) {
    enabled_segments_ = value;
}

}  // namespace leddar_vu8
