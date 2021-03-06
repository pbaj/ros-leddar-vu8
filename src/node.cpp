#include <limits>
#include <string>
#include <vector>

#include <angles/angles.h>

#include "./node.h"

Node::Node(ros::NodeHandle nh):
    nh_(nh)
{}

Node::~Node() {
    Close();
}

bool Node::Initialize() {
    // static params
    std::string interface = "can0";
    if (nh_.hasParam("interface")) {
        if (!nh_.getParam("interface", interface)) {
            ROS_ERROR("~interface parameter must be string");
            return false;
        }
    }
    sensor_.interface(interface);
    ROS_INFO_STREAM("interface=" << sensor_.interface());
    double send_timeout = 0;
    if (nh_.hasParam("send_timeout")) {
        if (!nh_.getParam("send_timeout", send_timeout)) {
            ROS_ERROR("~send_timeout parameter must be float");
            return false;
        }
    }
    sensor_.send_timeout(send_timeout);
    ROS_INFO_STREAM("send_timeout=" << sensor_.send_timeout());
    double recv_timeout = 0;
    if (nh_.hasParam("receive_timeout")) {
        if (!nh_.getParam("receive_timeout", recv_timeout)) {
            ROS_ERROR("~receive_timeout parameter must be float");
            return false;
        }
    }
    ROS_INFO_STREAM("receive_timeout=" << sensor_.receive_timeout());
    if (nh_.hasParam("listen_timeout")) {
        if (!nh_.getParam("listen_timeout", listen_timeout_)) {
            ROS_ERROR("~listen parameter must be float");
            return false;
        }
    }
    ROS_INFO_STREAM("listen_timeout=" << listen_timeout_);
    if (nh_.hasParam("min_range")) {
        if (!nh_.getParam("min_range", min_range_)) {
            ROS_ERROR("~min_range parameter must be float");
            return false;
        }
    }
    ROS_INFO_STREAM("min_range=" << min_range_);
    if (nh_.hasParam("max_range")) {
        if (!nh_.getParam("max_range", max_range_)) {
            ROS_ERROR("~max_range parameter must be float");
            return false;
        }
    }
    ROS_INFO_STREAM("max_range=" << max_range_);
    if (nh_.hasParam("fov")) {
        if (!nh_.getParam("fov", field_of_view_)) {
            ROS_ERROR("~fov parameter must be float");
            return false;
        }
    }
    ROS_INFO_STREAM("fov=" << field_of_view_);
    if (nh_.hasParam("rate")) {
        if (!nh_.getParam("rate", rate_)) {
            ROS_ERROR("~rate parameter must be float");
            return false;
        }
    }
    ROS_INFO_STREAM("rate=" << rate_);
    if (nh_.hasParam("frame_id")) {
        if (!nh_.getParam("frame_id", frame_id_)) {
            ROS_ERROR("~rate parameter must be string");
            return false;
        }
    }
    ROS_INFO_STREAM("frame_id=" << frame_id_);

    // config
    ROS_INFO("loading sensor config");
    leddar_vu8::Config &config = sensor_.config();
    if (nh_.hasParam("base_tx_message_id")) {
        int base_tx_message_id;
        if (!nh_.getParam("base_tx_message_id", base_tx_message_id)) {
            ROS_ERROR("~base_tx_message_id parameter must be integer");
            return false;
        }
        config.base_tx_message_id(base_tx_message_id);
    }
    ROS_INFO_STREAM("base_tx_message_id=" << config.base_tx_message_id());
    if (nh_.hasParam("base_rx_message_id")) {
        int base_rx_message_id;
        if (!nh_.getParam("base_rx_message_id", base_rx_message_id)) {
            ROS_ERROR("~base_rx_message_id parameter must be integer");
            return false;
        }
        config.base_rx_message_id(base_rx_message_id);
    }
    ROS_INFO_STREAM("base_rx_message_id=" << config.base_rx_message_id());

    // connect to sensor
    sensor_.receive_timeout(recv_timeout);
    ROS_INFO("connecting to sensor");
    if (!sensor_.Connect()) {
        return false;
    }

    // stop streaming
    if (!sensor_.StopStream(retry_)) {
        ROS_ERROR("stop sensor streaming failed");
        return false;
    }

    // load sensor config
    if (!config.Load(retry_)) {
        return false;
    }

    // advertise topic
    ROS_INFO("creating scan publisher");
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);

    // dynamic params
    dynamic_reconfigure::Server<leddar_vu8::LeddarVu8Config>::CallbackType f;
    f = boost::bind(&Node::Reconfig, this, _1, _2);
    reconfig_.setCallback(f);

    return true;
}

void Node::Close() {
    sensor_.StopStream();
    sensor_.Disconnect();
    scan_pub_.shutdown();
}

bool Node::StreamForever() {
    // continuous
    if (continuous_ && !sensor_.StartStream(retry_)) {
        ROS_ERROR("start sensor streaming failed");
        return false;
    }

    // listen for continuous
    unsigned int seq = 0;
    std::vector<leddar_vu8::Echo> echos;
    leddar_vu8::Stream stream = sensor_.Listen(listen_timeout_);
    if (!stream.Start()) {
        return false;
    }

    // poll detection -> scan @ rate
    sensor_msgs::LaserScan scan;
    ros::Rate rate(rate_);
    while (!ros::isShuttingDown()) {
        if (continuous_ && stream.is_listening()) {
            if (stream.sequence() != seq) {
                stream.last(seq, echos);
                ToLaserScan(echos, scan);
                scan_pub_.publish(scan);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    stream.Stop();

    return true;
}


void Node::ToLaserScan(
    const std::vector<leddar_vu8::Echo>& echos,
    sensor_msgs::LaserScan& scan
) {
    const auto inf = std::numeric_limits<float>::infinity();
    const auto count = echos.size();
    const float distance_scale = 1.0 / sensor_.config().distance_resolution();
    const auto segment_count = sensor_.config().segment_count();

    scan.header.frame_id = frame_id_;
    scan.header.stamp = ros::Time::now();

    // field of view
    scan.angle_min = angles::from_degrees(-field_of_view_ / 2.0);
    scan.angle_max = angles::from_degrees(field_of_view_ / 2.0);
    scan.angle_increment = angles::from_degrees(field_of_view_ / segment_count);
    scan.range_min = min_range_;
    scan.range_max = max_range_;

    // detections
    // TODO(me): intensities
    const size_t detection_count = echos.size();
    scan.ranges.resize(segment_count);
    for (size_t i = 0, j = 0; i != segment_count; i += 1) {
        if (j == detection_count || echos[j].segment_number != i) {
            scan.ranges[i] = inf;
        } else {
            scan.ranges[i] = echos[j].distance * distance_scale;
            j += 1;
        }
    }
}

void Node::Reconfig(leddar_vu8::LeddarVu8Config &config, uint32_t level) {
    ROS_INFO("reconfigure ...");

    // stop streaming
    if (!sensor_.StopStream(retry_)) {
        ROS_ERROR("stop sensor streaming failed");
        return;
    }

    leddar_vu8::Config &c = sensor_.config();

    continuous_ = config.stream;
    c.accumulation_exponent(config.accumulation_exponent);
    c.oversampling_exponent(config.oversampling_exponent);
    c.base_samples(config.base_samples);
    c.smoothing(config.smoothing);
    c.detection_threshold(config.detection_threshold);
    c.light_source_power_percent(config.light_source_power_percent);
    c.saturation_count(config.saturation_count);
    c.auto_light_source_power(config.auto_light_source_power);
    c.auto_light_source_power_enabled(config.auto_light_source_power_enabled);
    c.demerge_object_enabled(config.demerge_object_enabled);
    c.static_noise_removal_enabled(config.static_noise_removal_enabled);
    c.precision_enabled(config.precision_enabled);
    c.saturation_compenstation_enabled(config.saturation_compenstation_enabled);
    c.overshoot_management_enabled(config.overshoot_management_enabled);

    ROS_INFO("saving changes");
    if (!c.Save(retry_, false)) {
        ROS_ERROR("save changes failed, exiting");
        sensor_.StopStream(retry_);
        sensor_.Disconnect();
    }

    // restore streaming
    if (continuous_) {
        if (!sensor_.StartStream(retry_)) {
            ROS_ERROR("start sensor streaming failed");
            sensor_.Disconnect();
            return;
        }
    }

    ROS_INFO("reconfigured");
}
