#ifndef __NODE_H__
#define __NODE_H__

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>

#include <leddar_vu8/LeddarVu8Config.h>

#include "leddar_vu8.h"

class Node {
public:
    Node(ros::NodeHandle nh);
    ~Node();

    bool Initialize();
    void Close();
    bool StreamForever();

private:
    void ToLaserScan(
        const std::vector<leddar_vu8::Echo> &echoes,
        sensor_msgs::LaserScan& scan
    );
    void Reconfig(leddar_vu8::LeddarVu8Config &config, uint32_t level);

    ros::NodeHandle nh_;
    ros::Publisher scan_pub_;
    leddar_vu8::Sensor sensor_;
    std::string frame_id_ = "laser";
    double rate_ = 50;
    double field_of_view_ = 100;
    double min_range_ = 0;
    double max_range_ = 20;
    double listen_timeout_ = 1.0;
    dynamic_reconfigure::Server<leddar_vu8::LeddarVu8Config> reconfig_;
};

#endif // __NODE_H__
