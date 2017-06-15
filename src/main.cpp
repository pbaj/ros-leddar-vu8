#include <sstream>

#include "./leddar_vu8.h"
#include "./node.h"

namespace leddar_vu8 {

void log(
    LogLevel level,
    const char* file,
    int line,
    const char* function,
    const std::stringstream &msg
) {
    ROSCONSOLE_DEFINE_LOCATION(true, ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME);
    ros::console::levels::Level ros_level;
    switch (level) {
        case leddar_vu8::kLogLevelDebug:
            ros_level = ros::console::levels::Debug;
            break;
        case leddar_vu8::kLogLevelInfo:
            ros_level = ros::console::levels::Info;
            break;
        case leddar_vu8::kLogLevelWarn:
            ros_level = ros::console::levels::Warn;
            break;
        case leddar_vu8::kLogLevelError:
        default:
            ros_level = ros::console::levels::Error;
            break;
    }
    ros::console::print(NULL, __rosconsole_define_location__loc.logger_, ros_level, msg, file, line, function);
}

}  // namespace leddar_vu8

int main(int argc, char** argv) {
    ros::init(argc, argv, "leddar_vu8");
    Node node(ros::NodeHandle("~"));
    if (!node.Initialize()) {
        return -1;
    }
    if (!node.StreamForever()) {
        return -2;
    }
    return 0;
}
