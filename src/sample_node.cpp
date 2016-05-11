#include "ros/ros.h"
#include "std_msgs/String.h"
#include "logger.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sample_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    Logger *logger = new Logger(ros::this_node::getName());
    while (ros::ok())
    {
        logger->log_debug("debug1");
        logger->log_info("info1");
        logger->log_notice("notice1");
        logger->log_warn("warn1");
        logger->log_error("error1");
        logger->log_fatal("fatal1");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
