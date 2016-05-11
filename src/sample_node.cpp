#include "ros/ros.h"
#include "Definitions.h"
#include "std_msgs/String.h"
#include "logger.h"

#include <sstream>

bool initialize(ros::NodeHandle nh);
//Define program variables, these should be defined for every node.
std::string verbosity_level = "";
Logger *logger;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sample_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    if(initialize(n) == false)
    {
        return 0; 
    }
    else
    {
        
    }
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

bool initialize(ros::NodeHandle nh)
{
    int temp;
    nh.getParam("temp",temp);
    printf("Temp: %d\r\n",temp);
    if(nh.hasParam("verbosity_level"))
    {
        printf("Missing param: verbosity_level. Exiting.\r\n");
        return false;
    }
    else
    {
        nh.getParam("verbosity_level",verbosity_level);
        printf("Got: %s\r\n",verbosity_level.c_str());
        logger = new Logger(verbosity_level,ros::this_node::getName());
        
    }
    return true;
    //nh.getParam("verbosity_level",verbosity_level);
}
