#include "ros/ros.h"
#include "Definitions.h"
#include "std_msgs/String.h"
#include "logger.h"
#include <std_msgs/Bool.h>
#include <sstream>

//Template Code.  This should not be removed.
//Function Prototypes
bool initialize(ros::NodeHandle nh);

//Define program variables, these should be defined for every node.
int rate = 1;
std::string verbosity_level = "";
Logger *logger;
void TimeSync_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	logger->log_debug("Got TimeSync");
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sample_node");
    ros::NodeHandle n;
    if(initialize(n) == false)
    {
        return 0; 
    }
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        logger->log_debug("Executive");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

bool initialize(ros::NodeHandle nh)
{
    //Template code.  This should not be changed.
    if(nh.getParam("sample_node/verbosity_level",verbosity_level) == false)
    {
        logger = new Logger("FATAL",ros::this_node::getName());    
        logger->log_fatal("Missing Parameter: verbosity_level. Exiting");
        return false;

    }
    else
    {
        logger = new Logger(verbosity_level,ros::this_node::getName());      
    }
    if(nh.getParam("sample_node/loop_rate",rate) == false)
    {
        logger->log_fatal("Missing Parameter: loop_rate.  Exiting.");
        return false;
    }
    ros::Subscriber timesync_sub = nh.subscribe<std_msgs::Bool>("/timesync",1000,TimeSync_Callback);
    //Free to edit code from here.

    //More Template code here.  Do not edit.
    logger->log_info("Initialized!");
    return true;
}
