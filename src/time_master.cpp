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
ros::Publisher timesync_pub;
ros::Subscriber timesync_sub;
Logger *logger;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_master");
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
        std_msgs::Bool newstate;
        newstate.data = true;
        timesync_pub.publish(newstate);
    }
    return 0;
}

bool initialize(ros::NodeHandle nh)
{
    //Template code.  This should not be changed.
    if(nh.getParam("time_master/verbosity_level",verbosity_level) == false)
    {
        logger = new Logger("FATAL",ros::this_node::getName());    
        logger->log_fatal("Missing Parameter: verbosity_level. Exiting");
        return false;

    }
    else
    {
        logger = new Logger(verbosity_level,ros::this_node::getName());      
    }
    if(nh.getParam("time_master/loop_rate",rate) == false)
    {
        logger->log_fatal("Missing Parameter: loop_rate.  Exiting.");
        return false;
    }
    timesync_pub = nh.advertise<std_msgs::Bool>("/timesync",1000);
    //Free to edit code from here.

    //More Template code here.  Do not edit.
    logger->log_info("Initialized!");
    return true;
}
