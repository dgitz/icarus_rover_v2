#include "ros/ros.h"
#include "Definitions.h"
#include "std_msgs/String.h"
#include "logger.h"
#include <std_msgs/Bool.h>
#include <sstream>

//Template Code.  This should not be removed.
//Function Prototypes
bool initialize(ros::NodeHandle nh);

//Define general variables, these should be defined for every node.
int rate = 1;
std::string verbosity_level = "";
Logger *logger;
ros::Publisher pps_pub; //Not used as this is a pps subscriber only.
ros::Subscriber pps_sub;
bool require_pps_to_start = false;
bool received_pps = false;

//Define program variables.  These will vary based on the application.


void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	logger->log_info("Got pps");
	received_pps = true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_slave");
    ros::NodeHandle n;
    if(initialize(n) == false)
    {
        return 0; 
    }
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		logger->log_debug("Executive");
    		//User Code goes here.
    	}
    	else
    	{
    		logger->log_warn("Waiting on PPS to Start.");
    	}
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

bool initialize(ros::NodeHandle nh)
{
    //Template code.  This should not be changed.
    if(nh.getParam("time_slave/verbosity_level",verbosity_level) == false)
    {
        logger = new Logger("FATAL",ros::this_node::getName());    
        logger->log_fatal("Missing Parameter: verbosity_level. Exiting");
        return false;
    }
    else
    {
        logger = new Logger(verbosity_level,ros::this_node::getName());      
    }
    if(nh.getParam("time_slave/loop_rate",rate) == false)
    {
        logger->log_fatal("Missing Parameter: loop_rate.  Exiting.");
        return false;
    }
    pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
    if(nh.getParam("time_slave/require_pps_to_start",require_pps_to_start) == false)
	{
		logger->log_fatal("Missing Parameter: require_pps_to_start.  Exiting.");
		return false;
	}
    //Free to edit code from here.

    //More Template code here.  Do not edit.
    logger->log_info("Initialized!");
    return true;
}
