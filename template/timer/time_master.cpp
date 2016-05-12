#include "ros/ros.h"
#include "Definitions.h"
#include "std_msgs/String.h"
#include "logger.h"
#include <std_msgs/Bool.h>
#include <sstream>
#include <sys/time.h>

//Template Code.  This should not be removed.
//Function Prototypes
bool initialize(ros::NodeHandle nh);

//Define general variables, these should be defined for every node.
int rate = 1;
std::string verbosity_level = "";
ros::Publisher pps_pub;
ros::Subscriber pps_sub;  //Not used as this is a pps publisher only.
Logger *logger;
bool require_pps_to_start = false;
bool received_pps = false;


//Define program variables.  These will vary based on the application.
struct timeval current_timer,pps_timer;
long mtime,seconds,useconds;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_master");
    ros::NodeHandle n;
    if(initialize(n) == false)
    {
        return 0; 
    }
    ros::Rate loop_rate(rate);
    gettimeofday(&current_timer,NULL);
    gettimeofday(&pps_timer,NULL);
    while (ros::ok())
    {


        gettimeofday(&current_timer,NULL);
        seconds = current_timer.tv_sec - pps_timer.tv_sec;
        useconds = current_timer.tv_usec - pps_timer.tv_usec;
        mtime = ((seconds)*1000 + useconds/1000.0)+10; //This is just a compensation.  rostopic /hz reports average rate: 1.0
        if(mtime > 1000)
        {
			std_msgs::Bool newstate;
			newstate.data = true;
			pps_pub.publish(newstate);
			gettimeofday(&pps_timer,NULL);
        }
        logger->log_debug("Executive");
		ros::spinOnce();
		loop_rate.sleep();
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
    pps_pub = nh.advertise<std_msgs::Bool>("/pps",1000); //This is a pps source.
    if(nh.getParam("time_master/require_pps_to_start",require_pps_to_start) == false)
    {
    	logger->log_fatal("Missing Parameter: require_pps_to_start.  Exiting.");
    	return false;
    }
    //Free to edit code from here.

    //More Template code here.  Do not edit.
    logger->log_info("Initialized!");
    return true;
}
