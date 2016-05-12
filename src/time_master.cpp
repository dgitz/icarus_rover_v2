#include "ros/ros.h"
#include "Definitions.h"
#include "std_msgs/String.h"
#include "logger.h"
#include <boost/algorithm/string.hpp>
#include <std_msgs/Bool.h>
#include <sstream>
#include <sys/time.h>
#include <stdlib.h>
#include <icarus_rover_v2/resource.h>


//Template Code.  This should not be removed.
//Function Prototypes
bool initialize(ros::NodeHandle nh);
int get_pid();
bool check_resources();

//Define general variables, these should be defined for every node.
std::string node_name;
int rate = 1;
std::string verbosity_level = "";
ros::Publisher pps_pub;
ros::Subscriber pps_sub;  //Not used as this is a pps publisher only.
ros::Publisher resource_pub;
Logger *logger;
bool require_pps_to_start = false;
bool received_pps = false;
int pid = -1;
icarus_rover_v2::resource resources_used;


//Define program variables.  These will vary based on the application.
struct timeval current_timer,pps_timer;
long mtime,seconds,useconds;
int main(int argc, char **argv)
{
	node_name = "time_master";
    ros::init(argc, argv, node_name);
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

			if(check_resources() == false)
			{
				logger->log_fatal("Not able to check Node Resources.  Exiting.");
				return 0;
			}
			resource_pub.publish(resources_used);
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
    pid = get_pid();
    if(pid < 0)
    {
    	logger->log_fatal("Couldn't retrieve PID. Exiting");
    	return false;
    }
    resource_pub =  nh.advertise<icarus_rover_v2::resource>("/time_master/resource",1000); //This is a pps source.
    logger->log_info("Initialized!");
    return true;
}

int get_pid()
{
	int id = -1;
	std::string pid_filename;
	pid_filename = "/tmp/output/PID/" + node_name;
	char tempstr[130];
	sprintf(tempstr,"top -bn1 | grep %s | awk ' { print $1 }' > %s",node_name.c_str(),pid_filename.c_str());
	system(tempstr);  //First entry should be PID
	ifstream myfile;
	myfile.open(pid_filename.c_str());
	if(myfile.is_open())
	{
		std::string line;
		getline(myfile,line);
		id =  atoi(line.c_str());
	}
	else
	{
		id = -1;
	}
	myfile.close();
	return id;
}
bool check_resources()
{
	std::string resource_filename;
	resource_filename = "/tmp/output/RESOURCE/" + node_name;
	char tempstr[130];
	sprintf(tempstr,"top -bn1 | grep %d > %s",pid,resource_filename.c_str());
	system(tempstr); //RAM used is column 6, in KB.  CPU used is column 9, in percentage.
	ifstream myfile;
	myfile.open(resource_filename.c_str());
	if(myfile.is_open())
	{
		std::string line;
		getline(myfile,line);
		std::vector<std::string> strs;
		boost::split(strs,line,boost::is_any_of(" "),boost::token_compress_on);
		resources_used.PID = pid;
		resources_used.CPU_Perc = atoi(strs.at(9).c_str());
		resources_used.RAM_MB = atoi(strs.at(6).c_str())/1000.0;
		return true;
	}
	else
	{
		return false;
	}
	myfile.close();

}
