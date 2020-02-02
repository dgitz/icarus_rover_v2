#include "../../include/Base/BaseNodeProcess.cpp"
//C System Files
#include <sys/stat.h>   // For stat().
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project

/*! \class DataLoggerNodeProcess DataLoggerNodeProcess.h "DataLoggerNodeProcess.h"
 *  \brief This is a DataLoggerNodeProcess class.  Used for the sample_node node.
 *
 */
class DataLoggerNodeProcess: public BaseNodeProcess {
public:
    //Constants
    //Enums
    //Structs
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	void reset()
	{
		
	}
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	void set_channelexclude(std::string v) { channel_exclude = v; }
	std::string get_channelexclude() { return channel_exclude; }
    void set_logfileduration(double v) {logfile_duration = v; }
    double get_logfile_duration() { return logfile_duration; }
	bool set_logdirectory(std::string v) 
    { 
        log_directory = v; 
        log_directory_available = false;
		logging_enabled = false;
        struct stat status;
        if(stat(log_directory.c_str(),&status) == 0)
        {
            log_directory_available = true;
			logging_enabled = true;
        }

        return log_directory_available;
    }
	bool is_logging_enabled() { return logging_enabled; }
	std::string get_logdirectory() { return log_directory; }
	bool getSnapshotMode() { return snapshot_mode; }
	void setSnapshotMode(bool v) { snapshot_mode = v; }

	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);

	//Support Functions

    //Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<eros::diagnostic> check_programvariables();
	std::string log_directory;
	bool log_directory_available;
    double logfile_duration;
	bool logging_enabled;
	bool snapshot_mode;
	std::string channel_exclude;


};
