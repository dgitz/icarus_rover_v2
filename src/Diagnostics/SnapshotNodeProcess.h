#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
#include <ctime>
#include <sys/stat.h> 
#include <sys/types.h> 
//C++ System Files
#include <fstream>
#include <iostream>
//ROS Base Functionality
//ROS Messages
#include "std_msgs/Empty.h"
#include <eros/systemsnapshot_state.h>
//Project
#include <tinyxml.h>
/*! \class SnapshotNodeProcess SnapshotNodeProcess.h "SnapshotNodeProcess.h"
 *  \brief This is a SnapshotNodeProcess class.  Used for the snapshot_node node.
 *
 */
#define SYSTEMSNAPSHOT_TIMEOUT 5.0f //Seconds before the Node stops caring if a device snapshot zip file is received.
class SnapshotNodeProcess: public BaseNodeProcess {
public:
    //Constants
	const double TIMETOHOLD_SYSTEMSNAPSTATE_SEC = 10.0;
    //Enums
	enum class InstanceMode
	{
		UNKNOWN=0,
		MASTER=1,
		SLAVE=2
	};
	enum class SnapshotState
	{
		UNKNOWN=0,
		NOTRUNNING=1,
		RUNNING=2,
		READY=3,
		COMPLETE=4,
		INCOMPLETE=5
	};
    //Structs
	struct Command
	{	
		std::string command;
		std::string output_file;
	};
	struct SnapshotConfig
	{
		std::string architecture;
		std::vector<std::string> folders;
		std::vector<std::string> files;
		std::vector<Command> commands;
		std::vector<std::string> scripts;
		std::string destination_path;
	};
	struct SystemSnapshotInfo
	{
		double rostime_start;
		double rostime_stop;
		SnapshotState state;
		std::string generate_commandtext;
		std::string generate_commanddescription;
		uint8_t snapshot_mode;
		std::vector<std::string> devices;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic set_config_filepaths(std::string filepath);
	eros::diagnostic finish_initialization();
	eros::diagnostic setInstanceMode(std::string t_mode);
	InstanceMode getInstanceMode() { return mode; }
	SnapshotState getDeviceSnapshotState() { return devicesnapshot_state; }
	SnapshotState getSystemSnapshotState() { return systemsnapshot_state; }
	bool isSystemSnapshotComplete(eros::diagnostic &diag)
	{
		diag = root_diagnostic;
		switch(systemsnapshot_state)
		{
			case SnapshotState::READY:
				diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"System Snapshot Ready.");
				return true;
				break;
			case SnapshotState::INCOMPLETE:
				diag = update_diagnostic(DATA_STORAGE,NOTICE,DROPPING_PACKETS,"System Snapshot Ready But Incomplete.");
				return true;
				break;
			case SnapshotState::NOTRUNNING:
				diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"System Snapshot Not Running.");
				return false;
				break;	
			case SnapshotState::RUNNING:
				diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"System Snapshot Generation in Progress.");
				return false;
				break;
			default:
				diag = update_diagnostic(DATA_STORAGE,ERROR,DROPPING_PACKETS,"System Snapshot in Unknown State.");
				return false;
				break;
		}
	}
	bool isDeviceSnapshotComplete(std::string &path,std::string &name)
	{
		path = "";
		name = "";
		if(devicesnapshot_state != SnapshotState::READY)
		{
			return false;
		}
		else
		{
			path = snapshot_path;
			name = snapshot_name;
			return true;
		}
	}

	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);
	eros::diagnostic update_slow();
	void resetDeviceSnapshotState()
	{
		devicesnapshot_state = SnapshotState::NOTRUNNING;
	}
	eros::diagnostic finishSystemSnapshot(); //Zip it up
	//Attribute Functions
	SystemSnapshotInfo getSystemSnapshotInfo() { return systemsnapshot_info; }
	std::string getDeviceSnapshotPath() { return snapshot_config.destination_path; }
	std::vector<std::string> get_allsnapshot_devices() { return all_snapshot_devices; }
	std::vector<std::string> get_snapshot_devices_toquery() { return filtered_snapshot_devices; }
	std::string getDeviceActiveSnapshotCompletePath() { return active_snapshot_completepath; }
	std::string getDeviceActiveSnapshotName() { return snapshot_name; }
	std::string getSystemActiveSnapshotDirectory()
	{
		if(systemsnapshot_state == SnapshotState::RUNNING)
		{
			return systemsnapshot_path;
		}
		else
		{
			return "";
		}
	}
	std::string getSystemActiveSnapshotCompletePath() 
	{ 	
		if(systemsnapshot_state == SnapshotState::READY)
		{
			return active_systemsnapshot_completepath; 
		}
		else
		{
			return "";
		}
		
	}
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this  Function.
	 *
	 */
	eros::diagnostic new_truthpose(const eros::pose::ConstPtr& t_ptr);
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);
	bool received_snapshot_fromdevice(std::string devicename);
	eros::systemsnapshot_state getROSSnapshotState() { return eros_systemsnapshot_state; }

	//Support Functions
	std::string map_state_tostring(SnapshotState t_state);
	std::string map_snapshotmode_tostring(uint8_t t_mode);
	
	int count_files_indirectory(std::string directory,std::string filter);
    //Printing Functions
	void print_snapshotconfig(SnapshotNodeProcess::SnapshotConfig config);
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	eros::diagnostic load_configfile(std::string path);
	std::vector<eros::diagnostic> check_programvariables();
	std::vector<eros::diagnostic> createnew_snapshot(uint8_t snapshot_mode,std::string command_text,std::string description);
	std::string generate_systemsnapshotinfo(SystemSnapshotInfo info);
	void clear_allsnapshots();
	
	PoseHelper pose_helper;
	InstanceMode mode;
	std::string snapshot_name;
	std::string snapshot_path;
	std::string active_snapshot_completepath;
	std::string active_systemsnapshot_completepath;
	std::string systemsnapshot_directory;
	std::string systemsnapshot_name;
	std::string systemsnapshot_path;
	SnapshotState devicesnapshot_state;
	SnapshotState systemsnapshot_state;
	std::string config_filepath;
	std::string datalog_directory;
	std::string datalog_device;
	std::vector<SnapshotConfig> snapshot_configlist;
	SnapshotConfig snapshot_config;
	std::vector<std::string> all_snapshot_devices;
	std::vector<std::string> filtered_snapshot_devices;
	std::vector<std::string> missing_snapshots;
	bool run_systemsnapshot_timeout_timer;
	double systemsnapshot_timeout_timer;
	double systemsnapshot_complete_timer;
	SystemSnapshotInfo systemsnapshot_info;
	eros::systemsnapshot_state eros_systemsnapshot_state;
	std::string systemsnapinfo_extratext;
	std::string truthpose_string;


};
