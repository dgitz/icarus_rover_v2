#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
#include <ctime>
#include <sys/stat.h> 
#include <sys/types.h> 
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>
/*! \class SnapshotNodeProcess SnapshotNodeProcess.h "SnapshotNodeProcess.h"
 *  \brief This is a SnapshotNodeProcess class.  Used for the snapshot_node node.
 *
 */
class SnapshotNodeProcess: public BaseNodeProcess {
public:
    //Constants
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
		READY=3
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
		std::string destination_path;
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
	bool isSnapshotComplete(std::string &path,std::string &name)
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
			devicesnapshot_state = SnapshotState::NOTRUNNING;
			return true;
		}
	}

	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	std::string getDeviceSnapshotPath() { return snapshot_config.destination_path; }
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this  Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);

	//Support Functions
	std::string map_state_tostring(SnapshotState t_state);
    //Printing Functions
	void print_snapshotconfig(SnapshotNodeProcess::SnapshotConfig config);
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	eros::diagnostic load_configfile(std::string path);
	std::vector<eros::diagnostic> check_programvariables();
	std::vector<eros::diagnostic> createnew_snapshot();
	void clear_allsnapshots();
	std::string exec(const char* cmd,bool wait_for_result);

	InstanceMode mode;
	std::string snapshot_name;
	std::string snapshot_path;
	std::string systemsnapshot_directory;
	std::string systemsnapshot_name;
	std::string systemsnapshot_path;
	SnapshotState devicesnapshot_state;
	SnapshotState systemsnapshot_state;
	std::string config_filepath;
	std::vector<SnapshotConfig> snapshot_configlist;
	SnapshotConfig snapshot_config;


};
