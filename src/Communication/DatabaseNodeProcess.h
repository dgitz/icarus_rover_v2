#include "../../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>
/*! \class DatabaseNodeProcess DatabaseNodeProcess.h "DatabaseNodeProcess.h"
 *  \brief This is a DatabaseNodeProcess class.  Used for the database_node node.
 *
 */
class DatabaseNodeProcess: public BaseNodeProcess {
public:
    //Constants
    //Enums
    //Structs
	//Sub-Classes
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic set_config_filepaths(std::string filepath);
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

	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);

	//Support Functions
	//uint8_t map_sampledatatype_ToInt(std::string data);
	//std::string map_sampledatatype_ToString(uint8_t data);
	//bool process_jsonmsg(json msg);

    //Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	eros::diagnostic load_configfile(std::string path);
	std::vector<eros::diagnostic> check_programvariables();
	std::string config_filepath;

	std::map<uint8_t,std::string> sample_map;

};
