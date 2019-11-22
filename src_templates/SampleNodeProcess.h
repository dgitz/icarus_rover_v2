#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>
/*! \class SampleNodeProcess SampleNodeProcess.h "SampleNodeProcess.h"
 *  \brief This is a SampleNodeProcess class.  Used for the sample_node node.
 *
 */
class SampleNodeProcess: public BaseNodeProcess {
public:
    //Constants
	const uint8_t SAMPLE_CONSTANT = 0;
    //Enums
	enum class SampleEnum 
	{
		UNKNOWN=0,
		ENUMTYPEA=1,
		ENUMTYPEB=2,
	};
    //Structs
	struct SampleStruct
	{
		std::string field;
	};
	//Sub-Classes
	class SampleSubClass
	{
		public:
		bool initialize(std::string t_name)
		{
			name = t_name;
			return true;
		}
		std::string getName()
		{
			return name;
		}
		private:

			std::string name;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	void reset()
	{
	}
	eros::diagnostic set_config_filepaths(std::string filepath);
	eros::diagnostic finish_initialization();
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
	uint8_t map_sampledatatype_ToInt(std::string data);
	std::string map_sampledatatype_ToString(uint8_t data);
	bool process_jsonmsg(json msg);

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
