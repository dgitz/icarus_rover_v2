#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>
/*! \class NavigationNodeProcess NavigationNodeProcess.h "NavigationNodeProcess.h"
 *  \brief This is a NavigationNodeProcess class.  Used for the sample_node node.
 *
 */
class NavigationNodeProcess: public BaseNodeProcess {
public:
	//Constants
	//Enums
	enum class ControlGroupMode
	{
		UNKNOWN=0,
				ARCADE=1,
	};
	enum class ControlGroupInputType
	{
		UNKNOWN=0,
				THROTTLE=1,
				STEER=2,
	};
	enum class ControlGroupOutputType
	{
		UNKNOWN=0,
				LEFTDRIVE=1,
				RIGHTDRIVE=2,
	};
	//Structs
	struct ControlGroupInput
	{
		ControlGroupInputType type;
		std::string name;
	};
	struct ControlGroupOutput
	{
		ControlGroupOutputType type;
		std::string name;
		eros::pin pin;
	};
	struct ControlGroup
	{
		std::string name;
		ControlGroupMode mode;
		std::vector<ControlGroupInput> inputs;
		std::vector<ControlGroupOutput> outputs;
		double time_since_lastupdate;
	};
	struct DrivePerc
	{
		double left;
		double right;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	eros::diagnostic load_controlgroupfile();
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	eros::diagnostic fetch_complete()
	{
		eros::diagnostic diag = diagnostic;
		if(initialized == true)
		{
			ready = true;
		}
		return diag;
	}
	void set_filepaths(std::string _controlgroup_filepath)
	{
		controlgroup_filepath = _controlgroup_filepath;
	}
	std::vector<ControlGroup> get_controlgroups() { return control_groups; }
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);
	std::vector<eros::pin> get_pins()
	{
		std::vector<eros::pin> pins;
		for(std::size_t i = 0; i < control_groups.size(); ++i)
		{
			for(std::size_t j=0; j < control_groups.at(i).outputs.size(); ++j)
			{
				pins.push_back(control_groups.at(i).outputs.at(j).pin);
			}
		}
		return pins;
	}

	//Support Functions
	DrivePerc arcade_mix(double throttle_perc,double steer_perc);
	std::string map_ControlGroupMode_ToString(ControlGroupMode v);
	ControlGroupMode map_ControlGroupMode_ToEnum(std::string v);
	std::string map_ControlGroupInputType_ToString(ControlGroupInputType v);
	ControlGroupInputType map_ControlGroupInputType_ToEnum(std::string v);
	std::string map_ControlGroupOutputType_ToString(ControlGroupOutputType v);
	ControlGroupOutputType map_ControlGroupOutputType_ToEnum(std::string v);
	double scale_value(double input_perc,double y1,double neutral,double y2);
	//Printing Functions
	void print_controlgroups();
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */

	eros::diagnostic update_controlgroups(json cmd);
	std::map<ControlGroupMode,std::string> controlgroup_mode_map;
	std::map<ControlGroupInputType,std::string> controlgroup_inputtype_map;
	std::map<ControlGroupOutputType,std::string> controlgroup_outputtype_map;
	std::string controlgroup_filepath;
	std::vector<eros::diagnostic> check_programvariables();
	std::vector<ControlGroup> control_groups;
	std::vector<eros::pin> all_pins;

};
