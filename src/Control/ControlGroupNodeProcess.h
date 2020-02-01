#include "../../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>
#include "fl/Headers.h"
#include "../../../eROS/include/controlgroup.h"
/*! \class ControlGroupNodeProcess ControlGroupNodeProcess.h "ControlGroupNodeProcess.h"
 *  \brief This is a ControlGroupNodeProcess class.  Used for the implement_node node.
 *
 */
class ControlGroupNodeProcess: public BaseNodeProcess {
public:
    //Constants
    //Enums
    //Structs
	class FuzzyController
	{
		public:
		bool initialize(std::string name)
		{
			engine = new fl::Engine;
			engine->setName(name);
			return true;
		}
		std::string getName()
		{
			return engine->getName();
		}
		private:

			fl::Engine* engine;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic set_config_filepaths(std::string filepath);
	eros::diagnostic finish_initialization();
	void reset()
	{
		for(std::size_t i = 0; i < controlgroups.size(); ++i)
		{
			controlgroups.at(i).reset_integral();
		}
	}
	eros::diagnostic set_PIDGains(std::string controlgroup_name,double P,double I,double D);
	eros::diagnostic set_pinproperties(eros::pin pin);
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);
	eros::diagnostic new_inputsignalmsg(const eros::signal::ConstPtr& t_msg);

	//Attribute Functions
	void clear_controlgroups()
	{
		controlgroups.clear();
	}
	std::vector<ControlGroup> get_controlgroups() { return controlgroups; }
	std::vector<eros::signal> get_outputsignals()
	{
		std::vector<eros::signal> signals;
		for(std::size_t i = 0; i < controlgroups.size(); ++i)
		{
			std::vector<eros::signal> outs = controlgroups.at(i).get_outputsignals();
			for(std::size_t j = 0; j < outs.size(); ++j)
			{
				signals.push_back(outs.at(j));
			}
		}
		return signals;
	}
	std::vector<eros::pin> get_outputpins()
	{
		std::vector<eros::pin> pins;
		for(std::size_t i = 0; i < controlgroups.size(); ++i)
		{
			std::vector<eros::pin> outs = controlgroups.at(i).get_outputpins();
			for(std::size_t j = 0; j < outs.size(); ++j)
			{
				pins.push_back(outs.at(j));
			}
		}
		return pins;
	}
	std::vector<eros::view_controlgroup> get_controlgroupviews();
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);

	//Support Functions
	ControlGroup::Mode map_controlgroupmode_toenum(std::string v);
	ControlGroup::SignalClass map_signalclass_toenum(std::string v);
    //Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	eros::diagnostic load_configfile(std::string path);
	std::vector<eros::diagnostic> check_programvariables();
	std::vector<ControlGroup> controlgroups;
	std::string config_filepath;
	FuzzyController bucket_cylinder_controller;

};
