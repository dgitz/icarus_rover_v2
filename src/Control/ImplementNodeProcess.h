#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>
#include "fl/Headers.h"
/*! \class ImplementNodeProcess ImplementNodeProcess.h "ImplementNodeProcess.h"
 *  \brief This is a ImplementNodeProcess class.  Used for the implement_node node.
 *
 */
class ImplementNodeProcess: public BaseNodeProcess {
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

    //Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	eros::diagnostic load_configfile(std::string path);
	std::vector<eros::diagnostic> check_programvariables();
	std::string config_filepath;
	FuzzyController bucket_cylinder_controller;

};
