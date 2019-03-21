#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project

/*! \class NavigationNodeProcess NavigationNodeProcess.h "NavigationNodeProcess.h"
 *  \brief This is a NavigationNodeProcess class.  Used for the sample_node node.
 *
 */
class NavigationNodeProcess: public BaseNodeProcess {
public:
    //Constants
    //Enums
    //Structs
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
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
	std::vector<eros::diagnostic> check_programvariables();
};
