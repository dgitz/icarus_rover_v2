#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project

/*! \class SafetyNodeProcess SafetyNodeProcess.h "SafetyNodeProcess.h"
 *  \brief This is a SafetyNodeProcess class.  Used for the safety_node node.
 *
 */
class SafetyNodeProcess: public BaseNodeProcess {
public:
    //Constants
    //Enums
    //Structs
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	icarus_rover_v2::diagnostic finish_initialization();
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	icarus_rover_v2::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions

	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg);
	icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& t_device);
    icarus_rover_v2::diagnostic new_armswitchmsg(std_msgs::Bool v);

	//Support Functions

    //Printing Functions
	
	//Generic Hat Functions
	bool hat_present(const icarus_rover_v2::device::ConstPtr& t_device);
    icarus_rover_v2::diagnostic set_hat_running(std::string devicetype,uint16_t id);
	bool is_hat_running(std::string devicetype,uint16_t id);

	//Terminal Hat Functions
	icarus_rover_v2::diagnostic set_terminalhat_initialized();
	std::vector<icarus_rover_v2::pin> get_terminalhatpins(std::string Function);
	int get_pinnumber(std::string name);
	bool set_pinvalue(std::string name,int v);
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();
    bool arm_switch;
    std::vector<icarus_rover_v2::device> hats;
	std::vector<bool> hats_running;
};
