#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project

/*! \class TimeMasterNodeProcess TimeMasterNodeProcess.h "TimeMasterNodeProcess.h"
 *  \brief This is a TimeMasterNodeProcess class.  Used for the timemaster_node node.
 *
 */
class TimeMasterNodeProcess: public BaseNodeProcess {
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
	bool set_ppssource(std::string v);
	bool publish_1pps();

	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg);
	icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& device);

	//Support Functions

	//Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();
	std::string pps_source;
	double pps1_delay;
	double time_since_last_1pps;
};
