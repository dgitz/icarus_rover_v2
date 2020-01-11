#include "../../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
#include <visualization_msgs/Marker.h>
//Project

/*! \class CalibrationNodeProcess CalibrationNodeProcess.h "CalibrationNodeProcess.h"
 *  \brief This is a CalibrationNodeProcess class.  Used for the sample_node node.
 *
 */
class CalibrationNodeProcess: public BaseNodeProcess {
public:
    //Constants
    //Enums
    //Structs
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	void reset()
	{
		calibration_mode_changed = false;
		calibration_mode = ROVERCOMMAND_CALIBRATION_NONE;
	}
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions

	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Calibration Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);

	//Support Functions

    //Printing Functions
protected:
private:
	void set_calibration_mode(uint8_t mode);
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<eros::diagnostic> check_programvariables();
	uint8_t calibration_mode;
	bool calibration_mode_changed;
};
