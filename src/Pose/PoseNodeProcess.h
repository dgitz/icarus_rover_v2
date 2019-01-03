#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include "PoseModel/Definitions/PoseDefinitions.h"
#include <tf/transform_broadcaster.h>

/*! \class PoseNodeProcess PoseNodeProcess.h "PoseNodeProcess.h"
 *  \brief This is a PoseNodeProcess class.  Used for the pose_node node.
 *
 */
class PoseNodeProcess: public BaseNodeProcess {
public:
    //Constants
    //Enums
    enum PoseMode
    {
        UNKNOWN=0,
        CALIBRATE=1
    };
    //Structs
	struct IMUSensor
	{
		bool initialized;
		bool running;
		std::string topicname;
        tf::Transform transform;
		icarus_rover_v2::device device;
		icarus_rover_v2::imu imu_data;
		icarus_rover_v2::signal orientation_pitch;
		icarus_rover_v2::signal orientation_roll;
		icarus_rover_v2::signal orientation_yaw;
	};
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
	bool set_imucount(uint8_t v);
	IMUSensor get_imudata(std::string name);
    std::vector<IMUSensor> get_imus() { return imus; }

	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg);
	icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& device);
	icarus_rover_v2::diagnostic new_imumsg(std::string topic, const icarus_rover_v2::imu::ConstPtr& data);

	//Support Functions
    std::string map_posemode_tostring(PoseNodeProcess::PoseMode t_posemode);
    PoseNodeProcess::PoseMode map_posemode_tovalue(std::string t_posemode);
    double compute_acceleration_based_roll(double xacc,double yacc,double zacc);
    double compute_acceleration_based_pitch(double xacc,double yacc,double zacc);

    //Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
    PoseAcceleration pose_acc;
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();
    PoseMode current_mode;
	std::vector<IMUSensor> imus;
	uint8_t imu_count;
};
