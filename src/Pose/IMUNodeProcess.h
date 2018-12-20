#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include "Driver/IMUDriver.h"
/*! \class IMUNodeProcess IMUNodeProcess.h "IMUNodeProcess.h"
 *  \brief This is a IMUNodeProcess class.  Used for the imu_node node.
 *
 */
class IMUNodeProcess: public BaseNodeProcess {
public:
    //Constants
    //Enums
    //Structs
    struct IMU
    {
        std::string devicename;
        bool initialized;
        bool running;
        std::string connection_method;
        std::string device_path;
        std::string comm_rate;
        icarus_rover_v2::diagnostic diagnostic;
        icarus_rover_v2::imu imu_data;
        double acc_scale_factor;
        double gyro_scale_factor;
        double mag_scale_factor;
        uint64_t update_count;
        double update_rate;
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
    std::vector<IMU> get_imus() { return imus; }
    bool set_imu_running(std::string devicename);
    bool get_imus_initialized() { return imus_initialized; }
    bool get_imus_running() { return imus_running; }
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg);
	icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& device);
    icarus_rover_v2::diagnostic new_imumsg(std::string devicename,IMUDriver::RawIMU imu_data,icarus_rover_v2::imu &proc_imu);

	//Support Functions

    //Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();
    
    std::vector<IMU> imus;
    bool imus_initialized;
    bool imus_running;
};
