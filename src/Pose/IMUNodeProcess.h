#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>
#define RMS_BUFFER_LIMIT 200
#include <eigen3/Eigen/Dense>

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
	struct RotationMatrix
	{
		Eigen::Matrix3f Rotation_Acc_X;
		Eigen::Matrix3f Rotation_Acc_Y;
		Eigen::Matrix3f Rotation_Acc_Z;
		Eigen::Matrix3f Rotation_Acc;
		Eigen::Matrix3f Rotation_Gyro_X;
		Eigen::Matrix3f Rotation_Gyro_Y;
		Eigen::Matrix3f Rotation_Gyro_Z;
		Eigen::Matrix3f Rotation_Gyro;
		Eigen::Matrix3f Rotation_Mag_X;
		Eigen::Matrix3f Rotation_Mag_Y;
		Eigen::Matrix3f Rotation_Mag_Z;
		Eigen::Matrix3f Rotation_Mag;
	};
	struct IMU
	{
		std::string devicename;
        std::string sensor_info_path;
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
		double mounting_angle_offset_roll_deg;
		double mounting_angle_offset_pitch_deg;
		double mounting_angle_offset_yaw_deg;
		double xacc_rms_mean1;
		double yacc_rms_mean1;
		double zacc_rms_mean1;
		double xgyro_rms_mean1;
		double ygyro_rms_mean1;
		double zgyro_rms_mean1;
		double xmag_rms_mean1;
		double ymag_rms_mean1;
		double zmag_rms_mean1;
		RotationMatrix rotate_matrix;

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
	IMU get_imu(std::string devicename);
	bool set_imu_running(std::string devicename);

    bool set_imu_info_path(std::string devicename,std::string path); //Only used for Unit Testing
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
    bool load_sensorinfo(std::string devicename);
    bool set_imu_mounting_angles(std::string devicename,double roll_deg,double pitch_deg,double yaw_deg);
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();
	RotationMatrix generate_rotation_matrix(double mao_roll_deg,double mao_pitch_deg,double mao_yaw_deg);

	std::vector<IMU> imus;
	bool imus_initialized;
	bool imus_running;
};
