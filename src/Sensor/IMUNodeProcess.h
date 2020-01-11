#include "../../include/Base/BaseNodeProcess.cpp"
//C System Files
#include <math.h>       /* sqrt */
//C++ System Files
#include <iostream>

//ROS Base Functionality
//ROS Messages

//Project
#include <tinyxml.h>
#define RMS_BUFFER_LIMIT 200
#define COMM_TIMEOUT_THRESHOLD 10.0f
#define GRAVITATIONAL_ACCELERATION 9.81f
#define MAGNETOMETER_MAGNITUDE_LOWERBOUND 0.5f
#define MAGNETOMETER_MAGNITUDE_UPPERBOUND 1.2f
#include <eigen3/Eigen/Dense>

#include "Driver/IMUDriver.h"
/*! \class IMUNodeProcess IMUNodeProcess.h "IMUNodeProcess.h"
 *  \brief This is a IMUNodeProcess class.  Used for the imu_node node.
 *
 * 
 */
using namespace std;
class IMUNodeProcess: public BaseNodeProcess {
public:
	//Constants
	const double IMU_INVALID_TIME_THRESHOLD = 3.0f;
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
	struct RMS
	{
		double value;
		double mean;
	};
	struct IMU
	{
		std::string partnumber;
		std::string devicename;
		uint64_t serial_number;
		bool serial_number_checked;
        std::string sensor_info_path;
		bool initialized;
		bool running;
		std::string connection_method;
		std::string device_path;
		std::string comm_rate;
		eros::diagnostic diagnostic;
		eros::imu imu_data;
		double acc_scale_factor;
		double gyro_scale_factor;
		double mag_scale_factor;
		uint64_t update_count;
		uint16_t sequence_number;
		uint64_t packet_count;
		double update_rate;
		double temperature;
		double mounting_angle_offset_roll_deg;
		double mounting_angle_offset_pitch_deg;
		double mounting_angle_offset_yaw_deg;
		RMS xacc_rms;
		RMS yacc_rms;
		RMS zacc_rms;
		RMS xgyro_rms;
		RMS ygyro_rms;
		RMS zgyro_rms;
		RMS xmag_rms;
		RMS ymag_rms;
		RMS zmag_rms;
		/*
		double xacc_rms_mean1;
		double yacc_rms_mean1;
		double zacc_rms_mean1;
		double xgyro_rms_mean1;
		double ygyro_rms_mean1;
		double zgyro_rms_mean1;
		double xmag_rms_mean1;
		double ymag_rms_mean1;
		double zmag_rms_mean1;
		*/
		RotationMatrix rotate_matrix;
		double lasttime_rx;
		Eigen::Matrix3f MagnetometerEllipsoidFit_RotationMatrix;
		Eigen::Vector3f MagnetometerEllipsoidFit_Bias;
		Eigen::Vector3f MagnetometerEllipsoidFit_Offset;
		Eigen::Vector3f MagnetometerEllipsoidFit_Scale;

	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	void reset()
	{
		imu_reset_trigger = false;
		imu_reset_inprogress = false;
		imu_reset_inprogress_timer = 0.0;
	}
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	void set_readsensorfile(bool v) { load_sensorfile = v; }
	 bool set_imu_mounting_angles(std::string devicename,double roll_deg,double pitch_deg,double yaw_deg);
	double get_commtimeout_threshold() { return IMU_INVALID_TIME_THRESHOLD; }
	std::vector<IMU> get_imus() { return imus; }
	IMU get_imu(std::string devicename);
	bool set_imu_running(std::string devicename);

	bool get_imus_initialized() { return imus_initialized; }
	bool get_imus_running() { return imus_running; }
	bool get_imureset_trigger() 
	{
		if(imu_reset_trigger == true)
		{
			imu_reset_trigger = false;
			return true;
		}
		return false;
	}
	std::vector<uint64_t> get_imu_serialnumbers()
	{
		std::vector<uint64_t> serial_numbers;
		for(std::size_t i = 0; i < imus.size(); ++i)
		{
			serial_numbers.push_back(imus.at(i).serial_number);
		}
		return serial_numbers;
	}
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device,const eros::leverarm::ConstPtr& leverarm,bool override_config,std::string override_config_path);
	eros::diagnostic new_imumsg(std::string devicename,IMUDriver::RawIMU imu_data,eros::imu &proc_imu,eros::signal &proc_imu_temperature);

	//Support Functions

	//Printing Functions
protected:
private:
	eros::signal convert_signal(IMUDriver::Signal signal);
	std::string map_signalstate_tostring(uint8_t v);
	eros::diagnostic load_sensorinfo(std::string devicename);
	RMS compute_rms(RMS rms,double value,uint64_t update_count);
   
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<eros::diagnostic> check_programvariables();
	RotationMatrix generate_rotation_matrix(double mao_roll_deg,double mao_pitch_deg,double mao_yaw_deg);

	std::vector<IMU> imus;
	bool imus_initialized;
	bool imus_running;
	bool imu_reset_trigger;
	bool imu_reset_inprogress;
	double imu_reset_inprogress_timer;
	bool load_sensorfile;
};
