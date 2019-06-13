#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include<boost/algorithm/string/split.hpp>
#include<boost/algorithm/string.hpp>
#include "../IMUNodeProcess.h"
#define DIAGNOSTIC_TYPE_COUNT 4

std::string Node_Name = "/unittest_imu_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
void print_diagnostic(uint8_t level,eros::diagnostic diagnostic)
{
	if(diagnostic.Level >= level)
	{
		printf("Type: %d Message: %d Level: %d Device: %s Desc: %s\n",diagnostic.Diagnostic_Type,diagnostic.Diagnostic_Message,
			  		diagnostic.Level,diagnostic.DeviceName.c_str(),diagnostic.Description.c_str());
	}
}
void print_3x3matrix(std::string name,Eigen::Matrix3f mat)
{
	printf("Matrix: %s\n",name.c_str());
	printf("%f %f %f\r\n%f %f %f\n%f %f %f\n",
			mat(0,0),
			mat(0,1),
			mat(0,2),
			mat(1,0),
			mat(1,1),
			mat(1,2),
			mat(2,0),
			mat(2,1),
			mat(2,2));

}
IMUNodeProcess* initializeprocess(std::string imuname,std::string imu_partnumber,uint64_t *id,std::string override_config_path)
{
	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 1;
	device.DeviceParent = "None";
	device.Architecture = "armv7l";

	eros::device imu;
	imu.DeviceName = imuname;
	imu.DeviceParent = ros_DeviceName;
	imu.DeviceType = "IMU";
	imu.ID = 17;
	*id = imu.ID;
	imu.PartNumber = imu_partnumber;


	IMUNodeProcess *process;
	process = new IMUNodeProcess;
	process->initialize("imu_node",Node_Name,Host_Name,ROVER,ROBOT_CONTROLLER,POSE_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(SENSORS);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	EXPECT_TRUE(process->is_ready() == false);
	eros::device::ConstPtr imu_ptr(new eros::device(imu));
	eros::leverarm leverarm;

	if(imuname == "IMU1")
	{
		leverarm.name = imuname;
		leverarm.reference = "BodyOrigin";
		leverarm.x.value = 0.0;
		leverarm.y.value = 0.0;
		leverarm.z.value = 0.0;
		leverarm.roll.value = 0.0;
		leverarm.pitch.value = 0.0;
		leverarm.yaw.value = 0.0;
	}
	else if(imuname == "IMU2")
	{
		leverarm.name = imuname;
		leverarm.reference = "BodyOrigin";
		leverarm.x.value = 0.0;
		leverarm.y.value = 0.0;
		leverarm.z.value = 0.0;
		leverarm.roll.value = -10.0;
		leverarm.pitch.value = -30.0;
		leverarm.yaw.value = -170.0;
	}
	eros::leverarm::ConstPtr leverarm_ptr(new eros::leverarm(leverarm));
	eros::diagnostic diagnostic = process->new_devicemsg(imu_ptr,leverarm_ptr,true,override_config_path);
	print_diagnostic(NOTICE,diagnostic);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.0,0.0);
	EXPECT_TRUE(process->is_ready() == true);
	{
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT +(2*process->get_imus().size())));
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			if(diagnostics.at(i).Diagnostic_Type == SENSORS) //Should not be ok until the IMU is running and a message is received.
			{

			}
			else
			{
				EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
			}
		}
	}
	EXPECT_TRUE(process->get_imus_initialized() == true);
	EXPECT_TRUE(process->get_imus_running() == false);

	return process;
}
TEST(Template,Process_Initialization)
{
	uint64_t id = 0;
	initializeprocess("IMU1","110012",&id,"/home/robot/catkin_ws/src/icarus_rover_v2/src/Sensor/unit_tests/IMU1.xml");
}
TEST(Template,Process_Msg)
{
	uint64_t id = 0;
	IMUNodeProcess* process = initializeprocess("IMU1","110012",&id,"/home/robot/catkin_ws/src/icarus_rover_v2/src/Sensor/unit_tests/IMU1.xml");
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	bool all_valid_msg_sent = false;
	while(current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt,current_time);
		if(all_valid_msg_sent == true)
		{
			EXPECT_TRUE(process->get_ready_to_arm() == true);
		}
		EXPECT_TRUE(diag.Level <= NOTICE);
		int current_time_ms = (int)(current_time*1000.0);
		if((current_time_ms % 100) == 0)
		{
			fastrate_fire = true;
		}
		else { fastrate_fire = false; }
		if((current_time_ms % 1000) == 0)
		{
			mediumrate_fire = true;
		}
		else { mediumrate_fire = false; }
		if((current_time_ms % 10000) == 0)
		{
			slowrate_fire = true;
		}
		else { slowrate_fire = false; }

		if(fastrate_fire == true) //Nothing to do here
		{
			if(process->get_imus_running() == false)
			{
				std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
				for(std::size_t i = 0; i < imus.size(); ++i)
				{
					EXPECT_TRUE(process->set_imu_running(imus.at(i).devicename));
				}
			}
			EXPECT_TRUE(process->get_imus_running());
			std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				IMUDriver::RawIMU raw_imudata;
				raw_imudata.serial_number = id;
				raw_imudata.signal_state = SIGNALSTATE_UPDATED;
				raw_imudata.serial_number = id;
				raw_imudata.tov = current_time;
				raw_imudata.acc_x.value = 1.0;
				raw_imudata.acc_y.value = 2.0;
				raw_imudata.acc_z.value = 3.0;
				raw_imudata.gyro_x.value = 4.0;
				raw_imudata.gyro_y.value = 5.0;
				raw_imudata.gyro_z.value = 6.0;
				raw_imudata.mag_x.value = 7.0;
				raw_imudata.mag_y.value = 8.0;
				raw_imudata.mag_z.value = 9.0;
				eros::imu processed_imudata;
				eros::signal processed_imudata_temperature;
				diag = process->new_imumsg(imus.at(i).devicename,raw_imudata,processed_imudata,processed_imudata_temperature);
				EXPECT_TRUE(diag.Level <= NOTICE);
				all_valid_msg_sent = true;
				EXPECT_TRUE((fabs((raw_imudata.acc_x.value/imus.at(i).acc_scale_factor)-processed_imudata.xacc.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.acc_y.value/imus.at(i).acc_scale_factor)-processed_imudata.yacc.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.acc_z.value/imus.at(i).acc_scale_factor)-processed_imudata.zacc.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.gyro_x.value/imus.at(i).gyro_scale_factor)-processed_imudata.xgyro.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.gyro_y.value/imus.at(i).gyro_scale_factor)-processed_imudata.ygyro.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.gyro_z.value/imus.at(i).gyro_scale_factor)-processed_imudata.zgyro.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.mag_x.value/imus.at(i).mag_scale_factor)-processed_imudata.xmag.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.mag_y.value/imus.at(i).mag_scale_factor)-processed_imudata.ymag.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.mag_z.value/imus.at(i).mag_scale_factor)-processed_imudata.zmag.value) < .0001));
				
			}

		}
		if(mediumrate_fire == true)
		{
		}
		if(slowrate_fire == true)
		{
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT + (2*process->get_imus().size())));
				for (std::size_t i = 0; i < diagnostics.size(); ++i)
				{
					EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
				}
				return;
			}
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}

TEST(Template,Process_BadMsg)
{
	return;
	uint64_t id = 0;
	IMUNodeProcess* process = initializeprocess("IMU1","110012",&id,"/home/robot/catkin_ws/src/icarus_rover_v2/src/Sensor/unit_tests/IMU1.xml");
	double time_to_run = 1.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	bool message_sent = false;
	while(current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt,current_time);
		int current_time_ms = (int)(current_time*1000.0);
		if((current_time_ms % 100) == 0)
		{
			fastrate_fire = true;
		}
		else { fastrate_fire = false; }
		if((current_time_ms % 1000) == 0)
		{
			mediumrate_fire = true;
		}
		else { mediumrate_fire = false; }
		if((current_time_ms % 10000) == 0)
		{
			slowrate_fire = true;
		}
		else { slowrate_fire = false; }

		if(fastrate_fire == true) //Nothing to do here
		{
			if(process->get_imus_running() == false)
			{
				std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
				for(std::size_t i = 0; i < imus.size(); ++i)
				{
					EXPECT_TRUE(process->set_imu_running(imus.at(i).devicename));
				}
			}
			EXPECT_TRUE(process->get_imus_running());
			std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				IMUDriver::RawIMU raw_imudata;
				raw_imudata.serial_number = id;
				raw_imudata.signal_state = SIGNALSTATE_INVALID;
				raw_imudata.tov = current_time;
				raw_imudata.acc_x.value = 1.0;
				raw_imudata.acc_y.value = 2.0;
				raw_imudata.acc_z.value = 3.0;
				raw_imudata.gyro_x.value = 4.0;
				raw_imudata.gyro_y.value = 5.0;
				raw_imudata.gyro_z.value = 6.0;
				raw_imudata.mag_x.value = 7.0;
				raw_imudata.mag_y.value = 8.0;
				raw_imudata.mag_z.value = 9.0;
				eros::imu processed_imudata;
				eros::signal processed_imudata_temperature;
				EXPECT_TRUE(process->new_imumsg(imus.at(i).devicename,raw_imudata,processed_imudata,processed_imudata_temperature).Level > NOTICE);
				message_sent = true;
				
			}

		}
		if(mediumrate_fire == true)
		{
		}
		if(slowrate_fire == true)
		{
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT + (2*process->get_imus().size())));
				for (std::size_t i = 0; i < diagnostics.size(); ++i)
				{
					if(diagnostics.at(i).Diagnostic_Type == SENSORS) 
					{
						std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
						EXPECT_TRUE(imus.size() > 0);
						for(std::size_t j = 0; j < imus.size(); ++j)
						{
							if(imus.at(j).diagnostic.DeviceName == diagnostics.at(i).DeviceName)
							{
								if(message_sent == true)
								{
									EXPECT_TRUE(diagnostics.at(i).Level > NOTICE);
								}
							}
						}
					}
					else
					{
					
										
						EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
					}
				}

			}
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}

TEST(Template,Process_Command)
{
	return;
	uint64_t id = 0;
	IMUNodeProcess* process = initializeprocess("IMU1","110012",&id,"/home/robot/catkin_ws/src/icarus_rover_v2/src/Sensor/unit_tests/IMU1.xml");
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	while(current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt,current_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
		int current_time_ms = (int)(current_time*1000.0);
		if((current_time_ms % 100) == 0)
		{
			fastrate_fire = true;
		}
		else { fastrate_fire = false; }
		if((current_time_ms % 1000) == 0)
		{
			mediumrate_fire = true;
		}
		else { mediumrate_fire = false; }
		if((current_time_ms % 10000) == 0)
		{
			slowrate_fire = true;
		}
		else { slowrate_fire = false; }

		eros::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;

		if(fastrate_fire == true) //Nothing to do here
		{
			cmd.Option1 = LEVEL1;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);
			if(process->get_imus_running() == false)
			{
				std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
				for(std::size_t i = 0; i < imus.size(); ++i)
				{
					EXPECT_TRUE(process->set_imu_running(imus.at(i).devicename));
					IMUNodeProcess::IMU imu1 = process->get_imu("IMU1");
					//Check Acc Rotation Matrix
					{
						Eigen::Matrix3f R = imu1.rotate_matrix.Rotation_Acc;
						EXPECT_TRUE(fabs(R(0,0)-1.0)<.00001);
						EXPECT_TRUE(fabs(R(0,1)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(0,2)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(1,0)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(1,1)-1.0)<.00001);
						EXPECT_TRUE(fabs(R(1,2)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(2,0)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(2,1)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(2,2)-1.0)<.00001);
					}

					//Check Gyro Rotation Matrix
					{
						Eigen::Matrix3f R = imu1.rotate_matrix.Rotation_Gyro;
						EXPECT_TRUE(fabs(R(0,0)-1.0)<.00001);
						EXPECT_TRUE(fabs(R(0,1)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(0,2)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(1,0)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(1,1)-1.0)<.00001);
						EXPECT_TRUE(fabs(R(1,2)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(2,0)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(2,1)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(2,2)-1.0)<.00001);
					}

					//Check Mag Rotation Matrix
					{
						Eigen::Matrix3f R = imu1.rotate_matrix.Rotation_Mag;
						EXPECT_TRUE(fabs(R(0,0)-1.0)<.00001);
						EXPECT_TRUE(fabs(R(0,1)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(0,2)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(1,0)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(1,1)-1.0)<.00001);
						EXPECT_TRUE(fabs(R(1,2)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(2,0)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(2,1)-0.0)<.00001);
						EXPECT_TRUE(fabs(R(2,2)-1.0)<.00001);
					}
				}
			}
			EXPECT_TRUE(process->get_imus_running());
			std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				IMUDriver::RawIMU raw_imudata;
				raw_imudata.serial_number = id;
				raw_imudata.signal_state = SIGNALSTATE_UPDATED;
				raw_imudata.tov = current_time;
				raw_imudata.acc_x.value = 1.0;
				raw_imudata.acc_y.value = 2.0;
				raw_imudata.acc_z.value = 3.0;
				raw_imudata.gyro_x.value = 4.0;
				raw_imudata.gyro_y.value = 5.0;
				raw_imudata.gyro_z.value = 6.0;
				raw_imudata.mag_x.value = 7.0;
				raw_imudata.mag_y.value = 8.0;
				raw_imudata.mag_z.value = 9.0;
				eros::imu processed_imudata;
				eros::signal processed_imudata_temperature;
				diag = process->new_imumsg(imus.at(i).devicename,raw_imudata,processed_imudata,processed_imudata_temperature);
				EXPECT_TRUE(diag.Level <= NOTICE);
			}


		}
		if(mediumrate_fire == true)
		{
			cmd.Option1 = LEVEL2;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);

		}
		if(slowrate_fire == true)
		{
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT + (2*process->get_imus().size())));
				for (std::size_t i = 0; i < diagnostics.size(); ++i)
				{
					EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
				}
			}

		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}

TEST(Template,Rotation_Computation)
{
	return;
	{
		uint64_t id = 0;
		IMUNodeProcess* process = initializeprocess("IMU1","110015",&id,"/home/robot/catkin_ws/src/icarus_rover_v2/src/Sensor/unit_tests/IMU1.xml");
		IMUNodeProcess::IMU imu1 = process->get_imu("IMU1");
		std::string filepath = "/home/robot/catkin_ws/src/icarus_rover_v2/src/Sensor/unit_tests/MountingAngleOffset_UnitTest.csv";
		std::ifstream file(filepath.c_str());
 
		std::vector<std::vector<std::string> > dataList;
	
		std::string line = "";
		// Iterate through each line and split the content using delimeter
		bool first_line = true;
		int counter = 0;
		//printf("start\n");
		while (getline(file, line))
		{
			if(first_line == false)
			{
				//printf("line: %s\n",line.c_str());
				std::vector<std::string> elements;
				boost::split(elements,line,boost::is_any_of(","));
				double x_acc = std::atof(elements.at(0).c_str());
				double y_acc = std::atof(elements.at(1).c_str());
				double z_acc = std::atof(elements.at(2).c_str());
				double mao_roll_deg = std::atof(elements.at(5).c_str());
				double mao_pitch_deg = std::atof(elements.at(7).c_str());
				EXPECT_TRUE(process->set_imu_mounting_angles(imu1.devicename,mao_roll_deg,mao_pitch_deg,0.0));
				//print_3x3matrix("ACC-X",process->get_imu(imu1.devicename).rotate_matrix.Rotation_Acc_X);
				//print_3x3matrix("ACC-Y",process->get_imu(imu1.devicename).rotate_matrix.Rotation_Acc_Y);
				//print_3x3matrix("ACC-Z",process->get_imu(imu1.devicename).rotate_matrix.Rotation_Acc_Z);
				//print_3x3matrix("ACC",process->get_imu(imu1.devicename).rotate_matrix.Rotation_Acc);				
				IMUDriver::RawIMU raw_imudata;
				raw_imudata.serial_number = id;
				raw_imudata.signal_state = SIGNALSTATE_UPDATED;
				raw_imudata.tov = (double)counter;
				raw_imudata.acc_x.value = x_acc;
				raw_imudata.acc_y.value = y_acc;
				raw_imudata.acc_z.value = z_acc;
				raw_imudata.gyro_x.value = 0.0;
				raw_imudata.gyro_y.value = 0.0;
				raw_imudata.gyro_z.value = 0.0;
				raw_imudata.mag_x.value = 0.0;
				raw_imudata.mag_y.value = 0.0;
				raw_imudata.mag_z.value = 0.0;
				eros::imu processed_imudata;
				eros::signal processed_imudata_temperature;
				EXPECT_TRUE(process->new_imumsg(imu1.devicename,raw_imudata,processed_imudata,processed_imudata_temperature).Level <= NOTICE);
				EXPECT_TRUE(fabs(processed_imudata.xacc.value) < .00001);
				EXPECT_TRUE(fabs(processed_imudata.yacc.value) < .00001);
				EXPECT_TRUE(fabs(processed_imudata.zacc.value-GRAVITATIONAL_ACCELERATION) < .00001);


			}
			first_line = false;
			counter++;
			
		}
		// Close the File
		file.close();
	}
}

TEST(Template,RMS_Computation)
{
	return;
	std::map<long,double> rms_checkpoints;
	//Read these values manually from included spreadsheet
	rms_checkpoints.insert(std::make_pair(0,0.0));
	rms_checkpoints.insert(std::make_pair(1,0.707106781));
	rms_checkpoints.insert(std::make_pair(2,1.29099444873581));
	rms_checkpoints.insert(std::make_pair(132,5.89379691754502)); 
	rms_checkpoints.insert(std::make_pair(2000,5.913333767));


	//Test Identity Matrix
	uint64_t id = 0;
	IMUNodeProcess* process = initializeprocess("IMU1","110012",&id,"/home/robot/catkin_ws/src/icarus_rover_v2/src/Sensor/unit_tests/IMU1.xml");
	IMUNodeProcess::IMU imu1 = process->get_imu("IMU1");


	double time_to_run = 60.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	int counter = 0;
	while(current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt,current_time);

		if(process->get_imus_running() == false)
		{
			std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				EXPECT_TRUE(process->set_imu_running(imus.at(i).devicename));
			}
		}
		EXPECT_TRUE(process->get_imus_running());
		std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
		for(std::size_t i = 0; i < imus.size(); ++i)
		{
			IMUDriver::RawIMU raw_imudata;
			raw_imudata.serial_number = id;
			raw_imudata.signal_state = SIGNALSTATE_UPDATED;
			raw_imudata.tov = current_time;
			raw_imudata.acc_x.value = (double)(counter % 11)*imu1.acc_scale_factor; //sawtooth
			raw_imudata.acc_y.value = (double)(counter % 11)*imu1.acc_scale_factor;
			raw_imudata.acc_z.value = (double)(counter % 11)*imu1.acc_scale_factor;
			raw_imudata.gyro_x.value = (double)(counter % 11)*imu1.gyro_scale_factor;
			raw_imudata.gyro_y.value = (double)(counter % 11)*imu1.gyro_scale_factor;
			raw_imudata.gyro_z.value = (double)(counter % 11)*imu1.gyro_scale_factor;
			raw_imudata.mag_x.value = (double)(counter % 11)*imu1.mag_scale_factor;
			raw_imudata.mag_y.value = (double)(counter % 11)*imu1.mag_scale_factor;
			raw_imudata.mag_z.value = (double)(counter % 11)*imu1.mag_scale_factor;
			eros::imu processed_imudata;
			eros::signal processed_imudata_temperature;
			EXPECT_TRUE(process->new_imumsg(imus.at(i).devicename,raw_imudata,processed_imudata,processed_imudata_temperature).Level <= NOTICE);
			std::map<long,double>::iterator front = rms_checkpoints.begin();
			if(front->first == counter)
			{
				EXPECT_TRUE(fabs(processed_imudata.xacc.rms-(front->second)) < .00001);
				rms_checkpoints.erase(front);
				if(rms_checkpoints.size() == 0)
				{
					//No more to check.
					current_time = 2.0*time_to_run;
				}
			}
		}

		current_time += dt;
		counter++;
	}
}
TEST(Template,IMUReset_Auto)
{
	return;
	uint64_t id = 0;
	IMUNodeProcess* process = initializeprocess("IMU1","110012",&id,"/home/robot/catkin_ws/src/icarus_rover_v2/src/Sensor/unit_tests/IMU1.xml");
	IMUNodeProcess::IMU imu1 = process->get_imu("IMU1");
	double time_to_run = 100.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	bool send_imu_msg = true;
	bool sent_imu_msg = false;
	bool reset_test_ok = false;
	int counter = 0;
	while(current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt,current_time);
		if(sent_imu_msg == true)
		{
			EXPECT_TRUE(diag.Level <= NOTICE);
		}
		int current_time_ms = (int)(current_time*1000.0);
		if((current_time_ms % 100) == 0)
		{
			fastrate_fire = true;
		}
		else { fastrate_fire = false; }
		if((current_time_ms % 1000) == 0)
		{
			mediumrate_fire = true;
		}
		else { mediumrate_fire = false; }
		if((current_time_ms % 10000) == 0)
		{
			slowrate_fire = true;
		}
		else { slowrate_fire = false; }

		if(fastrate_fire == true) //Nothing to do here
		{
			if(process->get_imus_running() == false)
			{
				std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
				for(std::size_t i = 0; i < imus.size(); ++i)
				{
					EXPECT_TRUE(process->set_imu_running(imus.at(i).devicename));
				}
			}
			if(send_imu_msg == true)
			{
				EXPECT_TRUE(process->get_imus_running());
				std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
				for(std::size_t i = 0; i < imus.size(); ++i)
				{
					sent_imu_msg = true;
					IMUDriver::RawIMU raw_imudata;
					raw_imudata.serial_number = id;
					raw_imudata.signal_state = SIGNALSTATE_UPDATED;
					raw_imudata.tov = current_time;
					raw_imudata.acc_x.value = (double)(counter % 11)*imu1.acc_scale_factor; //sawtooth
					raw_imudata.acc_y.value = (double)(counter % 11)*imu1.acc_scale_factor;
					raw_imudata.acc_z.value = (double)(counter % 11)*imu1.acc_scale_factor;
					raw_imudata.gyro_x.value = (double)(counter % 11)*imu1.gyro_scale_factor;
					raw_imudata.gyro_y.value = (double)(counter % 11)*imu1.gyro_scale_factor;
					raw_imudata.gyro_z.value = (double)(counter % 11)*imu1.gyro_scale_factor;
					raw_imudata.mag_x.value = (double)(counter % 11)*imu1.mag_scale_factor;
					raw_imudata.mag_y.value = (double)(counter % 11)*imu1.mag_scale_factor;
					raw_imudata.mag_z.value = (double)(counter % 11)*imu1.mag_scale_factor;
					eros::imu processed_imudata;
					eros::signal processed_imudata_temperature;
					EXPECT_TRUE(process->new_imumsg(imus.at(i).devicename,raw_imudata,processed_imudata,processed_imudata_temperature).Level <= NOTICE);
				}
			}
			else
			{
				sent_imu_msg = false;
			}
		}
		if(mediumrate_fire == true)
		{
			std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				if((imus.at(i).diagnostic.Diagnostic_Message == DROPPING_PACKETS) and 
				  (imus.at(i).diagnostic.Level == ERROR))
				  {
					  printf("%f ",current_time);
					  print_diagnostic(WARN,imus.at(i).diagnostic);
				  }
			}
			bool reset = process->get_imureset_trigger();
			if(reset== true)
			{
				EXPECT_TRUE(process->get_imureset_trigger() == false);
				reset_test_ok = true;
				send_imu_msg = true;
			}
		}
		if(slowrate_fire == true)
		{
			if((current_time >= 20.0) and (reset_test_ok == false))
			{
				send_imu_msg = false;
			}
		}
		counter++;
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
TEST(Template,IMUReset_Manual)
{
	return;
	uint64_t id = 0;
	IMUNodeProcess* process = initializeprocess("IMU1","110012",&id,"/home/robot/catkin_ws/src/icarus_rover_v2/src/Sensor/unit_tests/IMU1.xml");
	IMUNodeProcess::IMU imu1 = process->get_imu("IMU1");
	double time_to_run = 100.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	bool send_imu_msg = true;
	bool sent_imu_msg = false;
	bool reset_test_ok = false;
	int counter = 0;
	while(current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt,current_time);
		if(sent_imu_msg == true)
		{
			EXPECT_TRUE(diag.Level <= NOTICE);
		}
		int current_time_ms = (int)(current_time*1000.0);
		if((current_time_ms % 100) == 0)
		{
			fastrate_fire = true;
		}
		else { fastrate_fire = false; }
		if((current_time_ms % 1000) == 0)
		{
			mediumrate_fire = true;
		}
		else { mediumrate_fire = false; }
		if((current_time_ms % 10000) == 0)
		{
			slowrate_fire = true;
		}
		else { slowrate_fire = false; }

		if(fastrate_fire == true) //Nothing to do here
		{
			if(process->get_imus_running() == false)
			{
				std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
				for(std::size_t i = 0; i < imus.size(); ++i)
				{
					EXPECT_TRUE(process->set_imu_running(imus.at(i).devicename));
				}
			}
			if(send_imu_msg == true)
			{
				EXPECT_TRUE(process->get_imus_running());
				std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
				for(std::size_t i = 0; i < imus.size(); ++i)
				{
					sent_imu_msg = true;
					IMUDriver::RawIMU raw_imudata;
					raw_imudata.serial_number = id;
					raw_imudata.signal_state = SIGNALSTATE_UPDATED;
					raw_imudata.tov = current_time;
					raw_imudata.acc_x.value = (double)(counter % 11)*imu1.acc_scale_factor; //sawtooth
					raw_imudata.acc_y.value = (double)(counter % 11)*imu1.acc_scale_factor;
					raw_imudata.acc_z.value = (double)(counter % 11)*imu1.acc_scale_factor;
					raw_imudata.gyro_x.value = (double)(counter % 11)*imu1.gyro_scale_factor;
					raw_imudata.gyro_y.value = (double)(counter % 11)*imu1.gyro_scale_factor;
					raw_imudata.gyro_z.value = (double)(counter % 11)*imu1.gyro_scale_factor;
					raw_imudata.mag_x.value = (double)(counter % 11)*imu1.mag_scale_factor;
					raw_imudata.mag_y.value = (double)(counter % 11)*imu1.mag_scale_factor;
					raw_imudata.mag_z.value = (double)(counter % 11)*imu1.mag_scale_factor;
					eros::imu processed_imudata;
					eros::signal processed_imudata_temperature;
					EXPECT_TRUE(process->new_imumsg(imus.at(i).devicename,raw_imudata,processed_imudata,processed_imudata_temperature).Level <= NOTICE);
				}
			}
			else
			{
				sent_imu_msg = false;
			}
		}
		if(mediumrate_fire == true)
		{
			std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				if((imus.at(i).diagnostic.Diagnostic_Message == DROPPING_PACKETS) and 
				  (imus.at(i).diagnostic.Level == ERROR))
				  {
					  printf("%f ",current_time);
					  print_diagnostic(WARN,imus.at(i).diagnostic);
				  }
			}
			bool reset = process->get_imureset_trigger();
			if(reset== true)
			{
				EXPECT_TRUE(process->get_imureset_trigger() == false);
				reset_test_ok = true;
				send_imu_msg = true;
			}
		}
		if(slowrate_fire == true)
		{
			if((current_time >= 20.0) and (reset_test_ok == false))
			{
				eros::command cmd;
				cmd.Command = ROVERCOMMAND_RESET;
				cmd.Option1 = SENSORS;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for(std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				bool reset = process->get_imureset_trigger();
				if(reset== true)
				{
					EXPECT_TRUE(process->get_imureset_trigger() == false);
					reset_test_ok = true;
				}
			}
		}
		counter++;
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

