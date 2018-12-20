#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../IMUNodeProcess.h"

std::string Node_Name = "/unittest_imu_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;


IMUNodeProcess* initializeprocess()
{
	icarus_rover_v2::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = COMMUNICATION_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";

	icarus_rover_v2::device device;
	device.DeviceName = diagnostic.DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 1;
	device.DeviceParent = "None";
	device.Architecture = "armv7l";

	icarus_rover_v2::device imu;
	imu.DeviceName = "IMULeft";
	imu.DeviceParent = ros_DeviceName;
	imu.DeviceType = "IMU";
	imu.ID = 0;
	imu.PartNumber = "110012";


	IMUNodeProcess *process;
	process = new IMUNodeProcess;
	process->initialize("imu_node",Node_Name,Host_Name);
	process->set_diagnostic(diagnostic);
	process->finish_initialization();
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	EXPECT_TRUE(process->is_ready() == false);
	icarus_rover_v2::device::ConstPtr imu_ptr(new icarus_rover_v2::device(imu));
	diagnostic = process->new_devicemsg(imu_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);
	EXPECT_TRUE(process->get_imus_initialized() == true);
	EXPECT_TRUE(process->get_imus_running() == false);
	return process;
}
TEST(Template,Process_Initialization)
{
	IMUNodeProcess* process = initializeprocess();
}

TEST(Template,Process_Msg)
{
	IMUNodeProcess* process = initializeprocess();
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	while(current_time <= time_to_run)
	{
		icarus_rover_v2::diagnostic diag = process->update(dt,current_time);
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
				raw_imudata.acc_x = 1.0;
				raw_imudata.acc_y = 2.0;
				raw_imudata.acc_z = 3.0;
				raw_imudata.gyro_x = 4.0;
				raw_imudata.gyro_y = 5.0;
				raw_imudata.gyro_z = 6.0;
				raw_imudata.mag_x = 7.0;
				raw_imudata.mag_y = 8.0;
				raw_imudata.mag_z = 9.0;
				icarus_rover_v2::imu processed_imudata;
				EXPECT_TRUE(process->new_imumsg(imus.at(i).devicename,raw_imudata,processed_imudata).Level <= NOTICE);
				EXPECT_TRUE((fabs((raw_imudata.acc_x/imus.at(i).acc_scale_factor)-processed_imudata.xacc.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.acc_y/imus.at(i).acc_scale_factor)-processed_imudata.yacc.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.acc_z/imus.at(i).acc_scale_factor)-processed_imudata.zacc.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.gyro_x/imus.at(i).gyro_scale_factor)-processed_imudata.xgyro.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.gyro_y/imus.at(i).gyro_scale_factor)-processed_imudata.ygyro.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.gyro_z/imus.at(i).gyro_scale_factor)-processed_imudata.zgyro.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.mag_x/imus.at(i).mag_scale_factor)-processed_imudata.xmag.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.mag_y/imus.at(i).mag_scale_factor)-processed_imudata.ymag.value) < .0001));
				EXPECT_TRUE((fabs((raw_imudata.mag_z/imus.at(i).mag_scale_factor)-processed_imudata.zmag.value) < .0001));
			}

		}
		if(mediumrate_fire == true)
		{
		}
		if(slowrate_fire == true)
		{
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
TEST(Template,Process_Command)
{
	IMUNodeProcess* process = initializeprocess();
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	while(current_time <= time_to_run)
	{
		icarus_rover_v2::diagnostic diag = process->update(dt,current_time);
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

		icarus_rover_v2::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;

		if(fastrate_fire == true) //Nothing to do here
		{
			cmd.Option1 = LEVEL1;
			icarus_rover_v2::command::ConstPtr cmd_ptr(new icarus_rover_v2::command(cmd));
			std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);


		}
		if(mediumrate_fire == true)
		{
			cmd.Option1 = LEVEL2;
			icarus_rover_v2::command::ConstPtr cmd_ptr(new icarus_rover_v2::command(cmd));
			std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);

		}
		if(slowrate_fire == true)
		{
			//Don't run LEVEL3 Test, as this will be called circularly and is only responsible for running this test anyways.
			/*
            cmd.Option1 = LEVEL3;
            std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
            for(std::size_t i = 0; i < diaglist.size(); i++)
            {
                EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
            }
             EXPECT_TRUE(diaglist.size() > 0);
			 */
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

