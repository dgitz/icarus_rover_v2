#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../PoseNodeProcess.h"
#define DIAGNOSTIC_TYPE_COUNT 5

std::string Node_Name = "/unittest_pose_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;


PoseNodeProcess* initializeprocess(uint8_t imucount)
{
	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";
	std::vector<eros::device> imu_list;
	if(imucount == 1)
	{
		eros::device imu;
		imu.DeviceName = "IMU1";;
		imu.DeviceParent = ros_DeviceName;
		imu.DeviceType = "IMU";
		imu.ID = 0;
		imu.PartNumber = "110012";
		imu_list.push_back(imu);
	}

	PoseNodeProcess *process;
	process = new PoseNodeProcess;
	process->initialize("pose_node",Node_Name,Host_Name,ROVER,ROBOT_CONTROLLER,POSE_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(SENSORS);
	diagnostic_types.push_back(POSE);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->set_imucount(imucount));
	for(std::size_t i = 0; i < imu_list.size(); ++i)
	{
		eros::device::ConstPtr imu_ptr(new eros::device(imu_list.at(i)));
		eros::diagnostic diagnostic = process->new_devicemsg(imu_ptr);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
PoseNodeProcess* readyprocess(PoseNodeProcess* process)
{
	eros::diagnostic diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);
	{
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT+process->get_imus().size()));
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
		}
	}
	return process;
}
TEST(Template,Process_Initialization)
{
	initializeprocess(1);
}
TEST(Math,Definitions)
{
	//ATAN
	{
		std::vector<double> x{-1.0,-1.0,0.0,0.0,0.0,1.0,1.0};
		std::vector<double> y{-1.0,0.0,-1.0,0.0,1.0,0.0,1.0};
		std::vector<double> atan2_expectedvalue_rad{-M_PI*3.0/4.0,-M_PI/2.0,M_PI,0.0,0.0,M_PI/2.0,M_PI/4.0};
		for(std::size_t i = 0; i < x.size(); ++i)
		{
			double v = atan2(x.at(i),y.at(i));
			EXPECT_TRUE((fabs(v-atan2_expectedvalue_rad.at(i)) < .000001));
		}
	}
	//ASIN
	{
		std::vector<double> x{0.0,-1.0,-2.0,-3.0,1.0,2.0,3.0};
		std::vector<double> y{1.0,2.0,3.0,4.0,5.0,6.0,7.0};
		std::vector<double> asin_expectedvalue_rad{0.0,-0.463647609000806,-0.588002603547568,-0.643501108793284,0.197395559849881,0.321750554396642,0.404891786285083};
		for(std::size_t i = 0; i < x.size(); ++i)
		{
			double r = pow((x.at(i)*x.at(i)+y.at(i)*y.at(i)),0.5);
			double v = asin(x.at(i)/r);
			EXPECT_TRUE((fabs(v-asin_expectedvalue_rad.at(i)) < .000001));
		}
	}
}
TEST(Blocks,Blocks)
{
	PoseNodeProcess* process = initializeprocess(1);
	process = readyprocess(process);
	//Acceleration Based Orientation
	{
		{//Roll
			std::vector<double> x{0.0,-4.0,-9.0,4.0,9.0,0.0,0.0,0.0,0.0};
			std::vector<double> y{0.0,0.0,0.0,0.0,0.0,-4.0,-9.0,4.0,9.0};
			std::vector<double> z{9.81,8.9574605,3.90334472,8.9574605,3.90334472,8.9574605,3.90334472,8.9574605,3.90334472};
			std::vector<double> expected{0.0,0.0,0.0,0.0,0.0,0.4199855,1.161575,-0.4199855,-1.161575};
			for(std::size_t i = 0; i < x.size(); ++i)
			{
				double v = process->compute_acceleration_based_roll(x.at(i),y.at(i),z.at(i));
				EXPECT_TRUE((fabs(v-expected.at(i)) < .000001));
			}
		}
		{//Pitch
			std::vector<double> x{0.0,-4.0,-9.0,4.0,9.0,0.0,0.0,0.0,0.0};
			std::vector<double> y{0.0,0.0,0.0,0.0,0.0,-4.0,-9.0,4.0,9.0};
			std::vector<double> z{9.81,8.9574605,3.90334472,8.9574605,3.90334472,8.9574605,3.90334472,8.9574605,3.90334472};
			std::vector<double> expected{0.0,0.4199855,1.161575,-0.4199855,-1.161575,0.0,0.0,0.0,0.0};
			for(std::size_t i = 0; i < x.size(); ++i)
			{
				double v = process->compute_acceleration_based_pitch(x.at(i),y.at(i),z.at(i));
				EXPECT_TRUE((fabs(v-expected.at(i)) < .000001));
			}
		}
	}
}

TEST(Template,Process_Command)
{
	PoseNodeProcess* process = initializeprocess(1);
	process = readyprocess(process);
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
				EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT+process->get_imus().size()));
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
TEST(Template,Process_IMUMsg)
{
	PoseNodeProcess* process = initializeprocess(1);
	process = readyprocess(process);
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


		if(fastrate_fire == true)
		{
			eros::imu imu_msg;
			imu_msg.xacc.value = 0.0;
			imu_msg.yacc.value = 0.0;
			imu_msg.zacc.value = 9.81;
			eros::imu::ConstPtr imudata_ptr(new eros::imu(imu_msg));
			diag = process->new_imumsg("/IMU1", imudata_ptr);
			PoseNodeProcess::IMUSensor imu = process->get_imudata("IMU1");
			EXPECT_TRUE(imu.orientation_pitch.status == SIGNALSTATE_UPDATED);
		}
		if(mediumrate_fire == true)
		{

		}
		if(slowrate_fire == true)
		{
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT+process->get_imus().size()));
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
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

