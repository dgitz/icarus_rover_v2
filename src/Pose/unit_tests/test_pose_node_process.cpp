#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../PoseNodeProcess.h"
#define DIAGNOSTIC_TYPE_COUNT 5

std::string Node_Name = "/unittest_pose_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;

void print_diagnostic(uint8_t level,eros::diagnostic diagnostic)
{
	DiagnosticClass diag_helper;
	if(diagnostic.Level >= level)
	{
		printf("Type: %s Message: %s Level: %s Device: %s Desc: %s\n",
			diag_helper.get_DiagTypeString(diagnostic.Diagnostic_Type).c_str(),
			diag_helper.get_DiagMessageString(diagnostic.Diagnostic_Message).c_str(),
			diag_helper.get_DiagLevelString(diagnostic.Level).c_str(),
			diagnostic.DeviceName.c_str(),diagnostic.Description.c_str());
	}
}
eros::signal generate_signal(std::string name,double value)
{
	eros::signal sig;
	sig.name = name;
	sig.value = value;
	return sig;
}
eros::imu generate_imumsg(uint16_t id)
{
	eros::imu msg;
	msg.xacc = generate_signal("xacc" + std::to_string(id),10.0);
	msg.yacc = generate_signal("yacc" + std::to_string(id),1.0);
	msg.zacc = generate_signal("zacc" + std::to_string(id),2.0);
	msg.xgyro = generate_signal("xgyro" + std::to_string(id),3.0);
	msg.ygyro = generate_signal("ygyro" + std::to_string(id),4.0);
	msg.zgyro = generate_signal("zgyro" + std::to_string(id),5.0);
	msg.xmag = generate_signal("xmag" + std::to_string(id),6.0);
	msg.ymag = generate_signal("ymag" + std::to_string(id),7.0);
	msg.zmag = generate_signal("zmag" + std::to_string(id),8.0);
	return msg;
}
PoseNodeProcess* initializeprocess(uint8_t imucount)
{
	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";
	std::vector<eros::device> imu_list;
	for(int i = 0; i < imucount; ++i)
	{
		eros::device imu;
		imu.DeviceName = "IMU" + std::to_string(i+1);
		imu.DeviceParent = ros_DeviceName;
		imu.DeviceType = "IMU";
		imu.ID = i;
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
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZING);
	process->set_mydevice(device);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZED);
	EXPECT_TRUE(process->set_imucount(imucount));
	for(std::size_t i = 0; i < imu_list.size(); ++i)
	{
		eros::device::ConstPtr imu_ptr(new eros::device(imu_list.at(i)));
		eros::diagnostic diagnostic = process->new_devicemsg(imu_ptr);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	process->update_diagnostic(SYSTEM_RESOURCE,INFO,NOERROR,"Not testing this during unit tests.");
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
PoseNodeProcess* readyprocess(PoseNodeProcess* process)
{
	std::vector<eros::diagnostic> diagnostics;
	eros::diagnostic diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Diagnostic_Type == POSE);
	EXPECT_TRUE(diag.Level == WARN); //NO Sensor Inputs Yet
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
	{
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT+process->get_imus().size()));
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			if(diagnostics.at(i).Diagnostic_Type == POSE) //NO Sensor Inputs Yet
			{
				EXPECT_TRUE(diagnostics.at(i).Level <= WARN);
			}
			else
			{
				EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
			}
			
			
		}
	}
	return process;
}
TEST(Template,Process_Initialization)
{
	PoseNodeProcess* process = initializeprocess(1);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZED);
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
TEST(Reference,HeadingReference)
{
	PoseHelper pose_helper;
	pose_helper.print_simple_headingstring_map();
	double angle = 0.0;
	//Check Simple Heading String Map Constructino
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "W");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,-180.0,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "WSW");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,-157.5,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "SW");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,-135.0,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "SSW");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,-112.5,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "S");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,-90.0,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "SSE");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,-67.5,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "SE");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,-45.0,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "ESE");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,-22.5,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "E");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,0.0,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "ENE");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,22.5,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "NE");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,45.0,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "NNE");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,67.5,
		0.0001));
		angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "N");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,90.0,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "NNW");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,112.5,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "NW");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,135.0,
		0.0001));
	angle = pose_helper.get_examplarheading_simplestring(PoseHelper::HeadingReference::COMMON_YAW, "WNW");
	EXPECT_TRUE(pose_helper.is_angleequal(
		PoseHelper::HeadingReference::COMMON_YAW,angle,
		PoseHelper::HeadingReference::COMMON_YAW,157.5,
		0.0001));
	
	// Spot check a few arbitrary angles
	EXPECT_TRUE("E"==pose_helper.compute_heading_simplestring(PoseHelper::HeadingReference::COMMON_YAW,5.0));
	EXPECT_TRUE("E"==pose_helper.compute_heading_simplestring(PoseHelper::HeadingReference::COMMON_YAW,-5.0));
	EXPECT_TRUE("N"==pose_helper.compute_heading_simplestring(PoseHelper::HeadingReference::COMMON_YAW,88.0));
	EXPECT_TRUE("NW"==pose_helper.compute_heading_simplestring(PoseHelper::HeadingReference::COMMON_YAW,136.0));
	EXPECT_TRUE("W"==pose_helper.compute_heading_simplestring(PoseHelper::HeadingReference::COMMON_YAW,181.0));
	EXPECT_TRUE("W"==pose_helper.compute_heading_simplestring(PoseHelper::HeadingReference::COMMON_YAW,179.0));
	EXPECT_TRUE("W"==pose_helper.compute_heading_simplestring(PoseHelper::HeadingReference::COMMON_YAW,-181.0));
	EXPECT_TRUE("W"==pose_helper.compute_heading_simplestring(PoseHelper::HeadingReference::COMMON_YAW,-179.0));
}
TEST(Template,Process_SensorInputs)
{
	PoseNodeProcess* process = initializeprocess(2);
	process = readyprocess(process);
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	bool allsensor_data_received = false;
	bool pause_resume_ran = false;
	while(current_time <= time_to_run)
	{
		if(current_time > 5.0)
		{
			EXPECT_TRUE(allsensor_data_received == true);
		}
		eros::diagnostic diag = process->update(dt,current_time);
		if(allsensor_data_received == true)
		{
			EXPECT_TRUE(diag.Level <= NOTICE);
		}
		else
		{
			EXPECT_TRUE(diag.Level == WARN);
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


		if(fastrate_fire == true)
		{
			{
				eros::imu imu_msg = generate_imumsg(1);
				eros::imu::ConstPtr imudata_ptr(new eros::imu(imu_msg));
				diag = process->new_imumsg("/IMU1", imudata_ptr);
				EXPECT_TRUE(diag.Level <= NOTICE);
			}
			{
				eros::imu imu_msg = generate_imumsg(2);
				eros::imu::ConstPtr imudata_ptr(new eros::imu(imu_msg));
				diag = process->new_imumsg("/IMU2", imudata_ptr);
				EXPECT_TRUE(diag.Level <= NOTICE);
			}
			allsensor_data_received = true;
			//PoseNodeProcess::IMUSensor imu = process->get_imudata("IMU1");
			//EXPECT_TRUE(imu.orientation_pitch.status == SIGNALSTATE_UPDATED);
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
					if(diagnostics.at(i).Diagnostic_Type == POSE) //NO Sensor Inputs Yet
					{
						if(allsensor_data_received == true)
						{
							EXPECT_TRUE(diag.Level <= NOTICE);
						}
						else
						{
							EXPECT_TRUE(diag.Level == WARN);
						}
					}
					else
					{
						EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
					}
				}
			}
			if(pause_resume_ran == false)
			{
				{
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
					eros::command cmd_taskcontrol;
					cmd_taskcontrol.Command = ROVERCOMMAND_TASKCONTROL;
					cmd_taskcontrol.Option1 = SUBSYSTEM_UNKNOWN;
					cmd_taskcontrol.Option2 = TASKSTATE_PAUSE;
					cmd_taskcontrol.CommandText = Node_Name;
					eros::command::ConstPtr cmd_ptr(new eros::command(cmd_taskcontrol));
					std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
					for (std::size_t i = 0; i < diaglist.size(); i++)
					{
						EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					}
					EXPECT_TRUE(diaglist.size() > 0);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_PAUSE);
				}
				{
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_PAUSE);
					eros::command cmd_taskcontrol;
					cmd_taskcontrol.Command = ROVERCOMMAND_TASKCONTROL;
					cmd_taskcontrol.Option1 = SUBSYSTEM_UNKNOWN;
					cmd_taskcontrol.Option2 = TASKSTATE_RUNNING;
					cmd_taskcontrol.CommandText = Node_Name;
					eros::command::ConstPtr cmd_ptr(new eros::command(cmd_taskcontrol));

					std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
					for (std::size_t i = 0; i < diaglist.size(); i++)
					{
						EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					}
					EXPECT_TRUE(diaglist.size() > 0);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
				}
				{
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
					eros::command cmd_taskcontrol;
					cmd_taskcontrol.Command = ROVERCOMMAND_TASKCONTROL;
					cmd_taskcontrol.Option1 = SUBSYSTEM_UNKNOWN;
					cmd_taskcontrol.Option2 = TASKSTATE_RESET;
					cmd_taskcontrol.CommandText = Node_Name;
					eros::command::ConstPtr cmd_ptr(new eros::command(cmd_taskcontrol));

					std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
					for (std::size_t i = 0; i < diaglist.size(); i++)
					{
						EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					}
					EXPECT_TRUE(diaglist.size() > 0);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RESET);
					diag = process->update(0.0, 0.0);
					EXPECT_TRUE(diag.Level <= NOTICE);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
				}
				pause_resume_ran = true;
			}
		}
		current_time += dt;
	}
	EXPECT_TRUE(pause_resume_ran == true);
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
	EXPECT_TRUE(process->get_poseupdate_counter() > 0);
	//Check Pose Outputs
	{
		PoseNodeProcess::TimedSignals timed_signals = process->get_timedsignals();
		EXPECT_TRUE(fabs(timed_signals.accel1x.value) > 0.0);
		EXPECT_TRUE(fabs(timed_signals.accel1y.value) > 0.0);
		EXPECT_TRUE(fabs(timed_signals.accel1z.value) > 0.0);

	}
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

