#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../BoardControllerNodeProcess.h"

std::string Node_Name = "/unittest_boardcontroller_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
std::string ros_DeviceType = "ControlModule";
#define DIAGNOSTIC_TYPE_COUNT 5

BoardControllerNodeProcess *initialized_process;
void print_diagnostic(uint8_t level,eros::diagnostic diagnostic)
{
	if(diagnostic.Level >= level)
	{
		printf("Type: %d Message: %d Level: %d Device: %s Desc: %s\n",diagnostic.Diagnostic_Type,diagnostic.Diagnostic_Message,
			  		diagnostic.Level,diagnostic.DeviceName.c_str(),diagnostic.Description.c_str());
	}
}
bool check_if_initialized(BoardControllerNodeProcess process);
void print_sensordata(std::vector<BoardControllerNodeProcess::Sensor> sensors);
BoardControllerNodeProcess* initializeprocess()
{
	eros::diagnostic diagnostic;
	
	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	BoardControllerNodeProcess* process;
	process = new BoardControllerNodeProcess;
	process->initialize("unit_test", Node_Name, Host_Name, ROVER, ROBOT_CONTROLLER, CONTROLLER_NODE);
	
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	diagnostic_types.push_back(SENSORS);
	process->enable_diagnostics(diagnostic_types);

	process->finish_initialization();
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	return process;
}
BoardControllerNodeProcess* initializeprocess(eros::device device)
{
	eros::diagnostic diagnostic;
	

	BoardControllerNodeProcess* process;
	process = new BoardControllerNodeProcess;
	process->initialize("boardcontroller_node",Node_Name,Host_Name,ROVER, ROBOT_CONTROLLER, CONTROLLER_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	diagnostic_types.push_back(SENSORS);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
BoardControllerNodeProcess *readyprocess(BoardControllerNodeProcess *process)
{
	process->update_diagnostic(SYSTEM_RESOURCE,INFO,NOERROR,"Not testing this during unit tests.");
	eros::diagnostic diag = process->update(0.0, 0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	{
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT+(2*process->get_boarddata().size())+(process->get_sensordata().size()));
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
		}
	}
	EXPECT_TRUE(process->is_ready() == true);
	return process;
}
TEST(Template,Process_Initialization)
{
	initializeprocess();
}

TEST(DeviceInitialization,DeviceInitialization_ArduinoBoard)
{
	eros::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = "";
	ros_device.DeviceType = ros_DeviceType;
	ros_device.BoardCount = 1;
	ros_device.SensorCount = 0;
	ros_device.ID = 17;

	eros::device arduinoboard1_device;
	arduinoboard1_device.DeviceName = "ArduinoBoard1";
	arduinoboard1_device.DeviceParent = ros_DeviceName;
	arduinoboard1_device.DeviceType = "ArduinoBoard";
	arduinoboard1_device.BoardCount = 0;
	arduinoboard1_device.SensorCount = 4;
	arduinoboard1_device.ID = 0;

	{
		eros::pin newpin;
		newpin.ConnectedSensor = "LeftEncoder";
		newpin.Name = "LeftEncoder";
		newpin.Number = 0;
		newpin.Function = "QuadratureEncoderInput";
		arduinoboard1_device.pins.push_back(newpin);
	}

	{
		eros::pin newpin;
		newpin.ConnectedSensor = "RightEncoder";
		newpin.Name = "RightEncoder";
		newpin.Number = 1;
		newpin.Function = "QuadratureEncoderInput";
		arduinoboard1_device.pins.push_back(newpin);
	}

	{
		eros::pin newpin;
		newpin.ConnectedSensor = "BucketAngleSensor";
		newpin.Name = "AnalogInput0";
		newpin.Number = 0;
		newpin.Function = "AnalogInput";
		arduinoboard1_device.pins.push_back(newpin);
	}

	{
		eros::pin newpin;
		newpin.ConnectedSensor = "BucketLiftSensor";
		newpin.Name = "AnalogInput1";
		newpin.Number = 1;
		newpin.Function = "AnalogInput";
		arduinoboard1_device.pins.push_back(newpin);
	}


	BoardControllerNodeProcess *process = initializeprocess(ros_device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);

	eros::device::ConstPtr arduinoboard1_ptr(new eros::device(arduinoboard1_device));
	eros::diagnostic diagnostic = process->new_devicemsg(arduinoboard1_ptr);
	process = readyprocess(process);
	diagnostic = process->new_message_Diagnostic(arduinoboard1_device.ID,ROVER,ENTIRE_SUBSYSTEM,GPIO_NODE,SENSORS,WARN,DROPPING_PACKETS);
	EXPECT_TRUE(diagnostic.Level == WARN);
	diagnostic = process->update(0.1, 0.1);
	EXPECT_TRUE(process->get_ready_to_arm() == false);
	diagnostic = process->new_message_Diagnostic(arduinoboard1_device.ID,ROVER,ENTIRE_SUBSYSTEM,GPIO_NODE,SENSORS,NOTICE,INITIALIZING);
	diagnostic = process->update(0.1, 0.2);
	EXPECT_TRUE(process->get_ready_to_arm() == false);
	diagnostic = process->new_message_Diagnostic(arduinoboard1_device.ID,ROVER,ENTIRE_SUBSYSTEM,GPIO_NODE,SENSORS,NOTICE,NOERROR);
	diagnostic = process->update(0.1, 0.3);
	EXPECT_TRUE(process->get_ready_to_arm() == true);

	uint16_t v1 = 123;
	uint16_t v2 = 456;
	diagnostic = process->new_message_GetANAPort1(arduinoboard1_device.ID,0.121,v1,v2,0,0,0,0);

	int16_t a1 = 123;
	int16_t a2 = -456;
	diagnostic = process->new_message_GetDIOPort1(arduinoboard1_device.ID,1.234,a1,a2);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	std::vector<BoardControllerNodeProcess::Sensor> sensors = process->get_sensordata();
	print_sensordata(process->get_sensordata());



}
TEST(Template, Process_Command)
{
	eros::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = "";
	ros_device.DeviceType = ros_DeviceType;
	ros_device.BoardCount = 1;
	ros_device.SensorCount = 0;
	ros_device.ID = 17;

	eros::device arduinoboard1_device;
	arduinoboard1_device.DeviceName = "ArduinoBoard1";
	arduinoboard1_device.DeviceParent = ros_DeviceName;
	arduinoboard1_device.DeviceType = "ArduinoBoard";
	arduinoboard1_device.BoardCount = 0;
	arduinoboard1_device.SensorCount = 4;
	arduinoboard1_device.ID = 0;

	{
		eros::pin newpin;
		newpin.ConnectedSensor = "LeftEncoder";
		newpin.Name = "LeftEncoder";
		newpin.Number = 0;
		newpin.Function = "QuadratureEncoderInput";
		arduinoboard1_device.pins.push_back(newpin);
	}

	{
		eros::pin newpin;
		newpin.ConnectedSensor = "RightEncoder";
		newpin.Name = "RightEncoder";
		newpin.Number = 1;
		newpin.Function = "QuadratureEncoderInput";
		arduinoboard1_device.pins.push_back(newpin);
	}

	{
		eros::pin newpin;
		newpin.ConnectedSensor = "BucketAngleSensor";
		newpin.Name = "AnalogInput0";
		newpin.Number = 0;
		newpin.Function = "AnalogInput";
		arduinoboard1_device.pins.push_back(newpin);
	}

	{
		eros::pin newpin;
		newpin.ConnectedSensor = "BucketLiftSensor";
		newpin.Name = "AnalogInput1";
		newpin.Number = 1;
		newpin.Function = "AnalogInput";
		arduinoboard1_device.pins.push_back(newpin);
	}


	BoardControllerNodeProcess *process = initializeprocess(ros_device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);

	eros::device::ConstPtr arduinoboard1_ptr(new eros::device(arduinoboard1_device));
	eros::diagnostic diagnostic = process->new_devicemsg(arduinoboard1_ptr);
	process = readyprocess(process);
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false;   //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false;   //0.1 Hz
	bool board_timeout_check_passed = false;
	while (current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt, current_time);
		if((board_timeout_check_passed == true))
		{
			EXPECT_TRUE(diag.Level <= NOTICE);
		}
		

		int current_time_ms = (int)(current_time * 1000.0);
		if ((current_time_ms % 100) == 0)
		{
			fastrate_fire = true;
		}
		else
		{
			fastrate_fire = false;
		}
		if ((current_time_ms % 1000) == 0)
		{
			mediumrate_fire = true;
		}
		else
		{
			mediumrate_fire = false;
		}
		if ((current_time_ms % 10000) == 0)
		{
			slowrate_fire = true;
		}
		else
		{
			slowrate_fire = false;
		}

		eros::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;

		if (fastrate_fire == true) //Nothing to do here
		{
			cmd.Option1 = LEVEL1;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for (std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);
		}
		if (mediumrate_fire == true)
		{
			cmd.Option1 = LEVEL2;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for (std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);
			if(board_timeout_check_passed == true)
			{
				diagnostic = process->new_message_Diagnostic(arduinoboard1_device.ID,ROVER,ENTIRE_SUBSYSTEM,GPIO_NODE,SENSORS,NOTICE,NOERROR);
				EXPECT_TRUE(diagnostic.Level <= NOTICE);
			}
			std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
			EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT+(2*process->get_boarddata().size())+(process->get_sensordata().size()));
			for (std::size_t i = 0; i < diagnostics.size(); ++i)
			{
				print_diagnostic(WARN,diagnostics.at(i));
				if((diagnostics.at(i).Diagnostic_Type == COMMUNICATIONS) and (diagnostics.at(i).DeviceName != Host_Name)) 
				{
					if((board_timeout_check_passed == false))
					{
						if((current_time > process->get_boardcomm_timeout_warn_threshold()) and
						   (current_time < process->get_boardcomm_timeout_error_threshold()))
						   {
							   EXPECT_TRUE(diagnostics.at(i).Level == WARN);
						   }
						   else if(current_time >= process->get_boardcomm_timeout_error_threshold())
						   {
							    EXPECT_TRUE(diagnostics.at(i).Level == ERROR);
								process->new_message_Diagnostic(arduinoboard1_device.ID,ROVER,ENTIRE_SUBSYSTEM,GPIO_NODE,SENSORS,NOTICE,NOERROR);
								board_timeout_check_passed = true;
								
						   }
					}
					else
					{
						EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
					}
					

				}
				else
				{
					EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
				}	
			}
		}
		if (slowrate_fire == true)
		{
			
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
void print_sensordata(std::vector<BoardControllerNodeProcess::Sensor> sensors)
{
	printf("-----Sensor Data-----\n");
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		printf("[%d] Sensor: %s Type: %s TOV: %f Value: %f\n",
				(int)i,sensors.at(i).name.c_str(),sensors.at(i).type.c_str(),sensors.at(i).signal.tov.toSec(),sensors.at(i).signal.value);
	}
}

