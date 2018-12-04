#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../autodrive_node_process.h"

std::string Node_Name = "/unittest_autodrive_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;


AutoDriveNodeProcess *initialized_process;


bool check_if_initialized(AutoDriveNodeProcess process);
void print_controlgroupinfo(AutoDriveNodeProcess *process,ControlGroup cg);
TEST(Template,ProcessInitialization)
{
    icarus_rover_v2::diagnostic diagnostic;
    diagnostic.DeviceName = ros_DeviceName;
    diagnostic.Node_Name = Node_Name;
    diagnostic.System = ROVER;
    diagnostic.SubSystem = ROBOT_CONTROLLER;
    diagnostic.Component = CONTROLLER_NODE;

    diagnostic.Diagnostic_Type = NOERROR;
    diagnostic.Level = INFO;
    diagnostic.Diagnostic_Message = INITIALIZING;
    diagnostic.Description = "Node Initializing";

    AutoDriveNodeProcess *process;
    process = new AutoDriveNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
	icarus_rover_v2::device ControlModule1;
	ControlModule1.DeviceName = "ControlModule1";
	icarus_rover_v2::pin pin;
	pin.Name = "SteerServo";
	pin.Function = "PWMOutput";
	pin.MinValue = 1000;
	pin.MaxValue = 2000;
	pin.DefaultValue = 1500;
	pin.Value = 1500;
	
	ControlModule1.pins.push_back(pin);
	diagnostic = process->new_devicemsg(ControlModule1);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(.01);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_initialized() == true);
	initialized_process = process;
}
TEST(Operation,ControlGroup_SteerServo)
{
	AutoDriveNodeProcess *process;
	process = initialized_process;
	icarus_rover_v2::diagnostic diagnostic = process->update(0.01);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_initialized() == true);
	std::vector<ControlGroup> controlgroups = process->get_controlgroups();
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		print_controlgroupinfo(process,controlgroups.at(i));
	}
	
	icarus_rover_v2::controlgroup msg;
	msg.name = "AutoSteer";
	msg.type = "PID";
	msg.value1 = 22.0;
	msg.value2 = 0.01;
	msg.value3 = -.3;
	msg.maxvalue = 1800;
	msg.minvalue = 1300;
	msg.defaultvalue = 1450;
	diagnostic = process->new_tunecontrolgroupmsg(msg);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	
	diagnostic = process->new_sensormsg("AutoSteer",1.23);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	
	diagnostic = process->new_commandmsg("AutoSteer",1.01);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	controlgroups = process->get_controlgroups();
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		print_controlgroupinfo(process,controlgroups.at(i));
	}
	
	diagnostic = process->new_armedstatemsg(ARMEDSTATUS_ARMED);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(.01);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	controlgroups = process->get_controlgroups();
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		print_controlgroupinfo(process,controlgroups.at(i));
	}
	std::vector<icarus_rover_v2::pin> pins = process->get_controlgroup_pins("/ControlModule1/PWMOutput");
	for(std::size_t i = 0; i < pins.size(); i++)
	{
		printf("Pin: %s Value: %d\r\n",pins.at(i).Name.c_str(),pins.at(i).Value);
	}
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
void print_controlgroupinfo(AutoDriveNodeProcess *process,ControlGroup cg)
{
	printf("---------------------------\r\n");
	printf("ControlGroup: %s\n",cg.name.c_str());
	printf("\tCommand:\r\n");
	printf("\t\tSource: %s:%s(%d)\r\n",cg.command.topic.c_str(),cg.command.name.c_str(),cg.command.index);
	printf("\t\tValue: %f\r\n",cg.command.input);
	printf("\tInput Sensor: %s\r\n",process->get_sensorname(cg.sensor.name).c_str());
	printf("\t\tValue: %f\r\n",cg.sensor.input);
	printf("\tGain: (%s)\r\n",cg.gain.type.c_str());
	printf("\t\tP: %0.4f I: %0.4f D: %0.4f\r\n",cg.gain.P,cg.gain.I,cg.gain.D);
	printf("\tOutput: %s (%s)\r\n",cg.output.topic.c_str(),cg.output.name.c_str());
	printf("\t\tMin: %d Max: %d Default: %d Value: %d\r\n",cg.output.pin.MinValue,cg.output.pin.MaxValue,cg.output.pin.DefaultValue,cg.output.pin.Value);
}
/*
struct Command
{
	std::string type;
	std::string topic;
	std::string name;
	int index;
	double min_value;
	double max_value;
};
struct InputSensor
{
	std::string type;
	uint8_t name;
};


struct ControlGroup
{
    std::string name;
	Command command;
	InputSensor sensor;
	Output output;
	Gain gain;
};
*/

