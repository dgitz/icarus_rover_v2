#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../boardcontroller_node_process.h"

std::string Node_Name = "/unittest_boardcontroller_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
std::string ros_DeviceType = "ControlModule";

BoardControllerNodeProcess *initialized_process;


bool check_if_initialized(BoardControllerNodeProcess process);
void print_sensordata(std::vector<Sensor> sensors);
TEST(Template,ProcessInitialization)
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

	BoardControllerNodeProcess *process;
	process = new BoardControllerNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
}
TEST(DeviceInitialization,DeviceInitialization_ArduinoBoard)
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

	icarus_rover_v2::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = "";
	ros_device.DeviceType = ros_DeviceType;
	ros_device.BoardCount = 1;
	ros_device.SensorCount = 4;
	ros_device.ID = 17;

	icarus_rover_v2::device arduinoboard1_device;
	arduinoboard1_device.DeviceName = "ArduinoBoard1";
	arduinoboard1_device.DeviceParent = ros_DeviceName;
	arduinoboard1_device.DeviceType = "ArduinoBoard";
	arduinoboard1_device.BoardCount = 0;
	arduinoboard1_device.SensorCount = 4;
	arduinoboard1_device.ID = 0;

	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedSensor = "LeftEncoder";
		newpin.Name = "LeftEncoder";
		newpin.Number = 0;
		newpin.Function = "QuadratureEncoderInput";
		arduinoboard1_device.pins.push_back(newpin);
	}

	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedSensor = "RightEncoder";
		newpin.Name = "RightEncoder";
		newpin.Number = 1;
		newpin.Function = "QuadratureEncoderInput";
		arduinoboard1_device.pins.push_back(newpin);
	}

    {
    	icarus_rover_v2::pin newpin;
    	newpin.ConnectedSensor = "BucketAngleSensor";
    	newpin.Name = "AnalogInput0";
    	newpin.Number = 0;
    	newpin.Function = "AnalogInput";
    	arduinoboard1_device.pins.push_back(newpin);
    }

    {
    	icarus_rover_v2::pin newpin;
    	newpin.ConnectedSensor = "BucketLiftSensor";
    	newpin.Name = "AnalogInput1";
    	newpin.Number = 1;
    	newpin.Function = "AnalogInput";
    	arduinoboard1_device.pins.push_back(newpin);
    }


	BoardControllerNodeProcess *process;
	process = new BoardControllerNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	process->set_mydevice(ros_device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);

	diagnostic = process->new_devicemsg(arduinoboard1_device);
	printf("%s\n",diagnostic.Description.c_str());
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	EXPECT_TRUE(process->is_ready() == true);
	EXPECT_TRUE(process->get_boarddiagnostics().size() == 1);
	diagnostic = process->new_message_Diagnostic(arduinoboard1_device.ID,ROVER,ENTIRE_SUBSYSTEM,GPIO_NODE,SENSORS,WARN,DROPPING_PACKETS);
	EXPECT_TRUE(diagnostic.Level == WARN);
	EXPECT_TRUE(process->get_ready_to_arm() == false);
	diagnostic = process->new_message_Diagnostic(arduinoboard1_device.ID,ROVER,ENTIRE_SUBSYSTEM,GPIO_NODE,SENSORS,NOTICE,INITIALIZING);
	EXPECT_TRUE(process->get_ready_to_arm() == false);
	diagnostic = process->new_message_Diagnostic(arduinoboard1_device.ID,ROVER,ENTIRE_SUBSYSTEM,GPIO_NODE,SENSORS,NOTICE,NOERROR);
	EXPECT_TRUE(process->get_ready_to_arm() == true);

	uint16_t v1 = 123;
	uint16_t v2 = 456;
	diagnostic = process->new_message_GetANAPort1(arduinoboard1_device.ID,0.121,v1,v2,0,0,0,0);

	int16_t a1 = 123;
	int16_t a2 = -456;
	diagnostic = process->new_message_GetDIOPort1(arduinoboard1_device.ID,1.234,a1,a2);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	std::vector<Sensor> sensors = process->get_sensordata();
	EXPECT_TRUE(sensors.size() == ros_device.SensorCount);
	print_sensordata(process->get_sensordata());



}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
void print_sensordata(std::vector<Sensor> sensors)
{
	printf("-----Sensor Data-----\n");
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		printf("[%d] Sensor: %s Type: %s TOV: %f Value: %f\n",
				i,sensors.at(i).name.c_str(),sensors.at(i).type.c_str(),sensors.at(i).tov,sensors.at(i).value);
	}
}

