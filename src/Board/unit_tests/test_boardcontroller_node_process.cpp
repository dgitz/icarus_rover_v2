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
    ros_device.SensorCount = 2;
    ros_device.ID = 17;
    
    icarus_rover_v2::device arduinoboard1_device;
    arduinoboard1_device.DeviceName = "ArduinoBoard1";
    arduinoboard1_device.DeviceParent = ros_DeviceName;
    arduinoboard1_device.DeviceType = "ArduinoBoard";
    arduinoboard1_device.BoardCount = 0;
    arduinoboard1_device.SensorCount = 0;
    arduinoboard1_device.ID = 1;
    {
    	icarus_rover_v2::pin newpin;
    	newpin.ConnectedDevice = "BucketAngleSensor";
    	newpin.Name = "AnalogInput0";
    	newpin.Number = 0;
    	newpin.Function = "AnalogInput";
    	arduinoboard1_device.pins.push_back(newpin);
    }

    {
    	icarus_rover_v2::pin newpin;
    	newpin.ConnectedDevice = "BucketLiftSensor";
    	newpin.Name = "AnalogInput1";
    	newpin.Number = 1;
    	newpin.Function = "AnalogInput";
    	arduinoboard1_device.pins.push_back(newpin);
    }

    BoardControllerNodeProcess *process;
    process = new BoardControllerNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    diagnostic = process->new_devicemsg(ros_device);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    diagnostic = process->new_devicemsg(arduinoboard1_device);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);

    EXPECT_TRUE(process->is_ready() == true);
    uint16_t v1 = 123;
    uint16_t v2 = 456;
    diagnostic = process->new_message_GetANAPort1(arduinoboard1_device.ID,v1,v2,0,0,0,0);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    std::vector<Sensor> sensors = process->get_sensordata();
    EXPECT_TRUE(sensors.size() == ros_device.SensorCount);
    EXPECT_TRUE(sensors.at(0).value > 0);
    EXPECT_TRUE(sensors.at(1).value > 0);
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
		printf("[%d] Sensor: %s Type: %s Value: %f\n",
				i,sensors.at(i).name.c_str(),sensors.at(i).type.c_str(),sensors.at(i).value);
	}
}

