#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include <boost/lexical_cast.hpp>
#include "../imu_node_process.h"

std::string Node_Name = "/unittest_imu_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;


IMUNodeProcess *initialized_process;
std::string generate_imudata(int id,int timer);
double get_random(double scale);
int get_random(int scale);
IMUNodeProcess setupprocess(int sensorcount)
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

	IMUNodeProcess process;
	diagnostic = process.init(diagnostic,std::string(Host_Name));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	icarus_rover_v2::device newdevicemsg;
	newdevicemsg.DeviceName = ros_DeviceName;
	newdevicemsg.BoardCount = 0;
	newdevicemsg.Architecture = "x86_64";
	newdevicemsg.DeviceParent = "None";
	newdevicemsg.DeviceType = "ComputeModule";
	newdevicemsg.ID = 1;
	newdevicemsg.SensorCount = sensorcount;
	newdevicemsg.ShieldCount = 0;
	process.set_mydevice(newdevicemsg);
	icarus_rover_v2::device myDevice = process.get_mydevice();
	EXPECT_TRUE(myDevice.DeviceName == newdevicemsg.DeviceName);
	for(int i = 0; i < sensorcount; i++)
	{
		icarus_rover_v2::device imumsg;
		char tempstr[128];
		sprintf(tempstr,"IMU%d",i+1);
		imumsg.DeviceName = std::string(tempstr);
		imumsg.DeviceParent = ros_DeviceName;
		imumsg.DeviceType = "Sensor";
		imumsg.ID = i+1;
		imumsg.PartNumber = "110012";
		diagnostic = process.new_devicemsg(imumsg);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	EXPECT_TRUE(process.get_sensors().size() == myDevice.SensorCount);
	return process;
}
bool check_if_initialized(IMUNodeProcess process);
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

    IMUNodeProcess *process;
    process = new IMUNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
}
TEST(Initialization,SensorInitialization)
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

	IMUNodeProcess *process;
	process = new IMUNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	icarus_rover_v2::device newdevicemsg;
	newdevicemsg.DeviceName = ros_DeviceName;
	newdevicemsg.BoardCount = 0;
	newdevicemsg.Architecture = "x86_64";
	newdevicemsg.DeviceParent = "None";
	newdevicemsg.DeviceType = "ComputeModule";
	newdevicemsg.ID = 1;
	newdevicemsg.SensorCount = 2;
	newdevicemsg.ShieldCount = 0;
	process->set_mydevice(newdevicemsg);
	icarus_rover_v2::device myDevice = process->get_mydevice();
	EXPECT_TRUE(myDevice.DeviceName == newdevicemsg.DeviceName);

	{
		icarus_rover_v2::device imumsg;
		imumsg.DeviceName = "IMU1";
		imumsg.DeviceParent = ros_DeviceName;
		imumsg.DeviceType = "Sensor";
		imumsg.ID = 1;
		imumsg.PartNumber = "110012";
		diagnostic = process->new_devicemsg(imumsg);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	{
		icarus_rover_v2::device imumsg;
		imumsg.DeviceName = "IMU2";
		imumsg.DeviceParent = ros_DeviceName;
		imumsg.DeviceType = "Sensor";
		imumsg.ID = 1;
		imumsg.PartNumber = "110012";
		diagnostic = process->new_devicemsg(imumsg);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	EXPECT_TRUE(process->get_sensors().size() == myDevice.SensorCount);
}
TEST(SensorProcess,NormalSensorOperation)
{
	int cur_time_ms = 0;
	double dt = 0.005;

	IMUNodeProcess process = setupprocess(1);
	icarus_rover_v2::diagnostic diagnostic = process.update(dt);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	for(int i = 0; i < 1000; i ++)
	{
		EXPECT_TRUE(process.new_message(generate_imudata(1,cur_time_ms)));
		cur_time_ms += 10;
		if(process.imudata_ready(1))
		{
			icarus_rover_v2::imu data = process.get_imudata(1);
			printf("imu: %f %f %f \n",data.tov,data.xacc.value,data.xacc.rms);
		}
	}


}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
std::string generate_imudata(int id,int timer)
{
	std::string tempstr;
	tempstr =  boost::lexical_cast<std::string>(int(id)) + "," +
			boost::lexical_cast<std::string>(int(timer)) + "," +
			boost::lexical_cast<std::string>(double(get_random(1000.0))) + "," +
			boost::lexical_cast<std::string>(double(get_random(1000.0))) + "," +
			boost::lexical_cast<std::string>(double(get_random(1000.0))) + "," +
			boost::lexical_cast<std::string>(double(get_random(1000.0))) + "," +
			boost::lexical_cast<std::string>(double(get_random(1000.0))) + "," +
			boost::lexical_cast<std::string>(double(get_random(1000.0))) + "," +
			boost::lexical_cast<std::string>(int(get_random(100))) + "," +
			boost::lexical_cast<std::string>(int(get_random(100))) + "," +
			boost::lexical_cast<std::string>(int(get_random(100)));

	return tempstr;
}
double get_random(double scale)
{
	double v = (double)(rand() % 1000);
	v = (v/500.0)-1.0;
	v = scale*v;
	return v;
}
int get_random(int scale)
{
	double v = get_random((double)scale);
	return (int)v;
}
