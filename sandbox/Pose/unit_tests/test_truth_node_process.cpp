#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include <boost/lexical_cast.hpp>
#include "../truth_node_process.h"

std::string Node_Name = "/unittest_truth_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;


TruthNodeProcess *initialized_process;
std::string generate_truthdata(int timer,int seq);
double get_random(double scale);
int get_random(int scale);
TruthNodeProcess setupprocess()
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

	TruthNodeProcess process;
	diagnostic = process.init(diagnostic,std::string(Host_Name));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	icarus_rover_v2::device newdevicemsg;
	newdevicemsg.DeviceName = ros_DeviceName;
	newdevicemsg.BoardCount = 0;
	newdevicemsg.Architecture = "x86_64";
	newdevicemsg.DeviceParent = "None";
	newdevicemsg.DeviceType = "ComputeModule";
	newdevicemsg.ID = 1;
	newdevicemsg.SensorCount = 1;
	newdevicemsg.ShieldCount = 0;
	process.set_mydevice(newdevicemsg);
	icarus_rover_v2::device myDevice = process.get_mydevice();
	EXPECT_TRUE(myDevice.DeviceName == newdevicemsg.DeviceName);
	icarus_rover_v2::device truthmsg;
	char tempstr[128];
	sprintf(tempstr,"Truth%d",1);
	truthmsg.DeviceName = std::string(tempstr);
	truthmsg.DeviceParent = ros_DeviceName;
	truthmsg.DeviceType = "Sensor";
	truthmsg.ID = 1;
	truthmsg.PartNumber = "810090";
	process.set_sensorname(truthmsg.PartNumber,truthmsg.DeviceName,truthmsg.ID);
	diagnostic = process.new_devicemsg(truthmsg);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process.load_sensorinfo());
    EXPECT_TRUE(process.get_initialized());
	return process;
}
bool check_if_initialized(TruthNodeProcess process);
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

    TruthNodeProcess *process;
    process = new TruthNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    process->set_sensorname("","Truth1",1);
    EXPECT_TRUE(process->load_sensorinfo());
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

	TruthNodeProcess *process;
	process = new TruthNodeProcess;
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
		icarus_rover_v2::device truthmsg;
		truthmsg.DeviceName = "Truth1";
		truthmsg.DeviceParent = ros_DeviceName;
		truthmsg.DeviceType = "Sensor";
		truthmsg.ID = 1;
		truthmsg.PartNumber = "810090";
		diagnostic = process->new_devicemsg(truthmsg);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	EXPECT_TRUE(process->load_sensorinfo());

}
TEST(SensorProcess,NormalSensorOperation)
{
	int cur_time_ms = 0;
	double dt = 0.005;

	TruthNodeProcess process = setupprocess();
	icarus_rover_v2::diagnostic diagnostic = process.update(dt);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	icarus_rover_v2::pose last_data;
	last_data.tov = 0.0;
	for(int i = 0; i <= 2000; i++)
	{
		cur_time_ms += 10;
		//EXPECT_TRUE(process.new_message(generate_imudata(cur_time_ms,i)));
		unsigned char msg[256];
		std::string tempstr = generate_truthdata(cur_time_ms,i);
		//strncpy(msg,tempstr,sizeof(msg));
		strcpy((char *) msg,tempstr.c_str());
		EXPECT_TRUE(process.new_serialmessage(msg,tempstr.size()));

		icarus_rover_v2::pose data;
		bool status;
		status = process.get_truthdata(&data);
		EXPECT_TRUE(status);
		if(status)
		{
			EXPECT_TRUE(data.tov != last_data.tov);
			//icarus_rover_v2::imu data = process.get_imudata(1);
			if((i % 1000) == 0)
			{
				printf("[%d] truth: %d %f "
						"roll: %f  %f %d "
						"pitch:  %f %f %d "
						"yaw: %f %f %d "
						"east: %f %f %d "
						"north: %f %f %d "
						"elev: %f %f %d "
						"wheelspeed: %f %f %d "
						"groundspeed: %f %f %d\n",
						i,data.header.seq,data.tov,
						data.roll.value,data.roll.rms,data.roll.status,
						data.pitch.value,data.pitch.rms,data.pitch.status,
						data.yaw.value,data.yaw.rms,data.yaw.status,
						data.east.value,data.east.rms,data.east.status,
						data.north.value,data.north.rms,data.north.status,
						data.elev.value,data.elev.rms,data.elev.status,
						data.wheelspeed.value,data.wheelspeed.rms,data.wheelspeed.status,
						data.groundspeed.value,data.groundspeed.rms,data.groundspeed.status);
			}
			last_data = data;

		}


	}



}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
std::string generate_truthdata(int timer,int seq)
{
	std::string tempstr;
	tempstr =  boost::lexical_cast<std::string>(int(timer)) + "," +
			boost::lexical_cast<std::string>(int(seq)) + "," +
			boost::lexical_cast<std::string>(double(5.0)) + "," + //pitch
			boost::lexical_cast<std::string>(double(-10.0)) + "," + //roll
			boost::lexical_cast<std::string>(double(7.0)) + "," + //yaw
			boost::lexical_cast<std::string>(double(-11.0)) + "," + //east
			boost::lexical_cast<std::string>(double(13.0)) + "," + //north
			boost::lexical_cast<std::string>(double(-1001.0)) + "," + //elev
			boost::lexical_cast<std::string>(double(2001.0)) + "," + //wheelspeed
			boost::lexical_cast<std::string>(double(-4345.0)) + ","; //groundspeed

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
