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
std::string generate_imudata(int timer,int seq);
double get_random(double scale);
int get_random(int scale);
void print_matrix(MatrixXd m);

IMUNodeProcess* initializeprocess()
{
    icarus_rover_v2::diagnostic diagnostic;
    diagnostic.DeviceName = ros_DeviceName;
    diagnostic.Node_Name = Node_Name;
    diagnostic.System = ROVER;
    diagnostic.SubSystem = ROBOT_CONTROLLER;
    diagnostic.Component = POSE_NODE;

    diagnostic.Diagnostic_Type = NOERROR;
    diagnostic.Level = INFO;
    diagnostic.Diagnostic_Message = INITIALIZING;
    diagnostic.Description = "Node Initializing";
    
    icarus_rover_v2::device device;
    device.DeviceName = diagnostic.DeviceName;
    device.BoardCount = 0;
    device.SensorCount = 2;
    device.DeviceParent = "None";
    
   



    IMUNodeProcess *process;
    process = new IMUNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process->get_initialized() == false);
    process->set_mydevice(device);
    EXPECT_TRUE(process->get_initialized() == true);
    EXPECT_TRUE(process->get_ready() == false);
    
    {
        icarus_rover_v2::device left_imu;
        left_imu.DeviceName = "LeftIMU";
        left_imu.DeviceType = "IMU";
        left_imu.DeviceParent = ros_DeviceName;
        left_imu.ID = 0;
        left_imu.PartNumber = "110013";
        diagnostic = process->new_devicemsg(microphone);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
    }
    
    {
        icarus_rover_v2::device right_imu;
        right_imu.DeviceName = "RightIMU";
        right_imu.DeviceType = "IMU";
        right_imu.ID = 1;
        right_imu.DeviceParent = ros_DeviceName;
        right_imu.PartNumber = "110013";
        diagnostic = process->new_devicemsg(microphone);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
    }
    
    
} 
IMUNodeProcess* readyprocess(IMUNodeProcess* process)
{
    icarus_rover_v2::diagnostic diag = process->update(0.0,0);
    EXPECT_TRUE(diag.Level <= NOTICE);
    EXPECT_TRUE(process->get_ready() == true);
    return process;
}
TEST(Template,Process_Initialization)
{
    IMUNodeProcess* process = initializeprocess();
}
/*
IMUNodeProcess setupprocess()
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
	newdevicemsg.SensorCount = 1;
	newdevicemsg.ShieldCount = 0;
	process.set_mydevice(newdevicemsg);
	icarus_rover_v2::device myDevice = process.get_mydevice();
	EXPECT_TRUE(myDevice.DeviceName == newdevicemsg.DeviceName);
	icarus_rover_v2::device imumsg;
	char tempstr[128];
	sprintf(tempstr,"IMU%d",1);
	imumsg.DeviceName = std::string(tempstr);
	imumsg.DeviceParent = ros_DeviceName;
	imumsg.DeviceType = "Sensor";
	imumsg.ID = 1;
	imumsg.PartNumber = "110012";
	process.set_sensorname(imumsg.PartNumber,imumsg.DeviceName,imumsg.ID);
	diagnostic = process.new_devicemsg(imumsg);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process.load_sensorinfo());
    EXPECT_TRUE(process.get_initialized());
    EXPECT_TRUE(process.set_mountingangle(0.0,0.0,0.0));
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
    process->set_sensorname("110012","IMU1",1);
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
	EXPECT_TRUE(process->load_sensorinfo());
	
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
	
	//EXPECT_TRUE(process->get_sensors().size() == myDevice.SensorCount);
}
TEST(SensorProcess,NormalSensorOperation)
{
	int cur_time_ms = 0;
	double dt = 0.005;

	IMUNodeProcess process = setupprocess();
	icarus_rover_v2::diagnostic diagnostic = process.update(dt);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	icarus_rover_v2::imu last_data;
	last_data.tov = 0.0;
	for(int i = 0; i <= 2000; i++)
	{
		cur_time_ms += 10;
		//EXPECT_TRUE(process.new_message(generate_imudata(cur_time_ms,i)));
		unsigned char msg[256];
		std::string tempstr = generate_imudata(cur_time_ms,i);
		//strncpy(msg,tempstr,sizeof(msg));
		strcpy((char *) msg,tempstr.c_str());
		EXPECT_TRUE(process.new_serialmessage(msg,tempstr.size()));

		icarus_rover_v2::imu data;
		bool status;
		data = process.get_imudata(&status);
		EXPECT_TRUE(status);
		if(status)
		{
			EXPECT_TRUE(data.tov != last_data.tov);
			//icarus_rover_v2::imu data = process.get_imudata(1);
            if((i % 1000) == 0)
            {
                printf("[%d] imu: %f "
                		"xacc: %f %f %d "
                		"yacc: %f %f %d "
                		"zacc: %f %f %d "
                		"xgyro: %f %f %d "
                		"ygyro: %f %f %d "
                		"zgyro: %f %f %d "
                		"xmag: %f %f %d "
                		"ymag: %f %f %d "
                		"zmag: %f %f %d\n",
						i,data.tov,
						data.xacc.value,data.xacc.rms,data.xacc.status,
						data.yacc.value,data.yacc.rms,data.yacc.status,
						data.zacc.value,data.zacc.rms,data.zacc.status,
						data.xgyro.value,data.xgyro.rms,data.xgyro.status,
						data.ygyro.value,data.ygyro.rms,data.ygyro.status,
						data.zgyro.value,data.zgyro.rms,data.zgyro.status,
						data.xmag.value,data.xmag.rms,data.xmag.status,
						data.ymag.value,data.ymag.rms,data.ymag.status,
						data.zmag.value,data.zmag.rms,data.zmag.status);

            }
            last_data = data;

		}

	}


}
TEST(SensorProcess,Mounting)
{
	int cur_time_ms = 0;
	double dt = 0.005;

	IMUNodeProcess process = setupprocess();
	icarus_rover_v2::diagnostic diagnostic = process.update(dt);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.set_mountingangle(0.0,0.0,0.0));
    
    MatrixXd r;
    r = process.get_rotationmatrix();
    for(int i = 0; i < r.cols(); i++)//R should be an identity matrix
    {
        for(int j = 0; j < r.rows(); j++)
        {
            if(i == j)
            {
                EXPECT_TRUE((abs(r(i,j)-1.0) <= .0000001));
            }
            else
            {
                EXPECT_TRUE((abs(r(i,j)) <= .0000001));
            }
        }
    }
    {
        MatrixXd v = MatrixXd::Zero(3,1);
        v(0,0) = 1.0;
        MatrixXd u = r*v;
        
        printf("Multiplying Identity: ");
        print_matrix(r);
        printf(" With v: ");
        print_matrix(v);
        printf("Gives: ");
        print_matrix(u);
        
    }
    
    EXPECT_TRUE(process.set_mountingangle(0.0,M_PI,0.0));
    r = process.get_rotationmatrix();
    {
        MatrixXd v = MatrixXd::Zero(3,1);
        v(0,0) = 1.0;
        v(1,0) = 2.0;
        v(2,0) = 3.0;
        MatrixXd u = r*v; 
        
        printf("Multiplying Rotation Matrix (rotated roll left by 180 deg): ");
        print_matrix(r);
        printf(" With v: ");
        print_matrix(v);
        printf("Gives: ");
        print_matrix(u);
        
    }
    EXPECT_TRUE(process.set_mountingangle(M_PI/2.0,0.0,0.0));
    r = process.get_rotationmatrix();
    {
        MatrixXd v = MatrixXd::Zero(3,1);
        v(0,0) = 1.0;
        v(1,0) = 2.0;
        v(2,0) = 3.0;
        MatrixXd u = r*v;  
        
        printf("Multiplying Rotation Matrix (rotated pitch forward by 90 deg): ");
        print_matrix(r);
        printf(" With v: ");
        print_matrix(v);
        printf("Gives: ");
        print_matrix(u);
        
    }
    EXPECT_TRUE(process.set_mountingangle(0.0,0.0,M_PI/2.0));
    r = process.get_rotationmatrix();
    {
        MatrixXd v = MatrixXd::Zero(3,1);
        v(0,0) = 1.0;
        v(1,0) = 2.0;
        v(2,0) = 3.0;
        MatrixXd u = r*v;  
        
        printf("Multiplying Rotation Matrix (rotated yaw left by 90 deg): ");
        print_matrix(r);
        printf(" With v: ");
        print_matrix(v);
        printf("Gives: ");
        print_matrix(u);
        
    }
    EXPECT_TRUE(process.set_mountingangle(M_PI/2.0,M_PI/2.0,M_PI/2.0));
    r = process.get_rotationmatrix();
    {
        MatrixXd v = MatrixXd::Zero(3,1);
        v(0,0) = 1.0;
        v(1,0) = 0.0;
        v(2,0) = 0.0;
        MatrixXd u = r*v;  
        
        printf("Multiplying Rotation Matrix: ");
        print_matrix(r);
        printf(" With v: ");
        print_matrix(v);
        printf("Gives: ");
        print_matrix(u);
        
    }
    
}
*/
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
/*
std::string generate_imudata(int timer,int seq)
{
	std::string tempstr;
	tempstr =  boost::lexical_cast<std::string>(int(timer)) + "," +
			boost::lexical_cast<std::string>(int(seq)) + "," +
			boost::lexical_cast<std::string>(double(5.0)) + "," +
			boost::lexical_cast<std::string>(double(10.0)) + "," +
			boost::lexical_cast<std::string>(double(7.0)) + "," +
			boost::lexical_cast<std::string>(double(11.0)) + "," +
			boost::lexical_cast<std::string>(double(13.0)) + "," +
			boost::lexical_cast<std::string>(double(1001.0)) + "," +
			boost::lexical_cast<std::string>(int(2001)) + "," +
			boost::lexical_cast<std::string>(int(4345)) + "," +
			boost::lexical_cast<std::string>(int(-5678)) + ",";

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
void print_matrix(MatrixXd m)
{
    std::cout << std::endl << m << std::endl;
}
*/