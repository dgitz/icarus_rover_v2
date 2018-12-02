#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/imu.h"
#include "icarus_rover_v2/signal.h"
#include <sys/time.h>
#include <math.h>
#include "../pose_node_process.h"
#include "dirent.h"
#include <Eigen/Dense>

#define VERBOSE 0
#define IMU_XACC_COLUMN 6
#define IMU_YACC_COLUMN 10
#define IMU_ZACC_COLUMN 14
#define IMU_XGYRO_COLUMN 18
#define IMU_YGYRO_COLUMN 22
#define IMU_ZGYRO_COLUMN 26
#define IMU_XMAG_COLUMN 30
#define IMU_YMAG_COLUMN 34
#define IMU_ZMAG_COLUMN 38
#define EPS 0.0000001
#define POSE_RATE 100
using namespace Eigen;
std::string Node_Name = "/unittest_pose_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
struct Source_Signal
{
	std::string name;
	std::string units;
	std::string type;
	std::string sensorname;
	uint8_t sensorindex;
	bool computed_signal;
	std::string sensorsource;
	std::vector<double> timestamp;
	std::vector<double> value;
	std::vector<uint8_t> status;
	std::vector<double> rms;
};


PoseNodeProcess *initialized_process;

double measure_time_diff(struct timeval a, struct timeval b);
bool check_if_initialized(PoseNodeProcess process);
double measure_timediff(struct timeval a, struct timeval b);
void print_signals(std::vector<Source_Signal> signals);
void print_signals(std::vector<Basic_Signal> signals);
void check_signals(std::vector<Basic_Signal> signals);
void print_signals(std::vector<Extended_Signal> signals);
void print_signals(std::vector<Sensor_Signal> signals);
std::vector<Source_Signal> load_sensordata(std::string path);
std::vector<Source_Signal> get_sourcedatabysensorname(std::string name,std::vector<Source_Signal> data);
icarus_rover_v2::imu get_rossourcedata(std::string name,int index, std::vector<Source_Signal> data);
TEST(Template,ProcessInitialization)
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

    PoseNodeProcess *process;
    process = new PoseNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    initialized_process = process;
}
TEST(PoseModel,LoadSensorDataFromFolder)
{
	std::vector<Source_Signal> raw_data;
	raw_data = load_sensordata("/home/robot/Dropbox/ICARUS/Scout/SIMULATION/Pose/Data/UnitTest_LoadSensorData1/");
	EXPECT_TRUE(raw_data.size() > 0);
	if(VERBOSE == 1)
	{
		print_signals(raw_data);
	}
}
TEST(PoseModel,TimeCompensate)
{
	int IMU_COUNT = 2;
	std::vector<int> sensorsample_index;
	sensorsample_index.resize(IMU_COUNT);
	for(std::size_t i = 0; i < IMU_COUNT; i++)
	{
		sensorsample_index.at(i) = 0;
	};
	std::vector<Source_Signal> raw_data;
	raw_data = load_sensordata("/home/robot/Dropbox/ICARUS/Scout/SIMULATION/Pose/Data/UnitTest_LoadSensorData1/");
	EXPECT_TRUE(raw_data.size() > 0);
	icarus_rover_v2::diagnostic diagnostic;
	PoseNodeProcess *process = initialized_process;

	{
		icarus_rover_v2::device newdevicemsg;
		newdevicemsg.DeviceName = ros_DeviceName;
		newdevicemsg.BoardCount = 0;
		newdevicemsg.Architecture = "x86_64";
		newdevicemsg.DeviceParent = "None";
		newdevicemsg.DeviceType = "ComputeModule";
		newdevicemsg.ID = 1;
		newdevicemsg.SensorCount = IMU_COUNT;
		newdevicemsg.ShieldCount = 0;
		diagnostic = process->new_devicemsg(newdevicemsg);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	for(int i = 0; i < IMU_COUNT; i++)
	{
		std::ostringstream ss;
		ss << "IMU" << i+1;
		icarus_rover_v2::device newdevicemsg;
		newdevicemsg.DeviceName = ss.str();
		newdevicemsg.PartNumber = "110012";
		newdevicemsg.DeviceParent = ros_DeviceName;
		newdevicemsg.DeviceType = "Sensor";
		newdevicemsg.ID = i+1;
		diagnostic = process->new_devicemsg(newdevicemsg);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);

	}
	EXPECT_TRUE(process->is_initialized());
	{
		std::vector<Basic_Signal> signals = process->get_rawsignals("IMU");
		EXPECT_TRUE(signals.size() == raw_data.size());
	}

	double run_time = 0;
	for(std::size_t i = 0; i < raw_data.size(); i++)
	{
		if(raw_data.at(i).timestamp.back() > run_time)
		{
			run_time = raw_data.at(i).timestamp.back();
		}
	}
	double sim_time = 0.0;
	struct timeval start;
	struct timeval finish;
	gettimeofday(&start,NULL);
	while(sim_time <= run_time)
	{
		for(int i = 0; i < IMU_COUNT; i++)
		{
			std::ostringstream ss;
			ss << "IMU" << i+1;
			std::vector<Source_Signal> imu_data = get_sourcedatabysensorname(ss.str(),raw_data);

			EXPECT_TRUE(imu_data.size() > 0);
			if(sim_time > imu_data.at(0).timestamp.at(sensorsample_index.at(i)))
			{
				sensorsample_index.at(i) = sensorsample_index.at(i) + 1;
				icarus_rover_v2::imu imudata = get_rossourcedata(ss.str(),sensorsample_index.at(i),imu_data);
				process->new_imudata(ss.str(),imudata);
			}

		}
		diagnostic = process->update((double)1.0/POSE_RATE);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		sim_time += 1.0/(double)(POSE_RATE);
	}
	gettimeofday(&finish,NULL);
	double etime = measure_time_diff(finish,start);
	EXPECT_TRUE(etime < run_time);
	printf("Test Case Actual Elapsed Time: %f (sec).\n",measure_time_diff(finish,start));
	std::vector<Extended_Signal> signals = process->get_extendedsignals("IMU");
	print_signals(signals);


}
TEST(KalmanFilter,ExampleFilter_SinWave)
{

    icarus_rover_v2::diagnostic diagnostic;
    PoseNodeProcess *process = initialized_process;
    EXPECT_TRUE(process->is_initialized());
    diagnostic = process->new_kalmanfilter("SinWave1", "Position");
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
	{	//KF-SinWave1
		int output_count = 1; //n
		int measurement_count = 1; //m
		MatrixXd C(measurement_count,output_count);
		C(0,0) = 1.0;
		MatrixXd Phi(output_count,output_count);
		Phi(0,0) = 1.0;
		MatrixXd Q(output_count,output_count);
		Q(0,0) = 1.0;
		MatrixXd R(measurement_count,measurement_count);
		R(0,0) = 1.0;
		diagnostic = process->set_kalmanfilter_properties("SinWave1", output_count,measurement_count,C,Phi,Q,R);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
    diagnostic = process->new_kalmanfilter_signal("Example2",0,0.0);
    EXPECT_TRUE(diagnostic.Level > NOTICE);
    diagnostic = process->new_kalmanfilter_signal("SinWave1",0,0.0);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    double time_limit = 10.0;
    double dt = 0.1;
    double timer = 1.3;
    double signal_update_rate = 0.1;
    double next_signal_time = signal_update_rate;
    double signal = 0.0;
    while(timer <= time_limit)
    {
        
        if(timer >= next_signal_time) 
        {
            next_signal_time += signal_update_rate; 
            double noise = 0.5;//static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
            signal = (noise-0.5)+sin(2.0*M_PI*timer/5.0);
            
            diagnostic = process->new_kalmanfilter_signal("SinWave1",0,signal);    
        }
        diagnostic = process->update(dt);
		//printf("%f,%f,%f\n",timer,signal,process->get_kalmanfilter_output("SinWave1",0));
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        timer += dt;
    }
   
}
/*
TEST(KalmanFilter,PoseModel)
{
    icarus_rover_v2::diagnostic diagnostic;
    PoseNodeProcess *process = initialized_process;
    diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
	{	//KF-Yawrate
		int output_count = 1; //n
		int measurement_count = 1; //m
		MatrixXd C(measurement_count,output_count);
		C(0,0) = 1.0;
		MatrixXd Phi(output_count,output_count);
		Phi(0,0) = 1.0;
		MatrixXd Q(output_count,output_count);
		Q(0,0) = 1.0;
		MatrixXd R(measurement_count,measurement_count);
		R(0,0) = 1.0;
		diagnostic = process->set_kalmanfilter_properties("Yawrate", output_count,measurement_count,C,Phi,Q,R);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	{	//KF-Yaw
		int output_count = 2; //n
		int measurement_count = 2; //m
		MatrixXd C(measurement_count,output_count);
		C(0,0) = 1.0;
		C(1,0) = 0.0;
		C(0,1) = 0.0;
		C(1,1) = 1.0;
		MatrixXd Phi(output_count,output_count);
		Phi(0,0) = 1.0;
		Phi(0,1) = .01;
		Phi(1,0) = 0.0;
		Phi(1,1) = 1.0;
		MatrixXd Q(output_count,output_count);
		Q(0,0) = 30.0;
		Q(0,1) = 0.0;
		Q(1,0) = 0.0;
		Q(1,1) = 200.0;
		MatrixXd R(measurement_count,measurement_count);
		R(0,0) = 30.0;
		R(0,1) = 0.0;
		R(1,0) = 0.0;
		R(1,1) = 200.0;
		diagnostic = process->set_kalmanfilter_properties("Yaw", output_count,measurement_count,C,Phi,Q,R);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
    icarus_rover_v2::pose pose;
    pose = process->get_pose();
    EXPECT_TRUE(pose.yaw.status == SIGNALSTATE_INITIALIZING);
    EXPECT_TRUE(pose.yawrate.status == SIGNALSTATE_INITIALIZING);
    
    diagnostic = process->update(0.01);
	pose = process->get_pose();
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
	
    EXPECT_TRUE(pose.yaw.status == SIGNALSTATE_INITIALIZING);
    EXPECT_TRUE(pose.yawrate.status == SIGNALSTATE_INITIALIZING);
    
	diagnostic = process->new_kalmanfilter_signal("Yaw", 1,0.0);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	
    diagnostic = process->update(0.01);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    pose = process->get_pose();
    EXPECT_TRUE(pose.yaw.status == SIGNALSTATE_UPDATED);
    EXPECT_TRUE(pose.yawrate.status == SIGNALSTATE_INITIALIZING);
    
	diagnostic = process->new_kalmanfilter_signal("Yawrate", 0,0.0);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

    diagnostic = process->update(0.01);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    pose = process->get_pose();
    EXPECT_TRUE(pose.yaw.status == SIGNALSTATE_UPDATED);
    EXPECT_TRUE(pose.yawrate.status == SIGNALSTATE_UPDATED);
}
*/
TEST(MathOperation,MatrixOperations)
{
    //Matrix Creation & Access
    {
        MatrixXd m(2,2);
        m(0,0) = 1;
        m(0,1) = 2;
        m(1,0) = 3;
        m(1,1) = 4;
        /*
        m = [1 2;
             3 4]
        */
        EXPECT_TRUE(m(0,0) == 1);
        EXPECT_TRUE(m(0,1) == 2);
        EXPECT_TRUE(m(1,0) == 3);
        EXPECT_TRUE(m(1,1) == 4);
        
        MatrixXd n(3,3);
        n(0,0) = 1.2345;
        n(0,1) = 2.3456;
        n(0,2) = 3.4567;
        n(1,0) = 4.5678;
        n(1,1) = 5.6789;
        n(1,2) = 6.7890;
        n(2,0) = 7.8901;
        n(2,1) = 8.9012;
        n(2,2) = 9.0123;
        
        MatrixXd o(1,1);
        o(0,0) = 23.45;
        
        //std::cout << n << std::endl;
        
    }
    
    //Matrix Transpose
    {
        MatrixXd m(2,2);
        m(0,0) = 1;
        m(0,1) = 2;
        m(1,0) = 3;
        m(1,1) = 4;
        MatrixXd n = m.transpose();
        EXPECT_TRUE(n(0,0) == 1);
        EXPECT_TRUE(n(0,1) == 3);
        EXPECT_TRUE(n(1,0) == 2);
        EXPECT_TRUE(n(1,1) == 4);
        
        for(int i = 1; i < 7; i++)
        {
            MatrixXd m(i,i);
            for(int j = 0; j < i; j++)
            {
                for(int k = 0; k < i; k++)
                {
                    m(j,k) = j+k;
                }
            }
            //std::cout << m << std::endl << std::endl;
            struct timeval start,stop;
            gettimeofday(&start,NULL);
            int count = 1000;
            for(int j = 0; j < count; j++)
            {
                MatrixXd n = m.transpose();
            }
            gettimeofday(&stop,NULL);
            double avgtime_operation = measure_timediff(start,stop)/(double)(count);
            //printf("Transpose for Matrix size: (%d,%d): %0.8f Total time: %0.3f\n",i,i,avgtime_operation,measure_timediff(start,stop));
            
        }
    }
    //Matrix Multiplication
    {

        MatrixXd m(2,2);
        m(0,0) = 1;
        m(0,1) = 2;
        m(1,0) = 3;
        m(1,1) = 4;
        MatrixXd m2 = m*m;
        EXPECT_TRUE(m2(0,0) == 7);
        EXPECT_TRUE(m2(0,1) == 10);
        EXPECT_TRUE(m2(1,0) == 15);
        EXPECT_TRUE(m2(1,1) == 22);
        
        for(int i = 1; i < 7; i++)
        {
            MatrixXd m(i,i);
            for(int j = 0; j < i; j++)
            {
                for(int k = 0; k < i; k++)
                {
                    m(j,k) = j+k;
                }
            }
            //std::cout << m << std::endl << std::endl;
            struct timeval start,stop;
            gettimeofday(&start,NULL);
            int count = 1000;
            for(int j = 0; j < count; j++)
            {
                MatrixXd n = m*m;
            }
            gettimeofday(&stop,NULL);
            double avgtime_operation = measure_timediff(start,stop)/(double)(count);
            //printf("Multiplication for Matrix size: (%d,%d): %0.8f Total time: %0.3f\n",i,i,avgtime_operation,measure_timediff(start,stop));
            
        }
    }
    
    //Matrix Inverse
    {

        MatrixXd m(2,2);
        m(0,0) = 1;
        m(0,1) = 2;
        m(1,0) = 3;
        m(1,1) = 4;
        MatrixXd m2 = m.inverse();
        EXPECT_TRUE(fabs(m2(0,0) - (-2.0)) < EPS);
        EXPECT_TRUE(fabs(m2(0,1) - (1.0)) < EPS);
        EXPECT_TRUE(fabs(m2(1,0) - (1.50)) < EPS);
        EXPECT_TRUE(fabs(m2(1,1) - (-0.5)) < EPS);
        
        
        for(int i = 1; i < 7; i++)
        {
            MatrixXd m(i,i);
            for(int j = 0; j < i; j++)
            {
                for(int k = 0; k < i; k++)
                {
                    m(j,k) = j+k;
                }
            }
            //std::cout << m << std::endl << std::endl;
            struct timeval start,stop;
            gettimeofday(&start,NULL);
            int count = 1000;
            for(int j = 0; j < count; j++)
            {
                MatrixXd n = m.inverse();
            }
            gettimeofday(&stop,NULL);
            double avgtime_operation = measure_timediff(start,stop)/(double)(count);
            //printf("Multiplication for Matrix size: (%d,%d): %0.8f Total time: %0.3f\n",i,i,avgtime_operation,measure_timediff(start,stop));
            
        }
    }
    
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
double measure_timediff(struct timeval a, struct timeval b)
{
    double a_time = (double)a.tv_sec + (double)(a.tv_usec)/1000000.0;
    double b_time = (double)b.tv_sec + (double)(b.tv_usec)/1000000.0;
    return b_time-a_time;
}
void print_signals(std::vector<Basic_Signal> signals)
{
	for(std::size_t i = 0; i < signals.size(); i++)
	{
		printf("[%d] Basic Signal: %s units: %s\n",(int)i,signals.at(i).name.c_str(),signals.at(i).units.c_str());
	}

}
void print_signals(std::vector<Extended_Signal> signals)
{
	for(std::size_t i = 0; i < signals.size(); i++)
	{
		EXPECT_TRUE(signals.at(i).status != SIGNALSTATE_UNDEFINED);
		if(VERBOSE == 1)
		{
			printf("[%d] Extended Signal: %s units: %s Buffer: %d\n",(int)i,signals.at(i).name.c_str(),signals.at(i).units.c_str(),(int)signals.at(i).value_buffer.size());

		}
	}
}
void print_signals(std::vector<Sensor_Signal> signals)
{
	for(std::size_t i = 0; i < signals.size(); i++)
	{
		if(VERBOSE == 1)
		{
			printf("[%d] Sensor Signal: %s units: %s\n",(int)i,signals.at(i).name.c_str(),signals.at(i).units.c_str());
		}
	}
}
void print_signals(std::vector<Source_Signal> signals)
{
	for(std::size_t i = 0; i < signals.size(); i++)
	{
		if(VERBOSE == 1)
		{
			printf("[%d] Source Signal: %s units: %s\n",(int)i,signals.at(i).name.c_str(),signals.at(i).units.c_str());
		}
	}
}
std::vector<Source_Signal> load_sensordata(std::string folder)
		{
	std::vector<Source_Signal> empty;
	std::vector<Source_Signal> raw_imudata;
	raw_imudata.clear();
	std::vector<std::string> imu_datafiles;
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (folder.c_str())) != NULL)
	{
	  while ((ent = readdir (dir)) != NULL)
	  {
		  std::size_t found_lockfile = std::string(ent->d_name).find("lock");
		  std::size_t found_imu = std::string(ent->d_name).find("IMU");
		  if(found_lockfile == std::string::npos)
		  {
			  if(found_imu != std::string::npos) { imu_datafiles.push_back(std::string(ent->d_name)); }
		  }
	  }
	  closedir (dir);
	}
	else
	{
	  printf("Could not open folder: %s. Exiting.\n",folder.c_str());
	  return empty;
	}

	for(std::size_t i = 0; i < imu_datafiles.size(); i++)
	{
		Source_Signal raw_imu_xacc;
		Source_Signal raw_imu_yacc;
		Source_Signal raw_imu_zacc;
		Source_Signal raw_imu_xgyro;
		Source_Signal raw_imu_ygyro;
		Source_Signal raw_imu_zgyro;
		Source_Signal raw_imu_xmag;
		Source_Signal raw_imu_ymag;
		Source_Signal raw_imu_zmag;
		{
			std::ostringstream ss1;
			ss1 << "xacc" << i+1;
			raw_imu_xacc.name = ss1.str();
			raw_imu_xacc.units = "meter/s^2";
			raw_imu_xacc.type = "Linear Acceleration";
			raw_imu_xacc.sensorsource = "110012";
			std::ostringstream ss2;
			ss2 << "IMU" << i+1;
			raw_imu_xacc.sensorname = ss2.str();
			raw_imu_xacc.sensorindex = i+1;
			raw_imu_xacc.computed_signal = 0;
		}
		{
			std::ostringstream ss1;
			ss1 << "yacc" << i+1;
			raw_imu_yacc.name = ss1.str();
			raw_imu_yacc.units = "meter/s^2";
			raw_imu_yacc.type = "Linear Acceleration";
			raw_imu_yacc.sensorsource = "110012";
			std::ostringstream ss2;
			ss2 << "IMU" << i+1;
			raw_imu_yacc.sensorname = ss2.str();
			raw_imu_yacc.sensorindex = i+1;
			raw_imu_yacc.computed_signal = 0;
		}
		{
			std::ostringstream ss1;
			ss1 << "zacc" << i+1;
			raw_imu_zacc.name = ss1.str();
			raw_imu_zacc.units = "meter/s^2";
			raw_imu_zacc.type = "Linear Acceleration";
			raw_imu_zacc.sensorsource = "110012";
			std::ostringstream ss2;
			ss2 << "IMU" << i+1;
			raw_imu_zacc.sensorname = ss2.str();
			raw_imu_zacc.sensorindex = i+1;
			raw_imu_zacc.computed_signal = 0;
		}
		{
			std::ostringstream ss1;
			ss1 << "xgyro" << i+1;
			raw_imu_xgyro.name = ss1.str();
			raw_imu_xgyro.units = "deg/s";
			raw_imu_xgyro.type = "Angle Rate";
			raw_imu_xgyro.sensorsource = "110012";
			std::ostringstream ss2;
			ss2 << "IMU" << i+1;
			raw_imu_xgyro.sensorname = ss2.str();
			raw_imu_xgyro.sensorindex = i+1;
			raw_imu_xgyro.computed_signal = 0;
		}
		{
			std::ostringstream ss1;
			ss1 << "ygyro" << i+1;
			raw_imu_ygyro.name = ss1.str();
			raw_imu_ygyro.units = "deg/s";
			raw_imu_ygyro.type = "Angle Rate";
			raw_imu_ygyro.sensorsource = "110012";
			std::ostringstream ss2;
			ss2 << "IMU" << i+1;
			raw_imu_ygyro.sensorname = ss2.str();
			raw_imu_ygyro.sensorindex = i+1;
			raw_imu_ygyro.computed_signal = 0;
		}
		{
			std::ostringstream ss1;
			ss1 << "zgyro" << i+1;
			raw_imu_zgyro.name = ss1.str();
			raw_imu_zgyro.units = "deg/s";
			raw_imu_zgyro.type = "Angle Rate";
			raw_imu_zgyro.sensorsource = "110012";
			std::ostringstream ss2;
			ss2 << "IMU" << i+1;
			raw_imu_zgyro.sensorname = ss2.str();
			raw_imu_zgyro.sensorindex = i+1;
			raw_imu_zgyro.computed_signal = 0;
		}
		{
			std::ostringstream ss1;
			ss1 << "xmag" << i+1;
			raw_imu_xmag.name = ss1.str();
			raw_imu_xmag.units = "uTesla";
			raw_imu_xmag.type = "Magnetic Field";
			raw_imu_xmag.sensorsource = "110012";
			std::ostringstream ss2;
			ss2 << "IMU" << i+1;
			raw_imu_xmag.sensorname = ss2.str();
			raw_imu_xmag.sensorindex = i+1;
			raw_imu_xmag.computed_signal = 0;
		}
		{
			std::ostringstream ss1;
			ss1 << "ymag" << i+1;
			raw_imu_ymag.name = ss1.str();
			raw_imu_ymag.units = "uTesla";
			raw_imu_ymag.type = "Magnetic Field";
			raw_imu_ymag.sensorsource = "110012";
			std::ostringstream ss2;
			ss2 << "IMU" << i+1;
			raw_imu_ymag.sensorname = ss2.str();
			raw_imu_ymag.sensorindex = i+1;
			raw_imu_ymag.computed_signal = 0;
		}
		{
			std::ostringstream ss1;
			ss1 << "zmag" << i+1;
			raw_imu_zmag.name = ss1.str();
			raw_imu_zmag.units = "uTesla";
			raw_imu_zmag.type = "Magnetic Field";
			raw_imu_zmag.sensorsource = "110012";
			std::ostringstream ss2;
			ss2 << "IMU" << i+1;
			raw_imu_zmag.sensorname = ss2.str();
			raw_imu_zmag.sensorindex = i+1;
			raw_imu_zmag.computed_signal = 0;
		}


		ifstream file;
		std::string filepath = folder + imu_datafiles.at(i);
		file.open(filepath.c_str());
		std::string line;
		if(file.is_open())
		{
			if(VERBOSE == 1)
			{
				printf("reading: %s\n",filepath.c_str());
			}
			std::getline(file,line); //First row is headers
			while(std::getline(file,line))
			{
				std::vector<std::string> strs;
				boost::split(strs,line,boost::is_any_of(","));

				raw_imu_xacc.timestamp.push_back(std::atof(strs.at(0).c_str()));
				raw_imu_xacc.value.push_back(std::atof(strs.at(IMU_XACC_COLUMN).c_str()));
				raw_imu_xacc.rms.push_back(std::atoi(strs.at(IMU_XACC_COLUMN+2).c_str()));
				raw_imu_xacc.status.push_back(std::atoi(strs.at(IMU_XACC_COLUMN+1).c_str()));

				raw_imu_yacc.timestamp.push_back(std::atof(strs.at(0).c_str()));
				raw_imu_yacc.value.push_back(std::atof(strs.at(IMU_YACC_COLUMN).c_str()));
				raw_imu_yacc.rms.push_back(std::atoi(strs.at(IMU_YACC_COLUMN+2).c_str()));
				raw_imu_yacc.status.push_back(std::atoi(strs.at(IMU_YACC_COLUMN+1).c_str()));

				raw_imu_zacc.timestamp.push_back(std::atof(strs.at(0).c_str()));
				raw_imu_zacc.value.push_back(std::atof(strs.at(IMU_ZACC_COLUMN).c_str()));
				raw_imu_zacc.rms.push_back(std::atoi(strs.at(IMU_ZACC_COLUMN+2).c_str()));
				raw_imu_zacc.status.push_back(std::atoi(strs.at(IMU_ZACC_COLUMN+1).c_str()));

				raw_imu_xgyro.timestamp.push_back(std::atof(strs.at(0).c_str()));
				raw_imu_xgyro.value.push_back(std::atof(strs.at(IMU_XGYRO_COLUMN).c_str()));
				raw_imu_xgyro.rms.push_back(std::atoi(strs.at(IMU_XGYRO_COLUMN+2).c_str()));
				raw_imu_xgyro.status.push_back(std::atoi(strs.at(IMU_XGYRO_COLUMN+1).c_str()));

				raw_imu_ygyro.timestamp.push_back(std::atof(strs.at(0).c_str()));
				raw_imu_ygyro.value.push_back(std::atof(strs.at(IMU_YGYRO_COLUMN).c_str()));
				raw_imu_ygyro.rms.push_back(std::atoi(strs.at(IMU_YGYRO_COLUMN+2).c_str()));
				raw_imu_ygyro.status.push_back(std::atoi(strs.at(IMU_YGYRO_COLUMN+1).c_str()));

				raw_imu_zgyro.timestamp.push_back(std::atof(strs.at(0).c_str()));
				raw_imu_zgyro.value.push_back(std::atof(strs.at(IMU_ZGYRO_COLUMN).c_str()));
				raw_imu_zgyro.rms.push_back(std::atoi(strs.at(IMU_ZGYRO_COLUMN+2).c_str()));
				raw_imu_zgyro.status.push_back(std::atoi(strs.at(IMU_ZGYRO_COLUMN+1).c_str()));

				raw_imu_xmag.timestamp.push_back(std::atof(strs.at(0).c_str()));
				raw_imu_xmag.value.push_back(std::atof(strs.at(IMU_XMAG_COLUMN).c_str()));
				raw_imu_xmag.rms.push_back(std::atoi(strs.at(IMU_XMAG_COLUMN+2).c_str()));
				raw_imu_xmag.status.push_back(std::atoi(strs.at(IMU_XMAG_COLUMN+1).c_str()));

				raw_imu_ymag.timestamp.push_back(std::atof(strs.at(0).c_str()));
				raw_imu_ymag.value.push_back(std::atof(strs.at(IMU_YMAG_COLUMN).c_str()));
				raw_imu_ymag.rms.push_back(std::atoi(strs.at(IMU_YMAG_COLUMN+2).c_str()));
				raw_imu_ymag.status.push_back(std::atoi(strs.at(IMU_YMAG_COLUMN+1).c_str()));

				raw_imu_zmag.timestamp.push_back(std::atof(strs.at(0).c_str()));
				raw_imu_zmag.value.push_back(std::atof(strs.at(IMU_ZMAG_COLUMN).c_str()));
				raw_imu_zmag.rms.push_back(std::atoi(strs.at(IMU_ZMAG_COLUMN+2).c_str()));
				raw_imu_zmag.status.push_back(std::atoi(strs.at(IMU_ZMAG_COLUMN+1).c_str()));
			}
			raw_imudata.push_back(raw_imu_xacc);
			raw_imudata.push_back(raw_imu_yacc);
			raw_imudata.push_back(raw_imu_zacc);
			raw_imudata.push_back(raw_imu_xgyro);
			raw_imudata.push_back(raw_imu_ygyro);
			raw_imudata.push_back(raw_imu_zgyro);
			raw_imudata.push_back(raw_imu_xmag);
			raw_imudata.push_back(raw_imu_ymag);
			raw_imudata.push_back(raw_imu_zmag);
			file.close();
		}
		else
		{
			printf("Could not open file: %s\n",filepath.c_str());
			return empty;
		}
	}
	return raw_imudata;
}
std::vector<Source_Signal> get_sourcedatabysensorname(std::string name,std::vector<Source_Signal> data)
{
	std::vector<Source_Signal> output;
	for(std::size_t i = 0; i < data.size(); i++)
	{
		if(data.at(i).sensorname.compare(name) == 0)
		{
			output.push_back(data.at(i));
		}
	}
	return output;
}
icarus_rover_v2::imu get_rossourcedata(std::string name,int index, std::vector<Source_Signal> data)
{
	icarus_rover_v2::imu output;
	output.tov = data.at(0).timestamp.at(index);
	for(std::size_t i = 0; i < data.size(); i++)
	{
		std::size_t found_xacc = std::string(data.at(i).name).find("xacc");
		std::size_t found_yacc = std::string(data.at(i).name).find("yacc");
		std::size_t found_zacc = std::string(data.at(i).name).find("zacc");
		std::size_t found_xgyro = std::string(data.at(i).name).find("xgyro");
		std::size_t found_ygyro = std::string(data.at(i).name).find("ygyro");
		std::size_t found_zgyro = std::string(data.at(i).name).find("zgyro");
		std::size_t found_xmag = std::string(data.at(i).name).find("xmag");
		std::size_t found_ymag = std::string(data.at(i).name).find("ymag");
		std::size_t found_zmag = std::string(data.at(i).name).find("zmag");
		icarus_rover_v2::signal signal;
		signal.value = data.at(i).value.at(index);
		signal.rms = data.at(i).rms.at(index);
		signal.status = data.at(i).status.at(index);

		if(found_xacc != std::string::npos)
		{
			output.xacc = signal;
		}
		else if(found_yacc != std::string::npos)
		{
			output.yacc = signal;
		}
		else if(found_zacc != std::string::npos)
		{
			output.zacc = signal;
		}
		else if(found_xgyro != std::string::npos)
		{
			output.xgyro = signal;
		}
		else if(found_ygyro != std::string::npos)
		{
			output.ygyro = signal;
		}
		else if(found_zgyro != std::string::npos)
		{
			output.zgyro = signal;
		}
		else if(found_xmag != std::string::npos)
		{
			output.xmag = signal;
		}
		else if(found_ymag != std::string::npos)
		{
			output.ymag = signal;
		}
		else if(found_zmag != std::string::npos)
		{
			output.zmag = signal;
		}

	}
	return output;
}
double measure_time_diff(struct timeval a, struct timeval b)
{
	double t1 = a.tv_sec + (double)a.tv_usec/1000000.0;
	double t2 = b.tv_sec + (double)b.tv_usec/1000000.0;
	return t1-t2;
}
