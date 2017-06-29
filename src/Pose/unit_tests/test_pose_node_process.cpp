#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include <sys/time.h>
#include <math.h>
#include "../pose_node_process.h"
#include <Eigen/Dense>
#define EPS 0.0000001
using namespace Eigen;
std::string Node_Name = "/unittest_pose_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;



PoseNodeProcess *initialized_process;


bool check_if_initialized(PoseNodeProcess process);
double measure_timediff(struct timeval a, struct timeval b);
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
TEST(KalmanFilter,ExampleFilter)
{
    icarus_rover_v2::diagnostic diagnostic;
    PoseNodeProcess *process = initialized_process;
    EXPECT_TRUE(process->is_initialized());
    diagnostic = process->new_kalmanfilter("Example1", "Position", 2);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = process->new_kalmanfilter_signal("Example2",0.0);
    EXPECT_TRUE(diagnostic.Level > NOTICE);
    diagnostic = process->new_kalmanfilter_signal("Example1",0.0);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = process->set_kalmanfilter_properties("Example1", 3.0,0.1);
    double time_limit = 10.0;
    double dt = 0.01;
    double timer = 0.0;
    double signal_update_rate = 0.1;
    double next_signal_time = signal_update_rate;
    double signal = 0.0;
    while(timer <= time_limit)
    {
        
        if(timer >= next_signal_time) 
        {
            next_signal_time += signal_update_rate; 
            double noise = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
            signal = (noise-0.5)+sin(M_PI*timer);
            
            diagnostic = process->new_kalmanfilter_signal("Example1",signal);    
        }
        diagnostic = process->update(dt);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        timer += dt;
    }
    
}
TEST(KalmanFilter,PoseModel)
{
    icarus_rover_v2::diagnostic diagnostic;
    PoseNodeProcess *process = initialized_process;
    EXPECT_TRUE(process->is_initialized());
    diagnostic = process->set_kalmanfilter_properties("Yaw", 3.0,0.1);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = process->set_kalmanfilter_properties("Yawrate", 3.0,0.1);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    icarus_rover_v2::pose pose;
    pose = process->get_pose();
    EXPECT_TRUE(pose.yaw.status == SIGNALSTATE_INITIALIZING);
    EXPECT_TRUE(pose.yawrate.status == SIGNALSTATE_INITIALIZING);
    
    diagnostic = process->update(0.01);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    pose = process->get_pose();
    EXPECT_TRUE(pose.yaw.status == SIGNALSTATE_INITIALIZING);
    EXPECT_TRUE(pose.yawrate.status == SIGNALSTATE_INITIALIZING);
    
    diagnostic = process->new_kalmanfilter_signal("Yaw", 0.0);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = process->update(0.01);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    pose = process->get_pose();
    EXPECT_TRUE(pose.yaw.status == SIGNALSTATE_UPDATED);
    EXPECT_TRUE(pose.yawrate.status == SIGNALSTATE_INITIALIZING);
    
    diagnostic = process->new_kalmanfilter_signal("Yawrate", 0.0);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = process->update(0.01);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    pose = process->get_pose();
    EXPECT_TRUE(pose.yaw.status == SIGNALSTATE_UPDATED);
    EXPECT_TRUE(pose.yawrate.status == SIGNALSTATE_UPDATED);
    
}
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
