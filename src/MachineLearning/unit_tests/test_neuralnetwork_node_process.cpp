#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../NeuralNetwork.h"
#include "../neuralnetwork_node_process.h"

std::string Node_Name = "/unittest_neuralnetwork_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ComputeModule";
TEST(NeuralNetwork,NeuralNetwork_Initialization_1Net)
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
    
	NeuralNetwork net(diagnostic);
    diagnostic = net.loadNetworkfile("Simple2");
    EXPECT_TRUE(diagnostic.Level > NOTICE); //This file does not exist, and it shouldn't;
    diagnostic = net.loadNetworkfile("Simple1");
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    std::vector<std::vector<double> > sampleinput_array = net.get_sample_inputdata();
    std::vector<std::vector<double> > sampleoutput_array = net.get_sample_outputdata();
    EXPECT_TRUE(sampleinput_array.size() > 0);
    for(int i = 0; i < sampleinput_array.size(); i++)
    {
        EXPECT_TRUE(sampleinput_array.at(i).size() > 0);
    }
    
    EXPECT_TRUE(sampleoutput_array.size() > 0);
    for(int i = 0; i < sampleoutput_array.size(); i++)
    {
        EXPECT_TRUE(sampleoutput_array.at(i).size() > 0);
    }
    
    EXPECT_TRUE(sampleinput_array.size() == sampleoutput_array.size());
    for(int i = 0; i < sampleinput_array.size(); i++)
    {
        std::vector<double> input = sampleinput_array.at(i);
        std::vector<double> expected_output = sampleoutput_array.at(i);
        std::vector<double> output = net.evaluate_network(input);
        for(int j = 0; j < output.size(); j++)
        {
            double error = fabs(output.at(j)-expected_output.at(j));
            EXPECT_TRUE(error < .0001);
        }
    }
    
    
    
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
