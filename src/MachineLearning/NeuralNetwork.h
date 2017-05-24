#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H
#include "Definitions.h"
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/pin.h"
#include "icarus_rover_v2/firmware.h"
#include <std_msgs/UInt8.h>
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#include <sys/stat.h>
#include <tinyxml.h>
#include <numeric>  
#include <sys/time.h>

using std::string;
using namespace std;
struct Neuron
{
    int inputcount;
    std::vector<double> input;
    std::vector<double> weight;
    double output;
};
struct Layer
{
    std::vector<Neuron> neurons;
    std::string name;
    int neuroncount;
};

class NeuralNetwork
{
public:
    NeuralNetwork(icarus_rover_v2::diagnostic diag);
    //NeuralNetwork();
	~NeuralNetwork(); 
    icarus_rover_v2::diagnostic loadNetworkfile(std::string name);
    std::vector<double> evaluate_network(std::vector<double> input);
    std::vector<std::vector<double> > get_sample_inputdata() { return sample_inputdata; }
    std::vector<std::vector<double> > get_sample_outputdata() { return sample_outputdata; }


protected:
private:
    bool parse_Networkfile(TiXmlDocument doc);
    std::vector<Layer> layers;
    icarus_rover_v2::diagnostic diagnostic;
    std::string name;
    std::string type;
    std::string activation_function;
    int layercount;
    std::vector<std::vector<double> > sample_inputdata;
    std::vector<std::vector<double> > sample_outputdata;

};
#endif
