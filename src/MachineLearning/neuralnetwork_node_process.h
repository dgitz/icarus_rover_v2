#ifndef NEURALNETWORKNODEPROCESS_H
#define NEURALNETWORKNODEPROCESS_H
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
#include <sys/time.h>
#include <tinyxml.h>
#include "NeuralNetwork.h"

using std::string;
using namespace std;
class NeuralNetworkNodeProcess
{
public:
	NeuralNetworkNodeProcess();
	~NeuralNetworkNodeProcess();    
protected:
private:

};
#endif
