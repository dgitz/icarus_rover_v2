#ifndef VISIONHELPER_H
#define VISIONHELPER_H

#include "ros/ros.h"
#include "Definitions.h"
#include "ros/time.h"
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>

using std::string;
using namespace std;
class VisionHelper
{
public:
	VisionHelper();
    ~VisionHelper();
    cv::Mat Detect_Edges(cv::Mat gray_image,int threshold);
private:
};
#endif
