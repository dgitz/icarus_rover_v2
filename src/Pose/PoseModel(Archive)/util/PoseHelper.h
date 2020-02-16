/*
 * PoseHelper.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */
//C System Files
//C++ System Files
#include <eigen3/Eigen/Dense>
//ROS Base Functionality
//ROS Messages
#include <eros/diagnostic.h>
//Project
#include "../Definitions/PoseDefinitions.h"

using namespace Eigen;
#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_UTIL_POSEHELPER_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_UTIL_POSEHELPER_H_

class PoseHelper {
public:

	PoseHelper();
	virtual ~PoseHelper();
	//Initialization Functions
	//Attribute Functions
	//Message Functions
	//Support Functions
	MatrixXf compute_variance_covariance_matrix(std::vector<std::vector<double> > input_data);
	//Printing Functions
private:
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_UTIL_POSEHELPER_H_ */
