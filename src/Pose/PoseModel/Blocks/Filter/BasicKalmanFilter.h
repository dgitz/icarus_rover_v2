/*
 * BasicKalmanFilter.h
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
#include "Definitions.h"

using namespace Eigen;
#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_FILTER_BASICKALMANFILTER_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_FILTER_BASICKALMANFILTER_H_

class BasicKalmanFilter {
public:

	struct FilterParameters
	{
		VectorXf xhat;
		MatrixXf R;
		VectorXf C;
		MatrixXf Q;
		MatrixXf P;
		MatrixXf I;
		MatrixXf G;
		VectorXf z;
	};
	BasicKalmanFilter();
	virtual ~BasicKalmanFilter();
	//Initialization Functions
	/*! \brief Filter Initialization
	 *
	 */
	eros::diagnostic initialize(eros::diagnostic _diagnostic,
			uint8_t _input_count,
			uint8_t _output_count,
			double _initial_state,
			MatrixXf _InputSignal_VarianceCovariance_Matrix,
			VectorXf _InputSignal_Correlation_Vector,
			double _Q);
	//Attribute Functions
	/*! \brief Get Filter Parameters
	 *
	 */
	FilterParameters get_filter() { return KF; }
	//Message Functions
	//Support Functions
	/*! \brief Update Filter given input vector and input weights
	 *
	 */
	double update(VectorXf input,VectorXf input_weight);
	//Printing Functions
private:
	eros::diagnostic diagnostic;
	double output;
	uint8_t input_count;
	uint8_t output_count;
	FilterParameters KF;
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_FILTER_BASICKALMANFILTER_H_ */
