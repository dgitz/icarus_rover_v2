/*
 * BasicKalmanFilter.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#include "BasicKalmanFilter.h"

BasicKalmanFilter::BasicKalmanFilter() {
	// TODO Auto-generated constructor stub
	output = 0.0;
}

BasicKalmanFilter::~BasicKalmanFilter() {
	// TODO Auto-generated destructor stub
}
eros::diagnostic BasicKalmanFilter::initialize(eros::diagnostic _diagnostic,
											uint8_t _input_count,
											uint8_t _output_count,
											double _initial_state,
											MatrixXf _InputSignal_VarianceCovariance_Matrix,
											VectorXf _InputSignal_Correlation_Vector,
											double _Q)
{
	eros::diagnostic diag = _diagnostic;
	input_count = _input_count;
	output_count = _output_count;
	VectorXf initial_state;
	initial_state.resize(1);
	initial_state(0) = _initial_state;
	KF.xhat = initial_state;
	KF.R = _InputSignal_VarianceCovariance_Matrix;
	KF.Q.resize(1,1);
	KF.Q(0,0) = _Q;
	KF.C = _InputSignal_Correlation_Vector;
	KF.P.resize(1,1);
	KF.P(0,0) = 1.0;
	KF.I = MatrixXf::Identity(output_count,output_count);
	KF.z.resize(input_count);
	KF.G.resize(1,input_count);
	return diag;
}
double BasicKalmanFilter::update(VectorXf input,VectorXf w)
{
	MatrixXf Rp = KF.R;
	for(int i = 0; i < w.size(); ++i)
	{
		Rp(i,i) = Rp(i,i)/w(i);
	}
	//Predict
	KF.xhat = KF.xhat; //Notation
	KF.P = KF.P + KF.Q;

	//Update

	MatrixXf T1 = (KF.C*KF.P*KF.C.transpose()+Rp);
	KF.G = KF.P*KF.C.transpose()*T1.inverse();
	KF.P = (KF.I-(KF.G*KF.C))*KF.P;
	KF.xhat = KF.xhat + KF.G*(input-(KF.C*KF.xhat));
	output = KF.xhat(0);
	return output;
}
