/*
 * PoseHelper.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#include "PoseHelper.h"

PoseHelper::PoseHelper() {
	// TODO Auto-generated constructor stub
}

PoseHelper::~PoseHelper() {
	// TODO Auto-generated destructor stub
}
MatrixXf PoseHelper::compute_variance_covariance_matrix(std::vector<std::vector<double> > input_data)
{

	MatrixXf R;
	uint8_t input_count = input_data.size();
	uint64_t sample_count = 0;
	for(std::size_t i = 0; i < input_data.size(); ++i)
	{
		if(sample_count == 0)
		{
			sample_count = input_data.at(i).size();
		}
		else
		{
			if(sample_count != input_data.at(i).size())
			{
				return R;
			}
		}
	}
	MatrixXf A(sample_count,input_count);
	for(uint64_t r = 0; r < sample_count; ++r)
	{
		for(uint8_t c = 0; c < input_count; ++c)
		{
			A(r,c) = input_data.at(c).at(r);
		}
	}
	R.resize(input_count,input_count);
	MatrixXf ones = MatrixXf::Ones(sample_count,sample_count);
	MatrixXf a(sample_count,input_count);
	a = A-ones*A/(double)(sample_count);
	R = a.transpose()*a/(double)(sample_count);
	return R;
}
