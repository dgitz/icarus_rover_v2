/*
 * LinearAccelerationLinker.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#include "LinearAccelerationLinker.h"

LinearAccelerationLinker::LinearAccelerationLinker() {
	// TODO Auto-generated constructor stub

}

LinearAccelerationLinker::~LinearAccelerationLinker() {
	// TODO Auto-generated destructor stub
}
bool LinearAccelerationLinker::initialize_object(std::string _name,eros::diagnostic diag)
{
	diagnostic = diag;
	name = _name;
	return true;
}
eros::diagnostic LinearAccelerationLinker::initialize_inputsignals(std::vector<eros::signal> inputs)
{
	eros::diagnostic diag = diagnostic;
	for(std::size_t i = 0; i < inputs.size(); ++i)
	{
		input_signals.push_back(inputs.at(i));
	}
	diagnostic = diag;
	return diagnostic;
}
eros::diagnostic LinearAccelerationLinker::initialize_outputsignals(std::vector<eros::signal> outputs)
{
	eros::diagnostic diag = diagnostic;
	for(std::size_t i = 0; i < outputs.size(); ++i)
	{
		output_signals.push_back(outputs.at(i));
	}
	diagnostic = diag;
	return diagnostic;
}
eros::diagnostic LinearAccelerationLinker::update(double dt, double _ros_time)
{
	eros::diagnostic diag = base_update(dt,_ros_time);
	for(std::size_t i = 0; i < output_signals.size(); ++i)
	{
		eros::signal out = input_signals.at(i);
		out.tov = ros_time;
		output_signals.at(i) = out;
	}
	diagnostic = diag;
	return diagnostic;
}