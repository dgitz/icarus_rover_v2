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

	diagnostic = diag;
	return diagnostic;
}
eros::diagnostic LinearAccelerationLinker::initialize_outputsignals(std::vector<eros::signal> outputs)
{
	eros::diagnostic diag = diagnostic;
	
	diagnostic = diag;
	return diagnostic;
}

