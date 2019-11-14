/*
 * MasterLinker.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#include "MasterLinker.h"
MasterLinker::MasterLinker()
{

}
MasterLinker::~MasterLinker()
{

}

eros::diagnostic MasterLinker::new_input(std::vector<PostProcessedSignal> signal_data)
{
	{
		diagnostic = splitter.new_input(signal_data);
	}
	{
		diagnostic = linearacceleration_linker.new_input(splitter.get_split_linearaccelerations());
	}
	{
		diagnostic = rotationrate_linker.new_input(splitter.get_split_rotationratates());
	}
	{
		std::vector<std::vector<InputSignal_3d> > inputs;
		std::vector<InputSignal_3d> linear_accels = get_linearaccelerations(); //PIPELINE: LINEARACCELERATION
		inputs.push_back(linear_accels);
	  	diagnostic = orientation_linker.new_input(inputs);
	}
	return diagnostic;
}

