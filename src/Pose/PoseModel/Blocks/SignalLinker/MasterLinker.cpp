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
	diagnostic = splitter.new_input(signal_data);
	return diagnostic;
}
