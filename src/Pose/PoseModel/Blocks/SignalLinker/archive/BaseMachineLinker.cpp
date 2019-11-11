/*
 * BaseMachineLinker.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#include "BaseMachineLinker.h"

BaseMachineLinker::BaseMachineLinker() {
	current_time=0.0;
	ros_time=0.0;

}
eros::diagnostic BaseMachineLinker::new_inputsignal(eros::signal sig)
{
	eros::diagnostic diag = diagnostic;
	bool found = false;
	for(std::size_t i = 0; i < input_signals.size(); ++i)
	{
		if(sig.name == input_signals.at(i).name)
		{
			input_signals.at(i) = sig;
			found = true;
		}
	}
	if(found == true)
	{
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "SignalLinker: " + name + " updated input signal: " + sig.name;
	}
	else
	{
		diag.Level = ERROR;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		diag.Description = "SignalLinker: " + name + " unable to update input signal: " + sig.name;
	}
	return diag;
	
}
BaseMachineLinker::~BaseMachineLinker() {
	// TODO Auto-generated destructor stub
}
eros::diagnostic BaseMachineLinker::base_update(double dt, double _ros_time)
{
	eros::diagnostic diag = diagnostic;
	current_time+=dt;
	ros_time=_ros_time;
	diagnostic = diag;
	return diag;
}
void BaseMachineLinker::print_inputs()
{
	printf("--- Signal Linker: %s INPUTS (Size=%d) ---\n",name.c_str(),(int)input_signals.size());
	for(std::size_t i = 0; i < input_signals.size(); ++i)
	{
		printf("\t[%d/%d] Signal: %s\n",(int)(i+1),(int)input_signals.size(),input_signals.at(i).name.c_str());
	}
	printf("--- End Signal Linker INPUTS ---\n");
}
void BaseMachineLinker::print_outputs()
{
	printf("--- Signal Linker: %s OUTPUTS (Size=%d) ---\n",name.c_str(),(int)output_signals.size());
	for(std::size_t i = 0; i < output_signals.size(); ++i)
	{
		printf("\t[%d/%d] Signal: %s\n",(int)(i+1),(int)output_signals.size(),output_signals.at(i).name.c_str());
	}
	printf("--- End Signal Linker OUTPUTS ---\n");
}