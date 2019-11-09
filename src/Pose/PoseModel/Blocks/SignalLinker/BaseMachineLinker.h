/*
 * BaseMachineLinker.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_BASEMACHINELINKER_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_BASEMACHINELINKER_H_
#include "../../../../../include/eROS_Definitions.h"
#include "eros/diagnostic.h"
#include "eros/signal.h"
class BaseMachineLinker {
public:
	BaseMachineLinker();
	virtual ~BaseMachineLinker();
	//Initialization Functions
	virtual bool initialize_object(std::string _name,eros::diagnostic diag) = 0;
	/*
	Input Signals are expected to be time compensated
	*/
	virtual eros::diagnostic initialize_inputsignals(std::vector<eros::signal> inputs) = 0;
	virtual eros::diagnostic initialize_outputsignals(std::vector<eros::signal> outputs) = 0;
	//Attribute Functions
	std::vector<eros::signal> get_inputsignals() { return input_signals; }
	std::vector<eros::signal> get_outputsignals() { return output_signals; }
	double get_currenttime() { return current_time; }
	double get_rostime() { return ros_time; }
	//Message Functions
	eros::diagnostic new_inputsignal(eros::signal sig); 
	//Support Functions
	virtual eros::diagnostic update(double dt, double ros_time) = 0;
	eros::diagnostic base_update(double dt,double _ros_time);
	//Printing Functions
	void print_inputs();
	void print_outputs();
protected:
	std::string name;
	std::vector<eros::signal> input_signals;
	std::vector<eros::signal> output_signals;
	eros::diagnostic diagnostic;
	double current_time;
	double ros_time;
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_BASEMACHINELINKER_H_ */
