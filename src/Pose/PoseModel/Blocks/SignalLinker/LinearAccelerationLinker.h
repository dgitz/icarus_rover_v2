/*
 * LinearAccelerationLinker.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_LINEARACCELERATIONLINKER_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_LINEARACCELERATIONLINKER_H_
#include "BaseMachineLinker.cpp"
class LinearAccelerationLinker: public BaseMachineLinker {
public:
	LinearAccelerationLinker();
	virtual ~LinearAccelerationLinker();
	//Initialization Functions
	bool initialize_object(std::string _name,eros::diagnostic diag);
	/*
	For this linker, all inputs are expected to be of the same class (i.e. Linear Acceleration signals)
	*/
	eros::diagnostic initialize_inputsignals(std::vector<eros::signal> inputs);
	/*
	For this linker, it is expected that there is a 1-to-1 mapping between input and output signals
	*/
	eros::diagnostic initialize_outputsignals(std::vector<eros::signal> outputs);
	//Attribute Functions
	//Message Functions
	//Support Functions
	//Printing Functions
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_LINEARACCELERATIONLINKER_H_ */
