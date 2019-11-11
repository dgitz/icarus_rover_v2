/*
 * SignalSplitter.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_SIGNALSPLITTER_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_SIGNALSPLITTER_H_
#include "../../../../../include/eROS_Definitions.h"
#include "../../Definitions/PoseDefinitions.h"
#include "eros/diagnostic.h"
#include "eros/signal.h"
class SignalSplitter {
public:
	SignalSplitter();
	~SignalSplitter();
	//Initialization Functions
	void init(eros::diagnostic _diagnostic)
	{
		diagnostic = _diagnostic;
	}
	//Message Functions
	eros::diagnostic new_input(std::vector<PostProcessedSignal> signals);
	//Attribute Functions
	std::vector<LinkedSensor_Acceleration> get_linked_accelerations() { return linked_accelerations; }
	std::vector<LinkedSensor_RotationRate> get_linked_rotationratates() { return linked_rotationrates; }
	std::vector<LinkedSensor_MagneticField> get_linked_magneticfields() { return linked_magneticfields; }
	//Printing functions
	void print_splitsignals();
private:
	eros::diagnostic init_signals(std::vector<PostProcessedSignal> signals);
	eros::diagnostic update_input(std::vector<PostProcessedSignal> signals);
	eros::diagnostic diagnostic;
	bool initialized;
	std::vector<LinkedSensor_Acceleration> linked_accelerations;
	std::vector<LinkedSensor_RotationRate> linked_rotationrates;
	std::vector<LinkedSensor_MagneticField> linked_magneticfields;
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_SIGNALSPLITTER_H_ */
