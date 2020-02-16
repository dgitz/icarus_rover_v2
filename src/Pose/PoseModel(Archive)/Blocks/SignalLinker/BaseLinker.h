/*
 * BaseLinker.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_BASELINKER_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_BASELINKER_H_
#include "../../../../../include/eROS_Definitions.h"
#include "../../Definitions/PoseDefinitions.h"
#include "eros/diagnostic.h"
#include "eros/signal.h"
class BaseLinker {
public:
	BaseLinker();
	virtual ~BaseLinker();
	//Initialization Functions
	virtual void init(eros::diagnostic _diagnostic) = 0;
	//Attribute Functions
	virtual void reset() = 0;
	std::vector<InputSignal_3d> get_outputs() { return outputs; }
	//Message Functions
	virtual eros::diagnostic new_input(std::vector<SplitSignal> inputs) = 0;
	virtual eros::diagnostic new_input(std::vector<std::vector<InputSignal_3d> > inputs) = 0;
protected:
	virtual eros::diagnostic update_input(std::vector<SplitSignal> inputs) = 0;
	virtual eros::diagnostic update_input(std::vector<std::vector<InputSignal_3d> > inputs) = 0;
	eros::diagnostic diagnostic;
	bool initialized;
	uint16_t input_count;
	std::vector<InputSignal_3d> outputs;
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_SIGNALSPLITTER_H_ */