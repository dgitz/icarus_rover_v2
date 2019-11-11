/*
 * MasterLinker.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_MASTERLINKER_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_MASTERLINKER_H_
#include "SignalSplitter.h"
#include "../../../../../include/eROS_Definitions.h"
#include "../../Definitions/PoseDefinitions.h"
#include "eros/diagnostic.h"
#include "eros/signal.h"
class MasterLinker {
public:
	MasterLinker();
	~MasterLinker();
	//Initialization Functions
	bool initialize_object(std::string _name,eros::diagnostic _diagnostic)
	{
		name = _name;
		diagnostic = _diagnostic;
		splitter.init(diagnostic);
		return true;
	}
	eros::diagnostic new_input(std::vector<PostProcessedSignal> signal_data);
protected:
	std::string name;
	eros::diagnostic diagnostic;
	SignalSplitter splitter;
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_MASTERLINKER_H_ */
