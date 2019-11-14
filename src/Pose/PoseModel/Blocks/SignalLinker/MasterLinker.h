/*
 * MasterLinker.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_MASTERLINKER_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_MASTERLINKER_H_
#include "../../../../../include/eROS_Definitions.h"
#include "../../Definitions/PoseDefinitions.h"
#include "eros/diagnostic.h"
#include "eros/signal.h"
#include "SignalSplitter.h"
#include "LinearAccelerationLinker.h"
#include "RotationRateLinker.h"
#include "OrientationLinker.h"
class MasterLinker {
public:
	MasterLinker();
	virtual ~MasterLinker();
	//Initialization Functions
	bool initialize_object(std::string _name,eros::diagnostic _diagnostic)
	{
		name = _name;
		diagnostic = _diagnostic;
		splitter.init(diagnostic);
		linearacceleration_linker.init(diagnostic);
		rotationrate_linker.init(diagnostic);
		return true;
	}
	eros::diagnostic new_input(std::vector<PostProcessedSignal> signal_data);
	std::vector<InputSignal_3d> get_linearaccelerations() { return linearacceleration_linker.get_outputs(); }
	std::vector<InputSignal_3d> get_rotationrates() { return rotationrate_linker.get_outputs(); }
	std::vector<InputSignal_3d> get_orientations() { return orientation_linker.get_outputs(); }
protected:
	std::string name;
	eros::diagnostic diagnostic;
	SignalSplitter splitter;
	LinearAccelerationLinker linearacceleration_linker;
	RotationRateLinker rotationrate_linker;
	OrientationLinker orientation_linker;
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_MASTERLINKER_H_ */
