/*
 * LinearAccelerationLinker.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_LINEACCELERATIONLINKER_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_LINEACCELERATIONLINKER_H_
#include "../../../../../include/eROS_Definitions.h"
#include "../../Definitions/PoseDefinitions.h"
#include "eros/diagnostic.h"
#include "eros/signal.h"
#include "BaseLinker.h"
class LinearAccelerationLinker: public BaseLinker {
public:
	LinearAccelerationLinker();
	virtual ~LinearAccelerationLinker();
	//Initialization Functions
	void init(eros::diagnostic _diagnostic)
	{
		diagnostic = _diagnostic;
	}
	//Attribute Functions
	void reset()
	{

	}
	//Message Functions
	eros::diagnostic new_input(std::vector<SplitSignal> inputs);
	eros::diagnostic new_input(__attribute__((unused)) std::vector<std::vector<InputSignal_3d> > inputs)
	{
		eros::diagnostic diag = diagnostic;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "LinearAccelerationLinker: Function is not available or not implemented yet.";
		diagnostic = diag;
		return diag;
	}

private:
	eros::diagnostic update_input(std::vector<SplitSignal> inputs);
	eros::diagnostic update_input(__attribute__((unused)) std::vector<std::vector<InputSignal_3d> > inputs)
	{
		eros::diagnostic diag = diagnostic;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "LinearAccelerationLinker: Function is not available or not implemented yet.";
		diagnostic = diag;
		return diag;
	}
	eros::diagnostic diagnostic;
	bool initialized;
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_LINEACCELERATIONLINKER_H_ */
