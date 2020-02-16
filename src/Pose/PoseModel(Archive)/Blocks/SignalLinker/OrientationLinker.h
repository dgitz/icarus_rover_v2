/*
 * OrientationLinker.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_ORIENTATIONLINKER_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_ORIENTATIONLINKER_H_
#include "../../../../../include/eROS_Definitions.h"
#include "../../Definitions/PoseDefinitions.h"
#include "eros/diagnostic.h"
#include "eros/signal.h"
#include "BaseLinker.h"
class OrientationLinker: public BaseLinker {
public:
	OrientationLinker();
	virtual ~OrientationLinker();
	const double LINEARACC_MU = 0.01;
	enum class Pipeline 
	{
		LINEARACCELERATION=0,
		ROTATIONRATE=1,
		MAGNETICFIELD=2,
	};
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
	eros::diagnostic new_input(__attribute__((unused)) std::vector<SplitSignal> inputs)
	{
		eros::diagnostic diag = diagnostic;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "OrientationLinker: Function is not available or not implemented yet.";
		diagnostic = diag;
		return diag;
	}
	eros::diagnostic new_input(std::vector<std::vector<InputSignal_3d> > inputs);


private:
	eros::diagnostic update_input(__attribute__((unused)) std::vector<SplitSignal> inputs)
	{
		eros::diagnostic diag = diagnostic;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "RotationRateLinker: Function is not available or not implemented yet.";
		diagnostic = diag;
		return diag;
	}
	eros::diagnostic update_input(std::vector<std::vector<InputSignal_3d> > inputs);
	
	
	eros::diagnostic diagnostic;
	bool initialized;
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SIGNALLINKER_LINEACCELERATIONLINKER_H_ */
