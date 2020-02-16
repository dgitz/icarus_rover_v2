/*
 * BaseMachinePostProcess.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SENSORPOSTPROCESSING_BASEPOSTPROCESS_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SENSORPOSTPROCESSING_BASEPOSTPROCESS_H_
#include "../../Definitions/PoseDefinitions.h"
#include "../../../Development/DummyData/CreateDummyData.h"
class BasePostProcess {
public:
	BasePostProcess();
	virtual ~BasePostProcess();
	//Initialization Functions
	void init(std::string _name)
	{
		name = _name;
		initialized = true;
	}
	//Attribute Functions
	std::string get_name() { return name; }
	//Message Functions
	virtual PostProcessedSignal new_signal(TimedSignal input) = 0;
	//Support Functions
	//Printing Functions
protected:
	bool initialized;
	std::string name;
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SENSORPOSTPROCESSING_BASEPOSTPROCESS_H_ */
