/*
 * IMUPostProcess.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SENSORPOSTPROCESSING_IMUPOSTPROCESS_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SENSORPOSTPROCESSING_IMUPOSTPROCESS_H_
#include "BasePostProcess.h"
class IMUPostProcess: public BasePostProcess {
public:
	IMUPostProcess();
	virtual ~IMUPostProcess();
	//Initialization Functions
	//Attribute Functions
	//Message Functions
	PostProcessedSignal new_signal(TimedSignal input);
	//Support Functions
	//Printing Functions
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_SENSORPOSTPROCESSING_IMUPOSTPROCESS_H_ */
