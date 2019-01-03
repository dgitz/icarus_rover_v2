/*
 * PoseAccelerationBlock.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_POSEBLOCKS_POSEACCELERATIONBLOCK_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_POSEBLOCKS_POSEACCELERATIONBLOCK_H_
#include "BasePoseBlock.cpp"
class PoseAccelerationBlock: public BasePoseBlock {
public:
	PoseAccelerationBlock();
	virtual ~PoseAccelerationBlock();
	//Initialization Functions
	//Attribute Functions
	PoseAcceleration get_output() { return pose_acc; }
	//Message Functions

	//Support Functions
	void update();
	//Printing Functions
private:
	PoseAcceleration pose_acc;
	BasicLinearAcceleration acc_input;
	PoseState pose_state;
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_POSEBLOCKS_POSEACCELERATIONBLOCK_H_ */
