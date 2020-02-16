/*
 * BasePoseBlock.h
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_POSEBLOCKS_BASEPOSEBLOCK_H_
#define SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_POSEBLOCKS_BASEPOSEBLOCK_H_
#include "../../Definitions/PoseDefinitions.h"
#include "../Filter/BasicKalmanFilter.h"
class BasePoseBlock {
public:
	BasePoseBlock();
	virtual ~BasePoseBlock();

private:
	virtual void update() = 0;
	BasicKalmanFilter filter;
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_POSE_POSEMODEL_BLOCKS_POSEBLOCKS_BASEPOSEBLOCK_H_ */
