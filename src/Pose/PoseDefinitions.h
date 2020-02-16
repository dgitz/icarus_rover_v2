#ifndef POSEDEFINITIONS_H
#define POSEDEFINITIONS_H
#include <stdio.h>
#include <iostream>
#include "../../../eROS/include/DiagnosticClass.h"
#include "../../include/Definitions.h"
#include <eros/signal.h>
#define TIMEDSIGNAL_ACCEL1X_INDEX 0
#define TIMEDSIGNAL_ACCEL1Y_INDEX 1
#define TIMEDSIGNAL_ACCEL1Z_INDEX 2
struct SensorSignal
{
	uint64_t sequence_number;
	eros::signal signal;
	uint8_t signal_class;
};
struct PostProcessedSignal
{
	eros::signal signal;
	uint8_t signal_class;
};


struct SplitSignal
{
	std::string instance_name;
	eros::signal x;
	eros::signal y;
	eros::signal z;
	uint64_t x_update_count;
	uint64_t y_update_count;
	uint64_t z_update_count;
};
struct InputSignal_3d
{
	std::string instance_name;
	eros::signal x;
	eros::signal y;
	eros::signal z;
	uint64_t x_update_count;
	uint64_t y_update_count;
	uint64_t z_update_count;
};
/*struct PoseDiagnostic
{

};
struct PoseState
{
	PoseDiagnostic pose_diagnostic;
};
struct PoseAcceleration
{
	PoseDiagnostic pose_diagnostic;
	eros::signal xacc;
	eros::signal yacc;
	eros::signal zacc;
};
struct PoseRotationRate
{
	PoseDiagnostic pose_diagnostic;
};
struct PoseVelocity
{
	PoseDiagnostic pose_diagnostic;
};
struct PoseOrientation
{
	PoseDiagnostic pose_diagnostic;
};
struct PosePosition
{
	PoseDiagnostic pose_diagnostic;
};
struct SensorLinearAcceleration
{
	std::vector<eros::signal> xacc;
	std::vector<eros::signal> yacc;
	std::vector<eros::signal> zacc;
};
struct BasicLinearVelocity
{

};
struct BasicRotationRate
{

};
struct BasicOrientation
{

};
struct BasicPosition
{

};
struct BaseMachine
{

};
*/

#endif