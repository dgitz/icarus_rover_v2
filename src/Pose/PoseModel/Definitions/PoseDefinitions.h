#ifndef POSEDEFINITIONS_H
#define POSEDEFINITIONS_H
#include <stdio.h>
#include <iostream>
#include "Definitions.h"
#include <eros/signal.h>
struct SensorSignal
{
	uint64_t sequence_number;
	eros::signal signal;
	uint8_t signal_class;
};
struct TimedSignal
{
	eros::signal signal;
	uint8_t signal_class;
};
struct PostProcessedSignal
{
	eros::signal signal;
	uint8_t signal_class;
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