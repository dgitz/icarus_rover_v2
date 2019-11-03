struct PoseDiagnostic
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
