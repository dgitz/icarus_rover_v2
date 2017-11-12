#include "imu_node_process.h"
IMUNodeProcess::IMUNodeProcess()
{
}
IMUNodeProcess::~IMUNodeProcess()
{

}
icarus_rover_v2::diagnostic IMUNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
    diagnostic = indiag;
	return diagnostic;
}
icarus_rover_v2::diagnostic IMUNodeProcess::update(double dt)
{
	return diagnostic;
}
