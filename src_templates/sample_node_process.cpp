#include "sample_node_process.h"
SampleNodeProcess::SampleNodeProcess()
{
}
SampleNodeProcess::~SampleNodeProcess()
{

}
icarus_rover_v2::diagnostic SampleNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
    diagnostic = indiag;
	return diagnostic;
}
icarus_rover_v2::diagnostic SampleNodeProcess::update(double dt)
{
	return diagnostic;
}