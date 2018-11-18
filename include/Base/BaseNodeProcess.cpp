#include "BaseNodeProcess.h"
bool BaseNodeProcess::update(double dt)
{
	run_time+=dt;
	return true;
}
void BaseNodeProcess::print_runtime()
{
}
