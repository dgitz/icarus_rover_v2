#include "SampleDriver.h"
SampleDriver::SampleDriver()
{
	name = "";
}
SampleDriver::~SampleDriver()
{
}
int SampleDriver::init(std::string t_name)
{
	name = t_name;
	return 1;
}