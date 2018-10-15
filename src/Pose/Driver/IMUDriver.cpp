/*
 * IMUDriver.cpp
 *
 *  Created on: Sep 26, 2018
 *      Author: robot
 */

#include "IMUDriver.h"

IMUDriver::IMUDriver() {
	// TODO Auto-generated constructor stub

}

IMUDriver::~IMUDriver() {
	// TODO Auto-generated destructor stub
}
std::string IMUDriver::get_connstring(int id)
{
	if(id == 0)
	{
		return "/dev/spidev0.0";
	}
	else if(id == 1)
	{
		return "/dev/spidev0.1";
	}
	else
	{
		return "";
	}
}
bool IMUDriver::init(long clock_rate,int ch1_,int ch2_)
{
	bool imu1_available = false;
	bool imu2_available = false;
	if((ch1_ < 0) || (ch1_ > 1))
	{
		imu1_available = false;
	}
	if((ch2_ < 0) || (ch2_ > 1))
	{
		imu2_available = false;
	}

	if((imu1_available == false) and (imu2_available == false))
	{
		return false;
	}
	if(imu1_available == true)
	{
		imu1.ch = ch1_;
		imu1.fd = wiringPiSPISetup(imu1.ch,clock_rate);
		imu1.id = read8(XMTYPE,LSM9DS0_REGISTER_WHO_AM_I_XM);

	}
	return false;
}

unsigned char IMUDriver::read8(bool type, unsigned char reg)
{
  uint8_t value;

  readBuffer(type, reg, 1, &value);

  return value;
}
