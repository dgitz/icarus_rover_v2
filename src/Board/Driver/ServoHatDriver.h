//Source: https://github.com/4ndr3w/PiBot/blob/e81e369494136ceea5d1d556b484e47e470a2e11/drivers/PWM.h
#ifndef ServoHatDriver_h
#define ServoHatDriver_h

#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>

class ServoHatDriver
{
public:
	enum ServoHatDriverConstant {
		__MODE1              = 0x00,
		__MODE2              = 0x01,
		__SUBADR1            = 0x02,
		__SUBADR2            = 0x03,
		__SUBADR3            = 0x04,
		__PRESCALE           = 0xFE,
		__LED0_ON_L          = 0x06,
		__LED0_ON_H          = 0x07,
		__LED0_OFF_L         = 0x08,
		__LED0_OFF_H         = 0x09,
		__ALL_LED_ON_L       = 0xFA,
		__ALL_LED_ON_H       = 0xFB,
		__ALL_LED_OFF_L      = 0xFC,
		__ALL_LED_OFF_H      = 0xFD,


		__RESTART            = 0x80,
		__SLEEP              = 0x10,


		__ALLCALL            = 0x01,
		__INVRT              = 0x10,
		__OUTDRV             = 0x04,
	};

	ServoHatDriver();
	~ServoHatDriver();
	void init(int address = 0x40);
	void setPWMFreq(int freq);
	void setPWM(int channel, int on, int off);
	void resetAllPWM(int on, int off);
	void setServoValue(int channel, int v);
	void resetAllServo();
	int get_address() { return address; }
private:
	int address;
	int ServoHatfd;
};


#endif
