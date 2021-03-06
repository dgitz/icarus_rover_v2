#include "ServoHatDriver.h"
ServoHatDriver::ServoHatDriver()
{
	address = -1;
	ServoHatfd = -1;
}
ServoHatDriver::~ServoHatDriver()
{

}


int ServoHatDriver::init(int address_)
{
    init_pinmap();
	address = address_;
    ServoHatfd = wiringPiI2CSetup(address);


    // zero all PWM ports
    resetAllPWM(0,0);

    wiringPiI2CWriteReg8(ServoHatfd, __MODE2, __OUTDRV);
    wiringPiI2CWriteReg8(ServoHatfd, __MODE1, __ALLCALL);

    int mode1 = wiringPiI2CReadReg8(ServoHatfd, __MODE1);
    //printf("model: %d\n",model);
    mode1 = mode1 & ~__SLEEP;
    wiringPiI2CWriteReg8(ServoHatfd, __MODE1, mode1);

    setPWMFreq(60);
    return mode1;
}

void ServoHatDriver::setPWMFreq(int freq)
{
    float prescaleval = 25000000;
    prescaleval /= 4096.0;
    prescaleval /= (float)freq;
    prescaleval -= 1.0;
    int prescale = floor(prescaleval + 0.5);

    int oldmode = wiringPiI2CReadReg8(ServoHatfd, __MODE1);
    int newmode = (oldmode & 0x7F) | 0x10;
    wiringPiI2CWriteReg8(ServoHatfd, __MODE1, newmode);
    wiringPiI2CWriteReg8(ServoHatfd, __PRESCALE, floor(prescale));
    wiringPiI2CWriteReg8(ServoHatfd, __MODE1, oldmode);

    wiringPiI2CWriteReg8(ServoHatfd, __MODE1, oldmode | 0x80);
}
void ServoHatDriver::setServoValue(int channel, int v)
{
    int on = 0;
    int off = v/3.90;
    setPWM(channel,on,off);
}
void ServoHatDriver::resetAllServo()
{
    
}
void ServoHatDriver::setPWM(int channel, int on, int off)
{
    wiringPiI2CWriteReg8(ServoHatfd, __LED0_ON_L+4*channel, on & 0xFF);
    wiringPiI2CWriteReg8(ServoHatfd, __LED0_ON_H+4*channel, on >> 8);
    wiringPiI2CWriteReg8(ServoHatfd, __LED0_OFF_L+4*channel, off & 0xFF);
    wiringPiI2CWriteReg8(ServoHatfd, __LED0_OFF_H+4*channel, off >> 8);
}

void ServoHatDriver::resetAllPWM(int on, int off)
{
    wiringPiI2CWriteReg8(ServoHatfd, __ALL_LED_ON_L, on & 0xFF);
    wiringPiI2CWriteReg8(ServoHatfd, __ALL_LED_ON_H, on >> 8);
    wiringPiI2CWriteReg8(ServoHatfd, __ALL_LED_OFF_L, off & 0xFF);
    wiringPiI2CWriteReg8(ServoHatfd, __ALL_LED_OFF_H, off >> 8);
}
int ServoHatDriver::map_channelname_topin(std::string name)
{
    if(PinMap.find(name) != PinMap.end())
    {
        return PinMap[name];
    }
    else
    {
        return PinMap["UNKNOWN"];
    }
    /*
    if(name == "") { return -1; }
    else if(name == "CH0") { return 0; }
    else if(name == "CH1") { return 1; }
    else if(name == "CH2") { return 2; }
    else if(name == "CH3") { return 3; }
    else if(name == "CH4") { return 4; }
    else if(name == "CH5") { return 5; }
    else if(name == "CH6") { return 6; }
    else if(name == "CH7") { return 7; }
    else if(name == "CH8") { return 8; }
    else if(name == "CH9") { return 9; }
    else if(name == "CH10") { return 10; }
    else if(name == "CH11") { return 11; }
    else if(name == "CH12") { return 12; }
    else if(name == "CH13") { return 13; }
    else if(name == "CH14") { return 14; }
    else if(name == "CH15") { return 15; }
    else { return -1; }
    */
}
void ServoHatDriver::init_pinmap()
{
    PinMap["CH0"] = 0;
    PinMap["CH1"] = 1;
    PinMap["CH2"] = 2;
    PinMap["CH3"] = 3;
    PinMap["CH4"] = 4;
    PinMap["CH5"] = 5;
    PinMap["CH6"] = 6;
    PinMap["CH7"] = 7;
    PinMap["CH8"] = 8;
    PinMap["CH9"] = 9;
    PinMap["CH10"] = 10;
    PinMap["CH11"] = 11;
    PinMap["CH12"] = 12;
    PinMap["CH13"] = 13;
    PinMap["CH14"] = 14;
    PinMap["CH15"] = 15;
    PinMap["UNKNOWN"] = -1;

}