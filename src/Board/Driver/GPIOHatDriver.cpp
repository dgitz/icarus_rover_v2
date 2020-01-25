#include "GPIOHatDriver.h"
GPIOHatDriver::GPIOHatDriver()
{
	address = -1;
	GPIOHatfd = -1;
}
GPIOHatDriver::~GPIOHatDriver()
{
	if (GPIOHatfd){ //If the I2C File handle is still open...
			close(GPIOHatfd); //...Close it.
		}
}


int GPIOHatDriver::init(int address_)
{
	address = address_;
	GPIOHatfd = open("/dev/i2c-1",O_RDWR);
	if (GPIOHatfd < 0)
	{
		printf("Can't open I2C BUS\n");
	}
	if (ioctl(GPIOHatfd, I2C_SLAVE, address) < 0)
	{ //Using ioctl set the i2c device to talk to address in the "addr" variable.
		printf("Can't set the I2C address for the slave device\n");
	}
	poweron_selftest_passed = true;
	/*
	unsigned char inputbuffer[12];
	int result = sendQuery(I2CMessageHandler::I2C_TestProgram_ID,inputbuffer);
	
	if(result == TESTPROGRAMSTATE_PASSED)
	{
		poweron_selftest_passed = true;
	}
	else
	{
		printf("[WARN] Power On Self Test Failed with Result: %d\n",result);
	}
	*/
	/*

    // zero all PWM ports
    resetAllPWM(0,0);

    wiringPiI2CWriteReg8(ServoHatfd, __MODE2, __OUTDRV);
    wiringPiI2CWriteReg8(ServoHatfd, __MODE1, __ALLCALL);

    int mode1 = wiringPiI2CReadReg8(ServoHatfd, __MODE1);
    //printf("model: %d\n",model);
    mode1 = mode1 & ~__SLEEP;
    wiringPiI2CWriteReg8(ServoHatfd, __MODE1, mode1);

    setPWMFreq(60);
    */
    return GPIOHatfd;
}
int GPIOHatDriver::sendQuery(unsigned char query,unsigned char * inputbuffer)
{
	int v = query;
	char tmp[1];
	tmp[0] = v;
	i2cWrite(tmp,1);
	read(GPIOHatfd,(char *)(&inputbuffer[0]),13);
	unsigned char running_checksum = 0;
	for(int i = 0; i < 12; i++)
	{
		running_checksum ^= inputbuffer[i];

	}
	if(inputbuffer[12] == running_checksum) { return 1; }
	else { return 0; }
}
int GPIOHatDriver::i2cRead(char *data,int length){
	int er = read(GPIOHatfd,data,length); //Read "length" number of bytes into the "data" buffer from the I2C bus.
	return er;
}
int GPIOHatDriver::i2cWrite(char *data,int length){
	int er = write(GPIOHatfd,data,length);//Write "length" number of bytes from the "data" buffer to the I2C bus.
	return er;
}

int GPIOHatDriver::i2cReadArduinoInt(){
	const int arr_size = 2;
	char tmp[arr_size]; //We know an Arduino Int is 2 Bytes.
	int retval=-1;

	if (i2cRead(tmp,arr_size) > 0){
		retval = tmp[1] << 8 | tmp[0]; //Using bit shifting, turn the 2 byte array into an Int.
	}
	return retval;
}

int GPIOHatDriver::i2cWriteArduinoInt(int input){
	const int arr_size = 2;
	char tmp[arr_size]; //We know an Arduino Int is 2 Bytes.
	int retval=0;

	tmp[0] = input; //get lowest 8 bits into the first part of the array;
	tmp[1] = input >> 8; //get the highest 8 bits into the second part of the array;
	retval = (i2cWrite(tmp,arr_size) > 0);
	return retval;
}
/*
int GPIOHatDriver::test_comm(int v)
{
	//int v1 = wiringPiI2CWriteReg8(GPIOHatfd, 0, v);
	//wiringPiI2CWriteReg8(GPIOHatfd, 0, v+1);
	//wiringPiI2CWriteReg8(GPIOHatfd, 0, v+2);
	//wiringPiI2CWriteReg8(GPIOHatfd, 0, v+3);
	printf("%d\n",wiringPiI2CReadReg8(GPIOHatfd, 0));
	printf("%d\n",wiringPiI2CReadReg8(GPIOHatfd, 0));
	printf("%d\n",wiringPiI2CReadReg8(GPIOHatfd, 0));
	printf("%d\n",wiringPiI2CReadReg8(GPIOHatfd, 0));
	return 1;
}
*/
/*
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
*/
