#ifndef __DEFINITIONS_INCLUDED__   
#define __DEFINITIONS_INCLUDED__
#include "eROS_Definitions.h"
//Armed State Definitions
#define ARMED 1
#define DISARMED 0


//Gear Definitions
#define GEAR_FORWARD 1
#define GEAR_PARK    0
#define GEAR_REVERSE -1

//Pin Mode Definitions
#define PINMODE_UNDEFINED 0
#define PINMODE_DIGITAL_OUTPUT 1
#define PINMODE_DIGITAL_INPUT 2
#define PINMODE_ANALOG_INPUT 3
#define PINMODE_FORCESENSOR_INPUT 4  //icarus_rover_v2::pin Value will be in units: grams
#define PINMODE_ULTRASONIC_INPUT 5
#define PINMODE_QUADRATUREENCODER_INPUT 6
#define PINMODE_PWM_OUTPUT 7
#define PINMODE_NOCHANGE 255

//GPIO Board Mode Definitions
#define GPIO_MODE_UNDEFINED 0
#define GPIO_MODE_BOOT 1
#define GPIO_MODE_INITIALIZING 2
#define GPIO_MODE_INITIALIZED 3
#define GPIO_MODE_RUNNING 4
#define GPIO_MODE_STOPPED 5
#endif
