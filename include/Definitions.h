#ifndef __DEFINITIONS_INCLUDED__   
#define __DEFINITIONS_INCLUDED__

#define RESOURCE_ID 0xAB11  //Should be the same as UDP_Resource_ID
#define DIAGNOSTIC_ID 0xAB12  //Should be the same as UDP_Diagnostic_ID
#define DEVICE_ID 0xAB13 //Should be the same as UDP_Device_ID

//Diagnostic Definitions
#define LEVEL1 1
#define LEVEL2 2
#define LEVEL3 3
#define LEVEL4 4

//Field 1: System
#define ROVER 1
#define GROUND_STATION 5
#define REMOTE_CONTROL 7

//Field 2: Subsystem
#define ENTIRE_SYSTEM 0
#define ROBOT_CONTROLLER 1


//Field 3: Component
#define ENTIRE_SUBSYSTEM 0
#define CONTROLLER_NODE 1
#define DIAGNOSTIC_NODE 2
#define NAVIGATION_NODE 3
#define MAPPING_NODE 7
#define EVOLUTION_NODE 8
#define TARGETING_NODE 9
#define TIMING_NODE 10
#define VISION_NODE 11
#define GPIO_NODE 12
#define COMMUNICATION_NODE 13



//Field 4: Diagnostic Type
#define NOERROR 0
#define ELECTRICAL 1
#define SOFTWARE 2
#define COMMUNICATIONS 3
#define SENSORS 4
#define ACTUATORS 5
#define DATA_STORAGE 6
#define REMOTE_CONTROL 7
#define GENERAL_ERROR 9

//Field 5: Level
#define DEBUG 0
#define INFO 1
#define NOTICE 2
#define WARN 3
#define ERROR 4
#define FATAL 5

//Field 6: Diagnostic_Message
//#define NOERROR 0  Already defined above, just leaving here for completeness.
#define INITIALIZING 1
#define INITIALIZING_ERROR 2
#define DROPPING_PACKETS 4
#define MISSING_HEARTBEATS 5
#define DEVICE_NOT_AVAILABLE 6
#define ROVER_ARMED 7
#define ROVER_DISARMED 8
#define TEMPERATURE_HIGH 9
#define TEMPERATURE_LOW 10
#define DIAGNOSTIC_PASSED 11
#define DIAGNOSTIC_FAILED 12

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
#define PINMODE_FORCESENSOR_INPUT 4
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
