#ifndef __DEFINITIONS_INCLUDED__   
#define __DEFINITIONS_INCLUDED__

//Diagnostic Definitions

//Field 1: System
#define ROVER 1
#define GROUND_STATION 5
#define REMOTE_CONTROL 7

//Field 2: Subsystem
#define ENTIRE_SYSTEM 0
#define ROBOT_CONTROLLER 1


//Field 3: Component
#define ENTIRE_SUBSYSTEM 0
#define DIAGNOSTIC_NODE 1
#define NAVIGATION_NODE 2
#define MOTION_CONTROLLER_NODE 3
#define SONIC_CONTROLLER_NODE 4
#define ROBOT_CONTROLLER_NODE 5
#define MAPPING_NODE 7
#define EVOLUTION_NODE 8
#define TARGETING_NODE 9
#define TIMING_NODE 10



//Field 4: Diagnostic Type
#define NO_ERROR 0
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
//#define NO_ERROR 0  Already defined above, just leaving here for completeness.
#define INITIALIZING 1
#define INITIALIZING_ERROR 2
#define DROPPING_PACKETS 4
#define MISSING_HEARTBEATS 5
#define DEVICE_NOT_AVAILABLE 6
#define ROVER_ARMED 7
#define ROVER_DISARMED 8
//#define GENERAL_ERROR 9  Already defined above, just leaving here for completeness

//Armed State Definitions
#define ARMED 1
#define DISARMED 0


//Gear Definitions
#define GEAR_FORWARD 1
#define GEAR_PARK    0
#define GEAR_REVERSE -1

#endif
