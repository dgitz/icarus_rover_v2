#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/command.h"
#include "../hatcontroller_node_process.h"

std::string Node_Name = "/unittest_hatcontroller_node_process";
std::string Host_Name = "ControlModule1";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ControlModule";

HatControllerNodeProcess* initializeprocess()
{
	icarus_rover_v2::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = GPIO_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";

	icarus_rover_v2::device device;
	device.DeviceName = diagnostic.DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.HatCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	HatControllerNodeProcess *process;
	process = new HatControllerNodeProcess("hatcontroller_node",Node_Name);
	diagnostic = process->init(diagnostic,std::string(Host_Name));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->get_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
HatControllerNodeProcess* initializeprocess(icarus_rover_v2::device device)
{
	icarus_rover_v2::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = GPIO_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";

	HatControllerNodeProcess *process;
	process = new HatControllerNodeProcess("hatcontroller_node",Node_Name);
	diagnostic = process->init(diagnostic,std::string(Host_Name));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->get_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
HatControllerNodeProcess* readyprocess(HatControllerNodeProcess* process)
{
	icarus_rover_v2::diagnostic diag = process->update(0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready() == true);
	return process;
}
TEST(Template,Process_Initialization_ServoHat)
{
	HatControllerNodeProcess* process = initializeprocess();
	icarus_rover_v2::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = COMMUNICATION_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";

	icarus_rover_v2::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = "";
	ros_device.DeviceType = ros_DeviceType;
	ros_device.HatCount = 1;
	ros_device.SensorCount = 0;
	ros_device.ID = 17;

	icarus_rover_v2::device servohat1_device;
	servohat1_device.DeviceName = "ServoHat1";
	servohat1_device.DeviceParent = ros_DeviceName;
	servohat1_device.DeviceType = "ServoHat";
	servohat1_device.BoardCount = 0;
	servohat1_device.ID = 64;

	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedDevice = "PWM1";
		newpin.Number = 0;
		newpin.Function = "PWMOutput-NonActuator";
		servohat1_device.pins.push_back(newpin);
	}

	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedDevice = "PWM2";
		newpin.Number = 1;
		newpin.Function = "PWMOutput-NonActuator";
		servohat1_device.pins.push_back(newpin);
	}

	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedDevice = "PWM3";
		newpin.Number = 0;
		newpin.Function = "PWMOutput-NonActuator";
		servohat1_device.pins.push_back(newpin);
	}

	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedDevice = "PWM4";
		newpin.Number = 1;
		newpin.Function = "PWMOutput-NonActuator";
		servohat1_device.pins.push_back(newpin);
	}





	diagnostic = process->init(diagnostic,std::string(Host_Name));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	process->set_mydevice(ros_device);
	EXPECT_TRUE(process->get_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);

	diagnostic = process->new_devicemsg(servohat1_device);
	printf("%s\n",diagnostic.Description.c_str());
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	EXPECT_TRUE(process->is_ready() == true);
}
TEST(Template,Process_Initialization_GPIOHat)
{
	HatControllerNodeProcess* process = initializeprocess();
	icarus_rover_v2::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = COMMUNICATION_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";

	icarus_rover_v2::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = "";
	ros_device.DeviceType = ros_DeviceType;
	ros_device.HatCount = 1;
	ros_device.SensorCount = 0;
	ros_device.ID = 17;

	icarus_rover_v2::device gpiohat1_device;
	gpiohat1_device.DeviceName = "GPIOHat1";
	gpiohat1_device.DeviceParent = ros_DeviceName;
	gpiohat1_device.DeviceType = "GPIOHat";
	gpiohat1_device.BoardCount = 0;
	gpiohat1_device.ID = 20;
	gpiohat1_device.SensorCount = 4;

	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedSensor = "FLSonar";
		newpin.Number = 8;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		gpiohat1_device.pins.push_back(newpin);
	}

	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedSensor = "FRSonar";
		newpin.Number = 9;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		gpiohat1_device.pins.push_back(newpin);
	}
	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedSensor = "BLSonar";
		newpin.Number = 10;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		gpiohat1_device.pins.push_back(newpin);
	}
	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedSensor = "BRSonar";
		newpin.Number = 11;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		gpiohat1_device.pins.push_back(newpin);
	}


	diagnostic = process->init(diagnostic,std::string(Host_Name));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	process->set_mydevice(ros_device);
	EXPECT_TRUE(process->get_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);

	diagnostic = process->new_devicemsg(gpiohat1_device);
	printf("%s\n",diagnostic.Description.c_str());
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	EXPECT_TRUE(process->is_ready() == true);
}

TEST(Template,Process_Command)
{
	HatControllerNodeProcess* process = initializeprocess();
	process = readyprocess(process);
	double time_to_run = 40.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	while(current_time <= time_to_run)
	{
		icarus_rover_v2::diagnostic diag = process->update(dt);
		EXPECT_TRUE(diag.Level <= NOTICE);
		int current_time_ms = (int)(current_time*1000.0);
		if((current_time_ms % 100) == 0)
		{
			fastrate_fire = true;
		}
		else { fastrate_fire = false; }
		if((current_time_ms % 1000) == 0)
		{
			mediumrate_fire = true;
		}
		else { mediumrate_fire = false; }
		if((current_time_ms % 10000) == 0)
		{
			slowrate_fire = true;
		}
		else { slowrate_fire = false; }
		icarus_rover_v2::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
		if(fastrate_fire == true)
		{
			cmd.Option1 = LEVEL1;
			std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);

		}
		if(mediumrate_fire == true)
		{
			std_msgs::Bool pps;
			pps.data = true;
			diag = process->new_ppsmsg(pps);
			EXPECT_TRUE(diag.Level <= NOTICE);
			cmd.Option1 = LEVEL2;
			std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);

		}
		if(slowrate_fire == true)
		{
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
TEST(DeviceInitialization,DeviceInitialization_TerminalHat)
{
	HatControllerNodeProcess* process = initializeprocess();
	icarus_rover_v2::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = ros_ParentDevice;
	ros_device.DeviceType = ros_DeviceType;
	ros_device.BoardCount = 0;
	ros_device.HatCount = 1;
	ros_device.ID = 123;


	icarus_rover_v2::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = GPIO_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";
	Logger *logger;
	std::string log_output = Node_Name + boost::lexical_cast<std::string>(1);
	logger = new Logger("DEBUG","UNIT_TESTS",log_output);
	process->init(diagnostic,std::string(Host_Name));
	process->set_analyzetiming(true);
	process->set_mydevice(ros_device);
	EXPECT_EQ(process->get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
	EXPECT_EQ(process->get_ready_to_arm(),false);

	diagnostic = process->new_devicemsg(ros_device);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_initialized() == true);
	EXPECT_TRUE(process->get_ready() == false);

	std_msgs::Bool pps;
	pps.data = true;
	diagnostic = process->new_ppsmsg(pps);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	icarus_rover_v2::device hat1;
	hat1.DeviceName = "TerminalHat1";
	hat1.DeviceType = "TerminalHat";
	hat1.DeviceParent = ros_DeviceName;
	hat1.ID = 64;
	hat1.Architecture = "None";
	hat1.ShieldCount = 0;
	hat1.SensorCount = 0;
	hat1.pins.clear();
	std::vector<icarus_rover_v2::pin> gpio_input_pins;
	std::vector<icarus_rover_v2::pin> gpio_output_actuator_pins;
	std::vector<icarus_rover_v2::pin> gpio_output_nonactuator_pins;
	for(int i = 0; i < 24;i++)
	{
		icarus_rover_v2::pin newpin;
		newpin.Number = i;
		if(i < 8) {  newpin.Function = "DigitalOutput"; }
		else if(i < 16) { newpin.Function = "DigitalInput"; }
		else { newpin.Function = "DigitalOutput-NonActuator"; }
		newpin.DefaultValue = 0;
		newpin.Value = 1;
		newpin.ParentDevice = hat1.DeviceName;
		if(i < 8) { gpio_output_actuator_pins.push_back(newpin); }
		else if(i < 16) { gpio_input_pins.push_back(newpin); }
		else { gpio_output_nonactuator_pins.push_back(newpin); }
		hat1.pins.push_back(newpin);
	}

	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	diagnostic = process->new_devicemsg(hat1);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);


	EXPECT_TRUE(process->is_ready() == true);
	diagnostic = process->set_terminalhat_initialized();
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	EXPECT_EQ(process->get_ready_to_arm(),true);
	EXPECT_TRUE(process->get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM);

	std::vector<icarus_rover_v2::pin> input_pins = process->get_terminalhatpins("DigitalInput",true);
	std::vector<icarus_rover_v2::pin> output_nonactuator_pins = process->get_terminalhatpins("DigitalOutput-NonActuator",true);
	std::vector<icarus_rover_v2::pin> output_actuator_pins = process->get_terminalhatpins("DigitalOutput",true);
	std::vector<icarus_rover_v2::pin> p = process->get_terminalhatpins("DigitalOutput",false);
	EXPECT_TRUE(input_pins.size() > 0);
	EXPECT_TRUE(output_nonactuator_pins.size() > 0);
	EXPECT_TRUE(output_actuator_pins.size() > 0);
	EXPECT_TRUE(p.size() > 0);
	EXPECT_TRUE(p.size() == (output_nonactuator_pins.size() + output_actuator_pins.size()));
	EXPECT_TRUE(input_pins.size() == gpio_input_pins.size());
	EXPECT_TRUE(output_nonactuator_pins.size() == gpio_output_nonactuator_pins.size());
	EXPECT_TRUE(output_actuator_pins.size() == gpio_output_actuator_pins.size());
	EXPECT_TRUE((input_pins.size() + output_nonactuator_pins.size() + output_actuator_pins.size())  == hat1.pins.size());
	for(std::size_t i = 0; i < output_actuator_pins.size(); i++)
	{
		EXPECT_TRUE(output_actuator_pins.at(i).DefaultValue == output_actuator_pins.at(i).Value);
		EXPECT_TRUE(output_actuator_pins.at(i).Value == gpio_output_actuator_pins.at(i).DefaultValue);
	}


	diagnostic = process->new_armedstatemsg(ARMEDSTATUS_ARMED);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_armedstate() == ARMEDSTATUS_ARMED);

	output_actuator_pins.clear();
	output_actuator_pins = process->get_terminalhatpins("DigitalOutput",true);

	for(std::size_t i = 0; i < output_actuator_pins.size(); i++)
	{
		EXPECT_TRUE(output_actuator_pins.at(i).Value == gpio_output_actuator_pins.at(i).Value);
	}
	diagnostic = process->new_armedstatemsg(ARMEDSTATUS_DISARMED);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_armedstate() == ARMEDSTATUS_DISARMED);

	output_actuator_pins.clear();
	output_actuator_pins = process->get_terminalhatpins("DigitalOutput",true);

	for(std::size_t i = 0; i < output_actuator_pins.size(); i++)
	{
		EXPECT_TRUE(output_actuator_pins.at(i).Value == gpio_output_actuator_pins.at(i).DefaultValue);
	}


	int value = 0;
	//icarus_rover_v2::iopins p;
	for(std::size_t i = 0; i < input_pins.size(); i++)
	{
		icarus_rover_v2::pin pin = input_pins.at(i);
		pin.Value = value;
		value = !value;
		gpio_input_pins.at(i) = pin;
		struct timeval now;
		gettimeofday(&now,NULL);
		pin.stamp = process->convert_time(now);
		usleep(10000);
		diagnostic = process->new_pinmsg(pin);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		//p.pins.push_back(pin);

	}


	input_pins.clear();
	input_pins = process->get_terminalhatpins("DigitalInput",true);
	EXPECT_TRUE(input_pins.size() == gpio_input_pins.size());
	for(std::size_t i = 0; i < input_pins.size(); i++)
	{
		EXPECT_TRUE(input_pins.at(i).Value == gpio_input_pins.at(i).Value);
	}
}
TEST(DeviceInitialization,DeviceInitialization_ServoHat_GPIOHat_TerminalHat)
{

	icarus_rover_v2::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = ros_ParentDevice;
	ros_device.DeviceType = ros_DeviceType;
	ros_device.HatCount = 3;
	ros_device.ID = 123;


	icarus_rover_v2::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = GPIO_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";

	HatControllerNodeProcess* process = initializeprocess(ros_device);


	process->set_analyzetiming(true);
	EXPECT_EQ(process->get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
	EXPECT_EQ(process->get_ready_to_arm(),false);

	diagnostic = process->new_devicemsg(ros_device);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);

	std_msgs::Bool pps;
	pps.data = true;
	diagnostic = process->new_ppsmsg(pps);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	icarus_rover_v2::device hat1;
	hat1.DeviceName = "ServoHat1";
	hat1.DeviceType = "ServoHat";
	hat1.DeviceParent = ros_DeviceName;
	hat1.ID = 64;
	hat1.Architecture = "None";
	hat1.ShieldCount = 0;
	hat1.SensorCount = 0;
	hat1.pins.clear();
	for(int i = 0; i < 16;i++)
	{
		icarus_rover_v2::pin newpin;
		newpin.Number = i;
		if(i < 8) {  newpin.Function = "PWMOutput"; }
		else { newpin.Function = "PWMOutput-NonActuator"; }
		newpin.DefaultValue = 1600+i*3;
		newpin.Value = 1400-(i*3);
		newpin.ParentDevice = hat1.DeviceName;
		hat1.pins.push_back(newpin);
	}

	icarus_rover_v2::device hat2;
	hat2.DeviceName = "GPIOHat1";
	hat2.DeviceType = "GPIOHat";
	hat2.DeviceParent = ros_DeviceName;
	hat2.ID = 2;
	hat2.Architecture = "None";
	hat2.ShieldCount = 0;
	hat2.SensorCount = 4;
	hat2.pins.clear();
	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedSensor = "FLSonar";
		newpin.Number = 0;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		hat2.pins.push_back(newpin);
	}

	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedSensor = "FRSonar";
		newpin.Number = 1;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		hat2.pins.push_back(newpin);
	}
	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedSensor = "BLSonar";
		newpin.Number = 2;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		hat2.pins.push_back(newpin);
	}
	{
		icarus_rover_v2::pin newpin;
		newpin.ConnectedSensor = "BRSonar";
		newpin.Number = 3;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		hat2.pins.push_back(newpin);
	}

	icarus_rover_v2::device hat3;
	hat3.DeviceName = "TerminalHat1";
	hat3.DeviceType = "TerminalHat";
	hat3.DeviceParent = ros_DeviceName;
	hat3.ID = 64;
	hat3.Architecture = "None";
	hat3.ShieldCount = 0;
	hat3.SensorCount = 0;
	hat3.pins.clear();
	std::vector<icarus_rover_v2::pin> gpio_input_pins;
	std::vector<icarus_rover_v2::pin> gpio_output_actuator_pins;
	std::vector<icarus_rover_v2::pin> gpio_output_nonactuator_pins;
	for(int i = 0; i < 24;i++)
	{
		icarus_rover_v2::pin newpin;
		newpin.Number = i;
		if(i < 8) {  newpin.Function = "DigitalOutput"; }
		else if(i < 16) { newpin.Function = "DigitalInput"; }
		else { newpin.Function = "DigitalOutput-NonActuator"; }
		newpin.DefaultValue = 0;
		newpin.Value = 1;
		newpin.ParentDevice = hat1.DeviceName;
		if(i < 8) { gpio_output_actuator_pins.push_back(newpin); }
		else if(i < 16) { gpio_input_pins.push_back(newpin); }
		else { gpio_output_nonactuator_pins.push_back(newpin); }
		hat3.pins.push_back(newpin);
	}
	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	diagnostic = process->new_devicemsg(hat1);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == false);

	diagnostic = process->new_devicemsg(hat2);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == false);

	diagnostic = process->new_devicemsg(hat3);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);

	diagnostic = process->set_hat_running("ServoHat",0);
	EXPECT_TRUE(diagnostic.Level > NOTICE);

	diagnostic = process->set_hat_running("ServoHat",hat1.ID);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_ready_to_arm(),false);

	diagnostic = process->set_hat_running("GPIOHat",hat2.ID);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_ready_to_arm(),false);

	diagnostic = process->set_hat_running("TerminalHat",hat3.ID);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_ready_to_arm(),true);

	std::vector<icarus_rover_v2::pin> servohat_pins = process->get_servohatpins(hat1.ID);
	std::vector<icarus_rover_v2::pin> gpiohat_pins = process->get_gpiohatpins(hat2.ID);
	EXPECT_TRUE(servohat_pins.size() == hat1.pins.size());
	for(std::size_t i = 0; i < hat1.pins.size(); i++)
	{
		EXPECT_TRUE(hat1.pins.at(i).Number == servohat_pins.at(i).Number);
		if(hat1.pins.at(i).Function == "PWMOutput")
		{
			EXPECT_TRUE(hat1.pins.at(i).DefaultValue == servohat_pins.at(i).Value);
		}
		else if(hat1.pins.at(i).Function == "PWMOutput-NonActuator")
		{
			EXPECT_TRUE(hat1.pins.at(i).Value == servohat_pins.at(i).Value);
		}
		else
		{
			EXPECT_TRUE(1 == 0); //Shouldn't get here
		}
	}
	EXPECT_TRUE(gpiohat_pins.size() == hat2.pins.size());
	for(std::size_t i = 0; i < hat2.pins.size(); i++)
	{
		EXPECT_TRUE(hat2.pins.at(i).Number == gpiohat_pins.at(i).Number);
		if(hat2.pins.at(i).Function == "UltraSonicSensorInput")
		{
			EXPECT_TRUE(hat2.pins.at(i).DefaultValue == gpiohat_pins.at(i).Value);
		}
		else
		{
			EXPECT_TRUE(1 == 0); //Shouldn't get here
		}
	}


	diagnostic = process->new_armedstatemsg((uint8_t)(ARMEDSTATUS_ARMED));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	servohat_pins.clear();
	gpiohat_pins.clear();
	std::vector<uint16_t> servohats = process->get_servohataddresses();
	EXPECT_TRUE(servohats.size() == 1);
	std::vector<uint16_t> gpiohats = process->get_gpiohataddresses();
	EXPECT_TRUE(gpiohats.size() == 1);
	servohat_pins = process->get_servohatpins(servohats.at(0));
	EXPECT_TRUE(servohat_pins.size() == hat1.pins.size());
	gpiohat_pins = process->get_gpiohatpins(hat2.ID);
	EXPECT_TRUE(gpiohat_pins.size() == hat2.pins.size());
	for(std::size_t i = 0; i < hat1.pins.size(); i++)
	{
		EXPECT_TRUE(hat1.pins.at(i).Number == servohat_pins.at(i).Number);
		EXPECT_TRUE(hat1.pins.at(i).Value == servohat_pins.at(i).Value);
	}
	for(std::size_t i = 0; i < hat2.pins.size(); i++)
	{
		EXPECT_TRUE(hat2.pins.at(i).Number == gpiohat_pins.at(i).Number);
		EXPECT_TRUE(hat2.pins.at(i).Value == gpiohat_pins.at(i).Value);
	}
	diagnostic = process->new_armedstatemsg(ARMEDSTATUS_DISARMED);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_armedstate() == ARMEDSTATUS_DISARMED);
	servohat_pins.clear();
	gpiohat_pins.clear();
	servohat_pins = process->get_servohatpins(servohats.at(0));
	gpiohat_pins = process->get_gpiohatpins(hat2.ID);
	EXPECT_TRUE(servohat_pins.size() == hat1.pins.size());
	EXPECT_TRUE(gpiohat_pins.size() == hat2.pins.size());
	for(std::size_t i = 0; i < hat1.pins.size(); i++)
	{
		EXPECT_TRUE(hat1.pins.at(i).Number == servohat_pins.at(i).Number);
		if(hat1.pins.at(i).Function == "PWMOutput")
		{
			EXPECT_TRUE(hat1.pins.at(i).DefaultValue == servohat_pins.at(i).Value);
		}
		else if(hat1.pins.at(i).Function == "PWMOutput-NonActuator")
		{
			EXPECT_TRUE(hat1.pins.at(i).Value == servohat_pins.at(i).Value);
		}
		else
		{
			EXPECT_TRUE(1 == 0); //Shouldn't get here
		}
	}
	for(std::size_t i = 0; i < hat2.pins.size(); i++)
	{
		EXPECT_TRUE(hat2.pins.at(i).Number == gpiohat_pins.at(i).Number);
		if(hat2.pins.at(i).Function == "UltraSonicSensorInput")
		{
			EXPECT_TRUE(hat2.pins.at(i).DefaultValue == gpiohat_pins.at(i).Value);
		}
		else
		{
			EXPECT_TRUE(1 == 0); //Shouldn't get here
		}
	}
	//icarus_rover_v2::iopins p;
	for(std::size_t i = 0; i < hat1.pins.size(); i++)
	{
		icarus_rover_v2::pin pin = hat1.pins.at(i);
		pin.Value = 1000 + i*5;
		hat1.pins.at(i).Value = pin.Value;
		struct timeval now;
		gettimeofday(&now,NULL);
		pin.stamp = process->convert_time(now);
		usleep(10000);
		diagnostic = process->new_pinmsg(pin);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);

	}

	uint16_t a1 = 11;
	uint16_t a2 = 22;
	uint16_t a3 = 33;
	uint16_t a4 = 44;
	diagnostic = process->new_message_GetDIOPort1(hat2.ID,1.234,a1,a2,a3,a4);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	servohat_pins.clear();
	gpiohat_pins.clear();
	servohat_pins = process->get_servohatpins(servohats.at(0));
	EXPECT_TRUE(servohat_pins.size() == hat1.pins.size());
	gpiohat_pins = process->get_gpiohatpins(hat2.ID);
	EXPECT_TRUE(gpiohat_pins.size() == hat2.pins.size());
	for(std::size_t i = 0; i < hat1.pins.size(); i++)
	{
		EXPECT_TRUE(hat1.pins.at(i).Number == servohat_pins.at(i).Number);
		if(hat1.pins.at(i).Function == "PWMOutput")
		{
			EXPECT_TRUE(hat1.pins.at(i).DefaultValue == servohat_pins.at(i).Value);
		}
		else if(hat1.pins.at(i).Function == "PWMOutput-NonActuator")
		{
			EXPECT_TRUE(hat1.pins.at(i).Value == servohat_pins.at(i).Value);
		}
		else
		{
			EXPECT_TRUE(1 == 0); //Shouldn't get here
		}
	}
	diagnostic = process->new_armedstatemsg(ARMEDSTATUS_ARMED);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_armedstate() == ARMEDSTATUS_ARMED);
	servohat_pins.clear();
	gpiohat_pins.clear();
	servohat_pins = process->get_servohatpins(servohats.at(0));
	EXPECT_TRUE(servohat_pins.size() == hat1.pins.size());
	gpiohat_pins = process->get_gpiohatpins(hat2.ID);
	EXPECT_TRUE(gpiohat_pins.size() == hat2.pins.size());
	for(std::size_t i = 0; i < hat1.pins.size(); i++)
	{
		EXPECT_TRUE(hat1.pins.at(i).Number == servohat_pins.at(i).Number);
		EXPECT_TRUE(hat1.pins.at(i).Value == servohat_pins.at(i).Value);
	}
	for(std::size_t i = 0; i < hat2.pins.size(); i++)
	{
		EXPECT_TRUE(hat2.pins.at(i).Number == gpiohat_pins.at(i).Number);
	}
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
