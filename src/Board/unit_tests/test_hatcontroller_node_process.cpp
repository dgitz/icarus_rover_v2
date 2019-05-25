#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "../HatControllerNodeProcess.h"

std::string Node_Name = "/unittest_hatcontroller_node_process";
std::string Host_Name = "ControlModule1";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ControlModule";
#define DIAGNOSTIC_TYPE_COUNT 5
void print_diagnostic(uint8_t level,eros::diagnostic diagnostic)
{
	if(diagnostic.Level >= level)
	{
		printf("Type: %d Message: %d Level: %d Device: %s Desc: %s\n",diagnostic.Diagnostic_Type,diagnostic.Diagnostic_Message,
			  		diagnostic.Level,diagnostic.DeviceName.c_str(),diagnostic.Description.c_str());
	}
}
HatControllerNodeProcess* initializeprocess()
{
	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.HatCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	HatControllerNodeProcess *process;
	process = new HatControllerNodeProcess;
	process->initialize("hatcontroller_node",Node_Name,Host_Name,ROVER,ROBOT_CONTROLLER,CONTROLLER_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	diagnostic_types.push_back(SENSORS);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
HatControllerNodeProcess* initializeprocess(eros::device device)
{
	HatControllerNodeProcess *process;
	process = new HatControllerNodeProcess;
	process->initialize("hatcontroller_node",Node_Name,Host_Name,ROVER,ROBOT_CONTROLLER,CONTROLLER_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	diagnostic_types.push_back(SENSORS);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
HatControllerNodeProcess* readyprocess(HatControllerNodeProcess* process)
{
	process->update_diagnostic(SYSTEM_RESOURCE,INFO,NOERROR,"Not testing this during unit tests.");
	eros::diagnostic diag = process->update(0.0, 0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	{
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		//EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT+(2*process->get_boarddata().size())+(process->get_sensordata().size()));
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			print_diagnostic(WARN,diagnostics.at(i));
			EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
		}
	}
	EXPECT_TRUE(process->is_ready() == true);
	return process;
}
TEST(Template,Process_Initialization_ServoHat)
{
	HatControllerNodeProcess* process = initializeprocess();

	eros::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = "";
	ros_device.DeviceType = ros_DeviceType;
	ros_device.HatCount = 1;
	ros_device.SensorCount = 0;
	ros_device.ID = 17;

	eros::device servohat1_device;
	servohat1_device.DeviceName = "ServoHat1";
	servohat1_device.DeviceParent = ros_DeviceName;
	servohat1_device.DeviceType = "ServoHat";
	servohat1_device.BoardCount = 0;
	servohat1_device.ID = 64;

	{
		eros::pin newpin;
		newpin.ConnectedDevice = "PWM1";
		newpin.Number = 0;
		newpin.Function = "PWMOutput-NonActuator";
		servohat1_device.pins.push_back(newpin);
	}

	{
		eros::pin newpin;
		newpin.ConnectedDevice = "PWM2";
		newpin.Number = 1;
		newpin.Function = "PWMOutput-NonActuator";
		servohat1_device.pins.push_back(newpin);
	}

	{
		eros::pin newpin;
		newpin.ConnectedDevice = "PWM3";
		newpin.Number = 0;
		newpin.Function = "PWMOutput-NonActuator";
		servohat1_device.pins.push_back(newpin);
	}

	{
		eros::pin newpin;
		newpin.ConnectedDevice = "PWM4";
		newpin.Number = 1;
		newpin.Function = "PWMOutput-NonActuator";
		servohat1_device.pins.push_back(newpin);
	}

	process->set_mydevice(ros_device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);

	eros::device::ConstPtr dev_ptr(new eros::device(servohat1_device));
	eros::diagnostic diagnostic = process->new_devicemsg(dev_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	process = readyprocess(process);
}
TEST(Template,Process_Initialization_GPIOHat)
{
	HatControllerNodeProcess* process = initializeprocess();
	eros::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = "";
	ros_device.DeviceType = ros_DeviceType;
	ros_device.HatCount = 1;
	ros_device.SensorCount = 0;
	ros_device.ID = 17;

	eros::device gpiohat1_device;
	gpiohat1_device.DeviceName = "GPIOHat1";
	gpiohat1_device.DeviceParent = ros_DeviceName;
	gpiohat1_device.DeviceType = "GPIOHat";
	gpiohat1_device.BoardCount = 0;
	gpiohat1_device.ID = 20;
	gpiohat1_device.SensorCount = 4;

	{
		eros::pin newpin;
		newpin.ConnectedSensor = "FLSonar";
		newpin.Number = 8;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		gpiohat1_device.pins.push_back(newpin);
	}

	{
		eros::pin newpin;
		newpin.ConnectedSensor = "FRSonar";
		newpin.Number = 9;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		gpiohat1_device.pins.push_back(newpin);
	}
	{
		eros::pin newpin;
		newpin.ConnectedSensor = "BLSonar";
		newpin.Number = 10;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		gpiohat1_device.pins.push_back(newpin);
	}
	{
		eros::pin newpin;
		newpin.ConnectedSensor = "BRSonar";
		newpin.Number = 11;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		gpiohat1_device.pins.push_back(newpin);
	}


	process->set_mydevice(ros_device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);

	eros::device::ConstPtr dev_ptr(new eros::device(gpiohat1_device));
	eros::diagnostic diagnostic = process->new_devicemsg(dev_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	process = readyprocess(process);
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
		eros::diagnostic diag = process->update(dt,current_time);
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
		eros::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
		if(fastrate_fire == true)
		{
			cmd.Option1 = LEVEL1;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);

		}
		if(mediumrate_fire == true)
		{
			EXPECT_TRUE(diag.Level <= NOTICE);
			cmd.Option1 = LEVEL2;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);

		}
		if(slowrate_fire == true)
		{
			std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
			EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT+process->get_hats().size() + process->get_sensordata().size()));
			for (std::size_t i = 0; i < diagnostics.size(); ++i)
			{
				print_diagnostic(WARN,diagnostics.at(i));
				EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
			}
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
TEST(DeviceInitialization,DeviceInitialization_TerminalHat)
{
	eros::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = ros_ParentDevice;
	ros_device.DeviceType = ros_DeviceType;
	ros_device.BoardCount = 0;
	ros_device.HatCount = 1;
	ros_device.ID = 123;
	HatControllerNodeProcess* process = initializeprocess(ros_device);


	eros::device hat1;
	hat1.DeviceName = "TerminalHat1";
	hat1.DeviceType = "TerminalHat";
	hat1.DeviceParent = ros_DeviceName;
	hat1.ID = 64;
	hat1.Architecture = "None";
	hat1.ShieldCount = 0;
	hat1.SensorCount = 0;
	hat1.pins.clear();
	std::vector<eros::pin> gpio_input_pins;
	std::vector<eros::pin> gpio_output_actuator_pins;
	std::vector<eros::pin> gpio_output_nonactuator_pins;
	for(int i = 0; i < 24;i++)
	{
		eros::pin newpin;
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

	eros::diagnostic diagnostic = process->update(0.02,0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	eros::device::ConstPtr hat1_ptr(new eros::device(hat1));
	diagnostic = process->new_devicemsg(hat1_ptr);
	process = readyprocess(process);
	diagnostic = process->set_terminalhat_initialized();
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	diagnostic = process->update(0.02,0.04);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	EXPECT_EQ(process->get_ready_to_arm(),true);
	EXPECT_TRUE(process->get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM);

	std::vector<eros::pin> input_pins = process->get_terminalhatpins("DigitalInput",true);
	std::vector<eros::pin> output_nonactuator_pins = process->get_terminalhatpins("DigitalOutput-NonActuator",true);
	std::vector<eros::pin> output_actuator_pins = process->get_terminalhatpins("DigitalOutput",true);
	std::vector<eros::pin> p = process->get_terminalhatpins("DigitalOutput",false);
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
	diagnostic = process->update(0.02,0.06);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_armedstate() == ARMEDSTATUS_DISARMED);

	output_actuator_pins.clear();
	output_actuator_pins = process->get_terminalhatpins("DigitalOutput",true);

	for(std::size_t i = 0; i < output_actuator_pins.size(); i++)
	{
		EXPECT_TRUE(output_actuator_pins.at(i).Value == gpio_output_actuator_pins.at(i).DefaultValue);
	}


	int value = 0;
	//eros::iopins p;
	for(std::size_t i = 0; i < input_pins.size(); i++)
	{
		eros::pin pin = input_pins.at(i);
		pin.Value = value;
		value = !value;
		gpio_input_pins.at(i) = pin;
		struct timeval now;
		gettimeofday(&now,NULL);
		pin.stamp = process->convert_time(now);
		usleep(10000);
		eros::pin::ConstPtr pin_ptr(new eros::pin(pin));
		diagnostic = process->new_pinmsg(pin_ptr);
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
	std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
	for (std::size_t i = 0; i < diagnostics.size(); ++i)
	{
		print_diagnostic(WARN,diagnostics.at(i));
		EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
	}
}
TEST(DeviceInitialization,DeviceInitialization_ServoHat_GPIOHat_TerminalHat)
{
	eros::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = ros_ParentDevice;
	ros_device.DeviceType = ros_DeviceType;
	ros_device.HatCount = 3;
	ros_device.ID = 123;


	eros::diagnostic diagnostic;
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

	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	eros::device hat1;
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
		eros::pin newpin;
		newpin.Number = i;
		if(i < 8) {  newpin.Function = "PWMOutput"; }
		else { newpin.Function = "PWMOutput-NonActuator"; }
		newpin.DefaultValue = 1600+i*3;
		newpin.Value = 1400-(i*3);
		newpin.ParentDevice = hat1.DeviceName;
		hat1.pins.push_back(newpin);
	}

	eros::device hat2;
	hat2.DeviceName = "GPIOHat1";
	hat2.DeviceType = "GPIOHat";
	hat2.DeviceParent = ros_DeviceName;
	hat2.ID = 2;
	hat2.Architecture = "None";
	hat2.ShieldCount = 0;
	hat2.SensorCount = 4;
	hat2.pins.clear();
	{
		eros::pin newpin;
		newpin.ConnectedSensor = "FLSonar";
		newpin.Number = 0;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		hat2.pins.push_back(newpin);
	}

	{
		eros::pin newpin;
		newpin.ConnectedSensor = "FRSonar";
		newpin.Number = 1;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		hat2.pins.push_back(newpin);
	}
	{
		eros::pin newpin;
		newpin.ConnectedSensor = "BLSonar";
		newpin.Number = 2;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		hat2.pins.push_back(newpin);
	}
	{
		eros::pin newpin;
		newpin.ConnectedSensor = "BRSonar";
		newpin.Number = 3;
		newpin.Name = "UltraSonicSensor";
		newpin.Function = "UltraSonicSensorInput";
		hat2.pins.push_back(newpin);
	}

	eros::device hat3;
	hat3.DeviceName = "TerminalHat1";
	hat3.DeviceType = "TerminalHat";
	hat3.DeviceParent = ros_DeviceName;
	hat3.ID = 64;
	hat3.Architecture = "None";
	hat3.ShieldCount = 0;
	hat3.SensorCount = 0;
	hat3.pins.clear();
	std::vector<eros::pin> gpio_input_pins;
	std::vector<eros::pin> gpio_output_actuator_pins;
	std::vector<eros::pin> gpio_output_nonactuator_pins;
	for(int i = 0; i < 24;i++)
	{
		eros::pin newpin;
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
	diagnostic = process->update(0.02,.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	eros::device::ConstPtr hat1_ptr(new eros::device(hat1));
	diagnostic = process->new_devicemsg(hat1_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == false);

	eros::device::ConstPtr hat2_ptr(new eros::device(hat2));
	diagnostic = process->new_devicemsg(hat2_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == false);

	eros::device::ConstPtr hat3_ptr(new eros::device(hat3));
	diagnostic = process->new_devicemsg(hat3_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	process = readyprocess(process);

	diagnostic = process->set_hat_running("ServoHat",0);
	EXPECT_TRUE(diagnostic.Level > NOTICE);

	diagnostic = process->set_hat_running("ServoHat",hat1.ID);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02,.04);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_ready_to_arm(),false);

	diagnostic = process->set_hat_running("GPIOHat",hat2.ID);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02,.06);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_ready_to_arm(),false);

	diagnostic = process->set_hat_running("TerminalHat",hat3.ID);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02,.08);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_ready_to_arm(),true);

	std::vector<eros::pin> servohat_pins = process->get_servohatpins(hat1.ID);
	std::vector<eros::pin> gpiohat_pins = process->get_gpiohatpins(hat2.ID);
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
	diagnostic = process->update(0.02,0.1);
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
	//eros::iopins p;
	for(std::size_t i = 0; i < hat1.pins.size(); i++)
	{
		eros::pin pin = hat1.pins.at(i);
		pin.Value = 1000 + i*5;
		hat1.pins.at(i).Value = pin.Value;
		struct timeval now;
		gettimeofday(&now,NULL);
		pin.stamp = process->convert_time(now);
		usleep(10000);
		eros::pin::ConstPtr pin_ptr(new eros::pin(pin));
		diagnostic = process->new_pinmsg(pin_ptr);
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
	diagnostic = process->update(0.02,0.12);
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
	std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
	//EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT+(2*process->get_boarddata().size())+(process->get_sensordata().size()));
	for (std::size_t i = 0; i < diagnostics.size(); ++i)
	{
		print_diagnostic(WARN,diagnostics.at(i));
		EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
	}
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
