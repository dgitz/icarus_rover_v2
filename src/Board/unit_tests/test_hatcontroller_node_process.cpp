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
void print_diagnostic(uint8_t level, eros::diagnostic diagnostic)
{
	if (diagnostic.Level >= level)
	{
		printf("Type: %d Message: %d Level: %d Device: %s Desc: %s\n", diagnostic.Diagnostic_Type, diagnostic.Diagnostic_Message,
			   diagnostic.Level, diagnostic.DeviceName.c_str(), diagnostic.Description.c_str());
	}
}
eros::device initialize_device(eros::device device, std::vector<HatControllerNodeProcess::HatMap> supported_hats)
{
	for (std::size_t i = 0; i < supported_hats.size(); ++i)
	{
		std::vector<HatControllerNodeProcess::PinDefinition> pin_defs = supported_hats.at(i).PinMap;
		if (supported_hats.at(i).FAST_PN == device.PartNumber)
		{
			if (supported_hats.at(i).FAST_PN == PN_625005)
			{
				{
					eros::pin newpin;
					newpin.Name = "GPIO23";
					newpin.ConnectedDevice = "BatteryPowerSwitch";
					newpin.Function = "DigitalInput";
					device.pins.push_back(newpin);
				}
				{
					eros::pin newpin;
					newpin.Name = "GPIO25";
					newpin.ConnectedDevice = "ArmSwitch";
					newpin.Function = "DigitalInput";
					device.pins.push_back(newpin);
				}
			}
			else if (supported_hats.at(i).FAST_PN == PN_100007)
			{
				{
					eros::pin newpin;
					newpin.Name = "PC6";
					newpin.ConnectedSensor = "FLSonar";
					newpin.Function = "UltraSonicSensorInput";
					device.pins.push_back(newpin);
				}
				{
					eros::pin newpin;
					newpin.Name = "PB7";
					newpin.ConnectedSensor = "FRSonar";
					newpin.Function = "UltraSonicSensorInput";
					device.pins.push_back(newpin);
				}
				{
					eros::pin newpin;
					newpin.Name = "PE6";
					newpin.ConnectedSensor = "BLSonar";
					newpin.Function = "UltraSonicSensorInput";
					device.pins.push_back(newpin);
				}
				{
					eros::pin newpin;
					newpin.Name = "PB4";
					newpin.ConnectedSensor = "BRSonar";
					newpin.Function = "UltraSonicSensorInput";
					device.pins.push_back(newpin);
				}
				{
					eros::pin newpin;
					newpin.Name = "PF4";
					newpin.ConnectedDevice = "ImplementCylinderLength";
					newpin.Function = "AnalogInput";
					device.pins.push_back(newpin);
				}
				{
					eros::pin newpin;
					newpin.Name = "PF6";
					newpin.ConnectedDevice = "LeftLiftCylinderCurrent";
					newpin.Function = "AnalogInput";
					device.pins.push_back(newpin);
				}
				{
					eros::pin newpin;
					newpin.Name = "PB6";
					newpin.ConnectedDevice = "LeftLiftCylinderTemperature";
					newpin.Function = "AnalogInput";
					device.pins.push_back(newpin);
				}
			}
			else if (supported_hats.at(i).FAST_PN == PN_625004)
			{
				{
					eros::pin newpin;
					newpin.Name = "CH0";
					newpin.ConnectedDevice = "PanServo";
					newpin.Function = "PWMOutput-NonActuator";
					device.pins.push_back(newpin);
				}
				{
					eros::pin newpin;
					newpin.Name = "CH1";
					newpin.ConnectedDevice = "TiltServo";
					newpin.Function = "PWMOutput-NonActuator";
					device.pins.push_back(newpin);
				}
				{
					eros::pin newpin;
					newpin.Name = "CH2";
					newpin.ConnectedDevice = "LeftMotorController";
					newpin.Function = "PWMOutput";
					device.pins.push_back(newpin);
				}
				{
					eros::pin newpin;
					newpin.Name = "CH3";
					newpin.ConnectedDevice = "RightMotorController";
					newpin.Function = "PWMOutput";
					device.pins.push_back(newpin);
				}
			}
		}
	}
	EXPECT_TRUE(device.pins.size() > 0);
	return device;
}
HatControllerNodeProcess *initializeprocess()
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
	process->initialize("hatcontroller_node", Node_Name, Host_Name, ROVER, ROBOT_CONTROLLER, CONTROLLER_NODE);
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
HatControllerNodeProcess *initializeprocess(eros::device device)
{
	HatControllerNodeProcess *process;
	process = new HatControllerNodeProcess;
	process->initialize("hatcontroller_node", Node_Name, Host_Name, ROVER, ROBOT_CONTROLLER, CONTROLLER_NODE);
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
HatControllerNodeProcess *readyprocess(HatControllerNodeProcess *process)
{
	process->update_diagnostic(SYSTEM_RESOURCE, INFO, NOERROR, "Not testing this during unit tests.");
	eros::diagnostic diag = process->update(0.0, 0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	{
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		//EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT+(2*process->get_boarddata().size())+(process->get_sensordata().size()));
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			print_diagnostic(WARN, diagnostics.at(i));
			EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
		}
	}
	if(process->is_ready() == false)
	{
		printf("[ERROR]: Process is not ready yet.  Configured Hats:\n");
		std::vector<eros::device> hats = process->get_hats();
		if(hats.size() == 0)
		{
			printf("[ERROR]: No Configured Hats!\n");
		}
		for(std::size_t i = 0; i < hats.size(); ++i)
		{
			printf("\t[%d] PN=%s DeviceName=%s\n",(int)i,hats.at(i).PartNumber.c_str(),hats.at(i).DeviceName.c_str());
		}
		print_diagnostic(INFO,diag);
	}
	EXPECT_TRUE(process->is_ready() == true);
	return process;
}
TEST(Template, Process_Initialization_ServoHat)
{
	HatControllerNodeProcess *process = initializeprocess();
	std::vector<HatControllerNodeProcess::HatMap> supported_hats = process->get_allsupportedhats();
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
	servohat1_device.PartNumber = PN_625004;
	servohat1_device.DeviceType = DEVICETYPE_SERVOHAT;
	servohat1_device.BoardCount = 0;
	servohat1_device.ID = 64;
	servohat1_device = initialize_device(servohat1_device, supported_hats);

	process->set_mydevice(ros_device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);

	eros::device::ConstPtr dev_ptr(new eros::device(servohat1_device));
	eros::diagnostic diagnostic = process->new_devicemsg(dev_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	process = readyprocess(process);
}
TEST(Template, Process_Initialization_GPIOHat)
{
	HatControllerNodeProcess *process = initializeprocess();
	std::vector<HatControllerNodeProcess::HatMap> supported_hats = process->get_allsupportedhats();
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
	gpiohat1_device.DeviceType = DEVICETYPE_GPIOHAT;
	gpiohat1_device.PartNumber = PN_100007;
	gpiohat1_device.BoardCount = 0;
	gpiohat1_device.ID = 64;
	gpiohat1_device = initialize_device(gpiohat1_device, supported_hats);

	process->set_mydevice(ros_device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);

	eros::device::ConstPtr dev_ptr(new eros::device(gpiohat1_device));
	eros::diagnostic diagnostic = process->new_devicemsg(dev_ptr);

	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	process = readyprocess(process);

	diagnostic = process->set_hat_running(gpiohat1_device.DeviceType,gpiohat1_device.ID);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_hat_running(gpiohat1_device.DeviceType,gpiohat1_device.ID));

	//Check Pins, should all be Default Values
	{
		std::vector<eros::pin> pins = process->get_gpiohatpins(gpiohat1_device.ID);
		EXPECT_TRUE(pins.size() == gpiohat1_device.pins.size());
		for(std::size_t i = 0; i < pins.size(); ++i)
		{
			EXPECT_TRUE(pins.at(i).Value == gpiohat1_device.pins.at(i).DefaultValue);
		}
	}

	//New Message for DIO Port 1
	int test_count = 7;
	for(int i = 0; i < test_count; ++i)
	{
		uint8_t port_width = 4;
		uint16_t v[port_width];
		int index = 0;
		for(int j = 0; j < port_width; ++j)
		{
			v[index] = (i*200) + (j+200);
		}
		diagnostic = process->new_message_GetDIOPort1(gpiohat1_device.DeviceType,gpiohat1_device.ID,(double)(i)*0.1,v[0],v[1],v[2],v[3]);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		std::vector<eros::pin> pins = process->get_pins_byport(gpiohat1_device.DeviceType,gpiohat1_device.ID,PORT_DIGIPORT_1);
		EXPECT_TRUE((int)pins.size() == port_width);
		for(std::size_t j = 0; j < pins.size(); ++j)
		{
			EXPECT_TRUE(pins.at(j).Value == v[j]);
		}
	}
	test_count = 5;
	for(int i = 0; i < test_count; ++i)
	{
		uint8_t port_width = 6;
		uint16_t v[port_width];
		int index = 0;
		for(int j = 0; j < port_width; ++j)
		{
			v[index] = (i*50) + (j+75);
		}
		diagnostic = process->new_message_GetANAPort1(gpiohat1_device.DeviceType,gpiohat1_device.ID,(double)(i)*0.1,v[0],v[1],v[2],v[3],v[4],v[5]);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		std::vector<eros::pin> pins = process->get_pins_byport(gpiohat1_device.DeviceType,gpiohat1_device.ID,PORT_ANAPORT_1);
		EXPECT_TRUE((int)pins.size() == port_width);
		for(std::size_t j = 0; j < pins.size(); ++j)
		{
			if(pins.at(j).Name != "")
			{
				EXPECT_TRUE(pins.at(j).Value == v[j]);
			}
		}
	}
	test_count = 5;
	for(int i = 0; i < test_count; ++i)
	{
		uint8_t port_width = 6;
		uint16_t v[port_width];
		int index = 0;
		for(int j = 0; j < port_width; ++j)
		{
			v[index] = (i*17) + (j+33);
		}
		diagnostic = process->new_message_GetANAPort2(gpiohat1_device.DeviceType,gpiohat1_device.ID,(double)(i)*0.1,v[0],v[1],v[2],v[3],v[4],v[5]);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		std::vector<eros::pin> pins = process->get_pins_byport(gpiohat1_device.DeviceType,gpiohat1_device.ID,PORT_ANAPORT_2);
		for(std::size_t j = 0; j < pins.size(); ++j)
		{
			if(pins.at(j).Name != "")
			{
				EXPECT_TRUE(pins.at(j).Value == v[j]);
			}
		}
	}

}

TEST(Template, Process_Command)
{
	HatControllerNodeProcess *process = initializeprocess();
	std::vector<HatControllerNodeProcess::HatMap> supported_hats = process->get_allsupportedhats();
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
	gpiohat1_device.DeviceType = DEVICETYPE_GPIOHAT;
	gpiohat1_device.PartNumber = PN_100007;
	gpiohat1_device.BoardCount = 0;
	gpiohat1_device.ID = 64;
	gpiohat1_device = initialize_device(gpiohat1_device, supported_hats);

	process->set_mydevice(ros_device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);

	eros::device::ConstPtr dev_ptr(new eros::device(gpiohat1_device));
	eros::diagnostic diagnostic = process->new_devicemsg(dev_ptr);

	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	process = readyprocess(process);
	double time_to_run = 40.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false;   //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false;   //0.1 Hz
	while (current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt, current_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
		int current_time_ms = (int)(current_time * 1000.0);
		if ((current_time_ms % 100) == 0)
		{
			fastrate_fire = true;
		}
		else
		{
			fastrate_fire = false;
		}
		if ((current_time_ms % 1000) == 0)
		{
			mediumrate_fire = true;
		}
		else
		{
			mediumrate_fire = false;
		}
		if ((current_time_ms % 10000) == 0)
		{
			slowrate_fire = true;
		}
		else
		{
			slowrate_fire = false;
		}
		eros::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
		if (fastrate_fire == true)
		{
			cmd.Option1 = LEVEL1;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for (std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);
		}
		if (mediumrate_fire == true)
		{
			EXPECT_TRUE(diag.Level <= NOTICE);
			cmd.Option1 = LEVEL2;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for (std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);
		}
		if (slowrate_fire == true)
		{
			std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
			EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT + process->get_hats().size() + process->get_sensordata().size()));
			for (std::size_t i = 0; i < diagnostics.size(); ++i)
			{
				print_diagnostic(WARN, diagnostics.at(i));
				EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
			}
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
TEST(DeviceInitialization, DeviceInitialization_TerminalHat)
{
	eros::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = ros_ParentDevice;
	ros_device.DeviceType = ros_DeviceType;
	ros_device.BoardCount = 0;
	ros_device.HatCount = 1;
	ros_device.ID = 123;
	HatControllerNodeProcess *process = initializeprocess(ros_device);
	std::vector<HatControllerNodeProcess::HatMap> supported_hats = process->get_allsupportedhats();
	eros::device terminalhat1_device;
	terminalhat1_device.DeviceName = "TerminalHat1";
	terminalhat1_device.DeviceParent = ros_DeviceName;
	terminalhat1_device.DeviceType = DEVICETYPE_TERMINALHAT;
	terminalhat1_device.PartNumber = PN_625005;
	terminalhat1_device.BoardCount = 0;
	terminalhat1_device.ID = 64;
	terminalhat1_device = initialize_device(terminalhat1_device, supported_hats);

	eros::diagnostic diagnostic = process->update(0.02, 0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	eros::device::ConstPtr hat1_ptr(new eros::device(terminalhat1_device));
	diagnostic = process->new_devicemsg(hat1_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	process = readyprocess(process);
	diagnostic = process->set_terminalhat_initialized();
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	diagnostic = process->update(0.02, 0.04);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	EXPECT_EQ(process->get_ready_to_arm(), true);
	EXPECT_TRUE(process->get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM);

	std::vector<eros::pin> input_pins = process->get_terminalhatpins("DigitalInput", true);
	std::vector<eros::pin> output_nonactuator_pins = process->get_terminalhatpins("DigitalOutput-NonActuator", true);
	std::vector<eros::pin> output_actuator_pins = process->get_terminalhatpins("DigitalOutput", true);
	std::vector<eros::pin> p = process->get_terminalhatpins("DigitalOutput", false);
	EXPECT_TRUE(input_pins.size() > 0);
	//EXPECT_TRUE(input_pins.size() == gpio_input_pins.size());
	//EXPECT_TRUE(output_nonactuator_pins.size() == gpio_output_nonactuator_pins.size());
	//EXPECT_TRUE(output_actuator_pins.size() == gpio_output_actuator_pins.size());
	EXPECT_TRUE((input_pins.size() + output_nonactuator_pins.size() + output_actuator_pins.size()) == terminalhat1_device.pins.size());
	for (std::size_t i = 0; i < output_actuator_pins.size(); i++)
	{
		EXPECT_TRUE(output_actuator_pins.at(i).DefaultValue == output_actuator_pins.at(i).Value);
		//EXPECT_TRUE(output_actuator_pins.at(i).Value == gpio_output_actuator_pins.at(i).DefaultValue);
	}

	diagnostic = process->new_armedstatemsg(ARMEDSTATUS_ARMED);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_armedstate() == ARMEDSTATUS_ARMED);

	output_actuator_pins.clear();
	output_actuator_pins = process->get_terminalhatpins("DigitalOutput", true);

	diagnostic = process->new_armedstatemsg(ARMEDSTATUS_DISARMED);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02, 0.06);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_armedstate() == ARMEDSTATUS_DISARMED);

	output_actuator_pins.clear();
	output_actuator_pins = process->get_terminalhatpins("DigitalOutput", true);

	int value = 0;
	//eros::iopins p;
	for (std::size_t i = 0; i < input_pins.size(); i++)
	{
		eros::pin pin = input_pins.at(i);
		pin.Value = value;
		value = !value;
		//gpio_input_pins.at(i) = pin;
		struct timeval now;
		gettimeofday(&now, NULL);
		pin.stamp = process->convert_time(now);
		usleep(10000);
		eros::pin::ConstPtr pin_ptr(new eros::pin(pin));
		diagnostic = process->new_pinmsg(pin_ptr);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		//p.pins.push_back(pin);
	}

	input_pins.clear();
	input_pins = process->get_terminalhatpins("DigitalInput", true);
	//EXPECT_TRUE(input_pins.size() == gpio_input_pins.size());
	std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
	for (std::size_t i = 0; i < diagnostics.size(); ++i)
	{
		print_diagnostic(WARN, diagnostics.at(i));
		EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
	}
}
TEST(DeviceInitialization,DeviceInitialization_EverySupportedHat)
{
	
	eros::device ros_device;
	ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = ros_ParentDevice;
	ros_device.DeviceType = ros_DeviceType;
	ros_device.HatCount = HatControllerNodeProcess::SUPPORTED_HATCOUNT;
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
	std::vector<HatControllerNodeProcess::HatMap> supported_hats = process->get_allsupportedhats();
	for(std::size_t i = 0; i < supported_hats.size(); ++i)
	{
		std::vector<std::string> pin_names;
		for(std::size_t j = 0; j < supported_hats.at(i).PinMap.size(); ++j)
		{
			pin_names.push_back(supported_hats.at(i).PinMap.at(j).PinName);
		}
		for(std::size_t j = 0; j < pin_names.size(); ++j)
		{
			std::string search_pinname = pin_names.at(j);
			uint64_t pin_match_count = 0;
			for(std::size_t k = 0; k < pin_names.size(); ++k)
			{
				if(pin_names.at(k) == search_pinname)
				{
					pin_match_count++;
				}
			}
			if(pin_match_count != 1)
			{
				printf("HatMap: PN=%s has an invalid PinMap definition for Pin: %s\n",
					supported_hats.at(i).FAST_PN.c_str(),
					search_pinname.c_str());
				EXPECT_TRUE(pin_match_count == 1);
			}
		}
	}
	EXPECT_TRUE(supported_hats.size() == (uint16_t)HatControllerNodeProcess::SUPPORTED_HATCOUNT);
	process->set_analyzetiming(true);
	EXPECT_EQ(process->get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
	EXPECT_EQ(process->get_ready_to_arm(),false);

	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	
	
	eros::device hat1;
	hat1.DeviceName = "ServoHat1";
	hat1.DeviceParent = ros_DeviceName;
	hat1.PartNumber = PN_625004;
	hat1.DeviceType = DEVICETYPE_SERVOHAT;
	hat1.BoardCount = 0;
	hat1.ID = 64;
	hat1 = initialize_device(hat1, supported_hats);

	
	eros::device hat2;
	hat2.DeviceName = "GPIOHat1";
	hat2.DeviceType = DEVICETYPE_GPIOHAT;
	hat2.DeviceParent = ros_DeviceName;
	hat2.PartNumber = PN_100007;
	hat2.ID = 2;
	hat2.Architecture = "None";
	hat2.ShieldCount = 0;
	hat2 = initialize_device(hat2,supported_hats);
	
	eros::device hat3;
	hat3.DeviceName = "TerminalHat1";
	hat3.DeviceType = DEVICETYPE_TERMINALHAT;
	hat3.PartNumber = PN_625005;
	hat3.DeviceParent = ros_DeviceName;
	hat3.ID = 64;
	hat3.Architecture = "None";
	hat3.ShieldCount = 0;
	hat3.SensorCount = 0;
	hat3 = initialize_device(hat3,supported_hats);

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

	diagnostic = process->set_hat_running(DEVICETYPE_SERVOHAT,0);
	EXPECT_TRUE(diagnostic.Level > NOTICE);

	diagnostic = process->set_hat_running(DEVICETYPE_SERVOHAT,hat1.ID);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02,.04);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_ready_to_arm(),false);

	diagnostic = process->set_hat_running(DEVICETYPE_GPIOHAT,hat2.ID);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02,.06);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_ready_to_arm(),false);

	diagnostic = process->set_hat_running(DEVICETYPE_TERMINALHAT,hat3.ID);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->update(0.02,.08);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_ready_to_arm(),true);

	std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
	//EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT+(2*process->get_boarddata().size())+(process->get_sensordata().size()));
	for (std::size_t i = 0; i < diagnostics.size(); ++i)
	{
		print_diagnostic(WARN,diagnostics.at(i));
		EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
	}
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
