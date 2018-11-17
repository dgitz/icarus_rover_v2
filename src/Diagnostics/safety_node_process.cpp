#include "safety_node_process.h"
/*! \brief Constructor
 */
SafetyNodeProcess::SafetyNodeProcess(std::string _base_node_name,
		std::string _node_name)
{
	base_node_name = _base_node_name;
	node_name = _node_name;
	unittest_running = false;
	run_time = 0.0;
	initialized = false;
	ready = false;

	ready_to_arm = false;
	arm_switch = false;
}
/*! \brief Deconstructor
 */
SafetyNodeProcess::~SafetyNodeProcess()
{

}
icarus_rover_v2::diagnostic SafetyNodeProcess::set_terminalhat_initialized()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	bool found = false;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "TerminalHat"))
		{
			found = true;
			hats_running.at(i) = true;
			break; //Only 1 Terminal Hat supported.
		}
	}
	if(found == false)
	{
		diag.Level = WARN;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[512];
		sprintf(tempstr,"TerminalHat Not Found");
		diag.Description = std::string(tempstr);
	}
	else
	{
		if(initialized == true)
		{
			ready = true;
		}
		diag.Level = INFO;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"TerminalHat is now Initialized");
		diag.Description = std::string(tempstr);
	}
	return diag;
}
int SafetyNodeProcess::get_pinnumber(std::string name)
{
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "TerminalHat"))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				if((hats.at(i).pins.at(j).Name == name))
				{
					return hats.at(i).pins.at(j).Number;
				}
			}
		}
	}
	return -1;
}
bool SafetyNodeProcess::set_pinvalue(std::string name,int v)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "TerminalHat"))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				if((hats.at(i).pins.at(j).Name == name))
				{
					hats.at(i).pins.at(j).Value = v;
					if(name == "ArmSwitch")
					{
						if(v == 1)
						{
							arm_switch = true;
						}
						else
						{

							arm_switch = false;
							diag.Diagnostic_Type = REMOTE_CONTROL;
							diag.Level = WARN;
							diag.Diagnostic_Message = ROVER_DISARMED;
							diagnostic = diag;
						}
					}
					return true;
				}
			}
		}
	}
	return false;
}
std::vector<icarus_rover_v2::pin> SafetyNodeProcess::get_terminalhatpins(std::string Function)
{
	std::vector<icarus_rover_v2::pin> pins;
	pins.clear();
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "TerminalHat"))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				icarus_rover_v2::pin pin;
				if((hats.at(i).pins.at(j).Function == Function) or (Function == ""))
				{
					pin = hats.at(i).pins.at(j);
					pins.push_back(pin);
				}
			}
		}
	}
	return pins;
}
icarus_rover_v2::diagnostic SafetyNodeProcess::new_armswitchmsg(std_msgs::Bool v)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	arm_switch = v.data;

	diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "ArmSwitch Updated";
	return diag;

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic SafetyNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{

	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;

	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic SafetyNodeProcess::update(double dt)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	run_time += dt;
	if(ready == true)
	{
		diag.Level = INFO;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Node Running";
		if((arm_switch == true))
		{
			ready_to_arm = true;
		}
		else
		{
			ready_to_arm = false;
		}
	}
	diagnostic = diag;



	return diag;
}
bool SafetyNodeProcess::hat_present(icarus_rover_v2::device device)
{
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceName == device.DeviceName))
		{
			return true;
		}
	}
	return false;
}
bool SafetyNodeProcess::is_hat_running(std::string devicetype,uint16_t id)
{
	bool found = false;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == devicetype) and (hats.at(i).ID == id))
		{
			found = true;
			if(hats_running.at(i) == true)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}
	return false;
}
icarus_rover_v2::diagnostic SafetyNodeProcess::set_hat_running(std::string devicetype,uint16_t id)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::device hat;
	bool found = false;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == devicetype) and (hats.at(i).ID == id))
		{
			found = true;
			hats_running.at(i) = true;
			hat = hats.at(i);
		}
	}
	if(found == false)
	{
		diag.Level = WARN;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[512];
		sprintf(tempstr,"Hat ID: %d Not Found.  Defined Hats: ",id);
		char tempstr2[512];
		for(int j = 0; j < hats.size(); j++)
		{
			sprintf(tempstr2,"%s:%ld,",hats.at(j).DeviceName.c_str(),hats.at(j).ID);
		}
		sprintf(tempstr,"%s%s\n",tempstr,tempstr2);
		diag.Description = std::string(tempstr);
	}
	else
	{
		diag.Level = INFO;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"%s is now Initialized",hat.DeviceName.c_str());
		diag.Description = std::string(tempstr);
	}
	return diag;
}

/*! \brief Setup Process Device info
 */
icarus_rover_v2::diagnostic SafetyNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	if(initialized == false)
	{
	}
	else
	{
		if(ready == false)
		{
			if(newdevice.DeviceParent == myhostname)
			{
				if(hat_present(newdevice) == true)
				{
					diag.Level = WARN;
					diag.Diagnostic_Type = SOFTWARE;
					diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
					char tempstr[512];
					sprintf(tempstr,"Hat: %s already loaded.",
							newdevice.DeviceName.c_str());
					diag.Description = std::string(tempstr);
					return diag;
				}
				std::size_t hat_message = newdevice.DeviceType.find("Hat");
				if(hat_message != std::string::npos)
				{
					if(newdevice.DeviceType == "TerminalHat")
					{
						bool arm_switch_found = false;
						for(std::size_t i = 0; i < newdevice.pins.size(); i++)
						{
							if((newdevice.pins.at(i).Function == "DigitalInput-Safety") and
									(newdevice.pins.at(i).Name == "ArmSwitch"))
							{
								arm_switch_found = true;
							}

						}
						if(arm_switch_found == false)
						{
							diag.Level = ERROR;
							diag.Diagnostic_Type = SOFTWARE;
							diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
							char tempstr[512];
							sprintf(tempstr,"ArmSwitch Not Defined.");
							diag.Description = std::string(tempstr);
							return diag;
						}
					}
					else
					{
						diag.Level = NOTICE;
						diag.Diagnostic_Type = SOFTWARE;
						diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
						char tempstr[512];
						sprintf(tempstr,"Hat Type: %s Not supported.",newdevice.DeviceType.c_str());
						diag.Description = std::string(tempstr);
						return diag;
					}
					hats.push_back(newdevice);
					hats_running.push_back(false);
				}
			}
		}
		else
		{
		}
	}
	diag.Level = INFO;
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Diagnostic_Message = NOERROR;
	char tempstr[512];
	sprintf(tempstr,"Initialized: %d Ready: %d",initialized,ready);
	diag.Description = std::string(tempstr);
	return diag;
}
/*! \brief Process Command Message
 */
std::vector<icarus_rover_v2::diagnostic> SafetyNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
	if (cmd.Command == ROVERCOMMAND_RUNDIAGNOSTIC) {
		if (cmd.Option1 == LEVEL1) {
			diaglist.push_back(diag);
		} else if (cmd.Option1 == LEVEL2) {
			diaglist = check_program_variables();
			return diaglist;
		} else if (cmd.Option1 == LEVEL3) {
			diaglist = run_unittest();
			return diaglist;
		} else if (cmd.Option1 == LEVEL4) {
		}
	}
	return diaglist;
}
/*! \brief Self-Diagnostic-Check Program Variables
 */
std::vector<icarus_rover_v2::diagnostic> SafetyNodeProcess::check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag=diagnostic;
	bool status = true;

	if(status == true)
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED";
		diaglist.push_back(diag);
	}
	else
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED";
		diaglist.push_back(diag);
	}
	return diaglist;
}
/*! \brief Run Unit Test
 */
std::vector<icarus_rover_v2::diagnostic> SafetyNodeProcess::run_unittest() {
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	if (unittest_running == false) {
		unittest_running = true;
		icarus_rover_v2::diagnostic diag = diagnostic;
		bool status = true;
		std::string data;
		std::string cmd =
				"cd ~/catkin_ws && "
						"bash devel/setup.bash && catkin_make run_tests_icarus_rover_v2_gtest_test_"
						+ base_node_name
						+ "_process >/dev/null 2>&1 && "
								"mv /home/robot/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_"
						+ base_node_name
						+ "_process.xml "
								"/home/robot/catkin_ws/build/test_results/icarus_rover_v2/"
						+ base_node_name + "/ >/dev/null 2>&1";
		system(cmd.c_str());
		cmd =
				"cd ~/catkin_ws && bash devel/setup.bash && catkin_test_results build/test_results/icarus_rover_v2/"
						+ base_node_name + "/";
		FILE * stream;

		const int max_buffer = 256;
		char buffer[max_buffer];
		cmd.append(" 2>&1");
		stream = popen(cmd.c_str(), "r");
		if (stream) {
			if (!feof(stream)) {
				if (fgets(buffer, max_buffer, stream) != NULL) {
					data.append(buffer);
				}
				pclose(stream);
			}
		}
		std::vector<std::string> strs;
		std::size_t start = data.find(":");
		data.erase(0, start + 1);
		boost::split(strs, data, boost::is_any_of(",: "),
				boost::token_compress_on);
		if(strs.size() < 6)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[1024];
			sprintf(tempstr,"Unable to process Unit Test Result: %s",data.c_str());
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			return diaglist;
		}
		int test_count = std::atoi(strs.at(1).c_str());
		int error_count = std::atoi(strs.at(3).c_str());
		int failure_count = std::atoi(strs.at(5).c_str());
		if (test_count == 0) {
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			diag.Description = "Test Count: 0.";
			diaglist.push_back(diag);
			status = false;
		}
		if (error_count > 0) {
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[512];
			sprintf(tempstr, "Error Count: %d", error_count);
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			status = false;
		}
		if (failure_count > 0) {
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[512];
			sprintf(tempstr, "Failure Count: %d", failure_count);
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			status = false;
		}
		if (status == true) {
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = NOTICE;
			diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
			diag.Description = "Unit Test -> PASSED";
			diaglist.push_back(diag);
		} else {
			diag.Diagnostic_Type = SOFTWARE;
			uint8_t highest_error = INFO;
			for (std::size_t i = 0; i < diaglist.size(); i++) {
				if (diaglist.at(i).Level > highest_error) {
					highest_error = diaglist.at(i).Level;
				}
			}
			diag.Level = highest_error;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			diag.Description = "Unit Test -> FAILED";
			diaglist.push_back(diag);
		}

		unittest_running = false;
	} else {

		icarus_rover_v2::diagnostic diag = diagnostic;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		diag.Description = "Unit Test -> IS STILL IN PROGRESS";
		diaglist.push_back(diag);
	}
	return diaglist;
}
