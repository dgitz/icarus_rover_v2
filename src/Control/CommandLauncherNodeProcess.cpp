#include "CommandLauncherNodeProcess.h"
eros::diagnostic  CommandLauncherNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	reset();
	init_processlist();
	if(load_configfiles() == false)
	{
		char tempstr[512];
		sprintf(tempstr,"Unable to load config files.");
		diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
	}
	return diag;
}
eros::diagnostic CommandLauncherNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if(task_state == TASKSTATE_PAUSE)
	{

	}
	else if(task_state == TASKSTATE_RESET)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if(v == false)
		{
			diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,
				"Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
		
	}
	else if(task_state == TASKSTATE_INITIALIZED)
	{
		request_statechange(TASKSTATE_RUNNING);
	}
	else if(task_state != TASKSTATE_RUNNING)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if(v == false)
		{
			diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,
				"Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
	}
	diag = update_baseprocess(t_dt,t_ros_time);
	bool processes_ok = true;
	if(task_state == TASKSTATE_RUNNING)
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error.");
		for(std::size_t i = 0; i < processlist.size(); i++)
		{
			if((processlist.at(i).running == true) and (processlist.at(i).initialized == true))
			{

			}
			else if(processlist.at(i).initialized == false)
			{
				processes_ok = false;
				char tempstr[512];
				sprintf(tempstr,"Unable to start process: %s",processlist.at(i).process_name.c_str());
				diag = update_diagnostic(SOFTWARE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
			}
			else if(processlist.at(i).running == false)
			{
				processes_ok = false;
				char tempstr[512];
				sprintf(tempstr,"Process: %s is Not Running.",processlist.at(i).process_name.c_str());
				diag = update_diagnostic(SOFTWARE,WARN,DEVICE_NOT_AVAILABLE,std::string(tempstr));
			}
		}

		if(processlist.size() == 0)
		{
			processes_ok = false;
			char tempstr[512];
			sprintf(tempstr,"No Processes Found.");
			diag = update_diagnostic(SOFTWARE,NOTICE,INITIALIZING_ERROR,std::string(tempstr));

		}
		if(processes_ok == true)
		{
			diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running.");
		}
	}
	return diag;
}
eros::diagnostic CommandLauncherNodeProcess::new_devicemsg(__attribute__((unused)) const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = root_diagnostic;
	return diag;
}
std::vector<eros::diagnostic> CommandLauncherNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	if (t_msg->Command == ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		if (t_msg->Option1 == LEVEL1)
		{
			diaglist.push_back(diag);
		}
		else if (t_msg->Option1 == LEVEL2)
		{
			diaglist = check_programvariables();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL3)
		{
			diaglist = run_unittest();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL4)
		{
		}
	}
	else if(t_msg->Command == ROVERCOMMAND_TASKCONTROL)
	{
		if(node_name.find(t_msg->CommandText) != std::string::npos)
		{
			uint8_t prev_taskstate = get_taskstate();
			bool v = request_statechange(t_msg->Option2);
			if(v == false)
			{
				diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,
					"Unallowed State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}
			else
			{
				if(task_state == TASKSTATE_RESET)
				{
					reset();
				}
				diag = update_diagnostic(SOFTWARE,NOTICE,DIAGNOSTIC_PASSED,
					"Commanded State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}

		}
	}
	return diaglist;
}
std::vector<eros::diagnostic> CommandLauncherNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true) {
		diag = update_diagnostic(SOFTWARE,INFO,DIAGNOSTIC_PASSED,"Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	} else {
		diag = update_diagnostic(SOFTWARE,WARN,DIAGNOSTIC_FAILED,"Checked Program Variables -> FAILED.");;
		diaglist.push_back(diag);
	}
	return diaglist;
}
bool CommandLauncherNodeProcess::set_processrunning(std::string name,bool running)
{
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		if(processlist.at(i).name == name)
		{
			processlist.at(i).running = running;
			return true;
		}
	}
	return false;
}
bool CommandLauncherNodeProcess::set_processpid(std::string name,uint32_t pid)
{
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		if(processlist.at(i).name == name)
		{
			processlist.at(i).pid = pid;
			return true;
		}
	}
	return false;
}
bool CommandLauncherNodeProcess::set_process_restarted(std::string name)
{
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		if(processlist.at(i).name == name)
		{
			processlist.at(i).restart_counter++;
			return true;
		}
	}
	return false;
}
std::string CommandLauncherNodeProcess::get_processinfo()
{
	char tempstr[2048];
	tempstr[0] = 0;
	//sprintf(tempstr,"");
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		sprintf(tempstr,"%sProcess: %s Init: %d Running: %d PID: %d Restarted: %d",tempstr,
				processlist.at(i).name.c_str(),
				processlist.at(i).initialized,
				processlist.at(i).running,
				processlist.at(i).pid,
				processlist.at(i).restart_counter);
	}
	return std::string(tempstr);
}
std::string CommandLauncherNodeProcess::lookup_deviceIP(std::string hostname)
{
	for(std::size_t i = 0; i < ipmap.size(); i++)
	{
		if(ipmap.at(i).hostname == hostname)
		{
			return ipmap.at(i).IPAddress;
		}
	}
	return "";
}
uint32_t CommandLauncherNodeProcess::lookup_port(std::string portname)
{
	for(std::size_t i = 0; i < portmap.size(); i++)
	{
		if(portmap.at(i).name == portname)
		{
			return portmap.at(i).port;
		}
	}
	return 0;
}
void CommandLauncherNodeProcess::init_processlist()
{
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		processlist.at(i).running = false;
		processlist.at(i).initialized = false;
		processlist.at(i).param_string_1 = "";
		processlist.at(i).param_uint32_1 = 0;
		processlist.at(i).process_name = "";
		processlist.at(i).pid = 0;
		processlist.at(i).restart_counter = 0;
		processlist.at(i).command_text = "";
	}
}
bool CommandLauncherNodeProcess::load_configfiles()
{
	{
		TiXmlDocument device_doc("/home/robot/config/DeviceFile.xml");
		bool devicefile_loaded = device_doc.LoadFile();
		if(devicefile_loaded == false) { return false; }
		TiXmlElement *l_pRootElement = device_doc.RootElement();

		if( NULL != l_pRootElement )
		{
			TiXmlElement *l_pDeviceList = l_pRootElement->FirstChildElement( "DeviceList" );
			if ( NULL != l_pDeviceList )
			{
				TiXmlElement *l_pDevice = l_pDeviceList->FirstChildElement( "Device" );
				while( l_pDevice )
				{
					std::string name,ipaddress;
					name = "";
					ipaddress = "";
					TiXmlElement *l_pDeviceName = l_pDevice->FirstChildElement( "DeviceName" );
					if ( NULL != l_pDeviceName )
					{
						name = l_pDeviceName->GetText();
					}
					TiXmlElement *l_pDeviceIP = l_pDevice->FirstChildElement( "PrimaryIP" );
					if ( NULL != l_pDeviceIP )
					{
						ipaddress = l_pDeviceIP->GetText();
					}

					if((name != "") and (ipaddress != ""))
					{
						IPMap ip;
						ip.hostname = name;
						ip.IPAddress = ipaddress;
						ipmap.push_back(ip);
					}

					l_pDevice = l_pDevice->NextSiblingElement( "Device" );
				}
			}
		}
	}
	{
		TiXmlDocument miscconfig_doc("/home/robot/config/MiscConfig.xml");
		bool miscconfig_loaded = miscconfig_doc.LoadFile();
		if(miscconfig_loaded == false) { return false; }
		TiXmlElement *l_pRootElement = miscconfig_doc.RootElement();

		if( NULL != l_pRootElement )
		{
			TiXmlElement *l_pPortList = l_pRootElement->FirstChildElement( "PortList" );
			if ( NULL != l_pPortList )
			{
				TiXmlElement *l_pPort = l_pPortList->FirstChildElement( "Port" );
				while( l_pPort )
				{
					std::string name;
					uint32_t number;
					name = "";
					number = 0;
					TiXmlElement *l_pPortName = l_pPort->FirstChildElement( "Name" );
					if ( NULL != l_pPortName )
					{
						name = l_pPortName->GetText();
					}
					TiXmlElement *l_pNumber = l_pPort->FirstChildElement( "Number" );
					if ( NULL != l_pNumber )
					{
						number = atoi(l_pNumber->GetText());
					}

					if((name != "") and (number != 0))
					{
						PortMap port;
						port.name = name;
						port.port = number;
						portmap.push_back(port);
					}

					l_pPort = l_pPort->NextSiblingElement( "Port" );
				}
			}
		}
	}
	return true;
}
