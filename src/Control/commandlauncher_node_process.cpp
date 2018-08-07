#include "commandlauncher_node_process.h"
/*! \brief Constructor
 */
CommandLauncherNodeProcess::CommandLauncherNodeProcess()
{
	run_time = 0.0;
	initialized = false;
    ready = false;

	init_processlist();
}
/*! \brief Deconstructor
 */
CommandLauncherNodeProcess::~CommandLauncherNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic CommandLauncherNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
    diagnostic = indiag;
	mydevice.DeviceName = hostname;
	if(load_configfiles() == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Diagnostic_Message = ERROR;
		char tempstr[512];
		sprintf(tempstr,"Unable to load config files.");
		diagnostic.Description = std::string(tempstr);
	}
	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic CommandLauncherNodeProcess::update(double dt)
{
	run_time += dt;
	if(initialized == true) { ready = true; }
	icarus_rover_v2::diagnostic diag = diagnostic;
    bool processes_ok = true;
    if(initialized == true)
    {
    	for(std::size_t i = 0; i < processlist.size(); i++)
    	{
    		if((processlist.at(i).running == true) and (processlist.at(i).initialized == true))
    		{

    		}
    		else if(processlist.at(i).initialized == false)
    		{
    			processes_ok = false;
    			diag.Diagnostic_Type = SOFTWARE;
    			diag.Level = WARN;
    			diag.Diagnostic_Message = INITIALIZING_ERROR;
    			char tempstr[512];
    			sprintf(tempstr,"Unable to start process: %s",processlist.at(i).process_name.c_str());
    			diag.Description = std::string(tempstr);
    		}
    		else if(processlist.at(i).running == false)
    		{
    			processes_ok = false;
    			diag.Diagnostic_Type = SOFTWARE;
    			diag.Level = WARN;
    			diag.Diagnostic_Message = INITIALIZING_ERROR;
    			char tempstr[512];
    			sprintf(tempstr,"Process: %s is Not Running.",processlist.at(i).process_name.c_str());
    			diag.Description = std::string(tempstr);
    		}
    	}

    	if(processlist.size() == 0)
    	{
    		processes_ok = false;
    		diag.Diagnostic_Type = SOFTWARE;
    		diag.Level = WARN;
    		diag.Diagnostic_Message = INITIALIZING_ERROR;
    		char tempstr[512];
    		sprintf(tempstr,"No Processes Found.");
    		diag.Description = std::string(tempstr);

    	}
    	if(processes_ok == true)
    	{
    		diag.Diagnostic_Type = NOERROR;
    		diag.Level = INFO;
    		diag.Diagnostic_Message = NOERROR;
    		diag.Description = "Node Running";
    	}
    }
	diagnostic = diag;
	return diag;
}
/*! \brief Setup Process Device info
 */
icarus_rover_v2::diagnostic CommandLauncherNodeProcess::new_devicemsg(icarus_rover_v2::device device)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
	bool new_device = true;
	if(device.DeviceName == myhostname)
	{

    }
    return diag;
}
/*! \brief Process Command Message
 */
std::vector<icarus_rover_v2::diagnostic> CommandLauncherNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
	if (cmd.Command ==  ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		if(cmd.Option1 == LEVEL1)
		{
		}
		else if(cmd.Option1 == LEVEL2)
		{
			diaglist = check_program_variables();
			return diaglist;
		}
		else if(cmd.Option1 == LEVEL3)
		{
		}
		else if(cmd.Option1 == LEVEL4)
		{
		}
	}
	diaglist.push_back(diag);
	return diaglist;
}
/*! \brief Self-Diagnostic-Check Program Variables
 */
std::vector<icarus_rover_v2::diagnostic> CommandLauncherNodeProcess::check_program_variables()
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
}
bool CommandLauncherNodeProcess::set_process_restarted(std::string name)
{
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		if(processlist.at(i).name == name)
		{
			processlist.at(i).restart_counter++;
		}
	}
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
bool CommandLauncherNodeProcess::set_camerastream(std::string portname)
{
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		if(processlist.at(i).name == "CameraStream")
		{
			/*
			 raspivid -t 999999 -h 480 -w 640 -fps 25 -hf -b 2000000 -o - | gst-launch-1.0 -v fdsrc ! h264parse !  rtph264pay config-interval=1 pt=96 ! gdppay ! tcpserversink host=10.0.0.165 port=12345,
			 */
			uint32_t port = lookup_port(portname);
			if(port == 0) { return false; }
			processlist.at(i).param_uint32_1 = port;
			char tempstr[1024];
			sprintf(tempstr,"raspivid -t 999999 -vf -h 480 -w 640 -fps 25 -hf -b 2000000 -o - "
					"| gst-launch-1.0 -v fdsrc ! h264parse !  rtph264pay config-interval=1 pt=96 ! gdppay ! tcpserversink host=%s port=%d > /dev/null 2>&1 &",
					mydevice.PrimaryIP.c_str(),port);
			processlist.at(i).command_text = std::string(tempstr);
			processlist.at(i).process_name = "raspivid";
			processlist.at(i).pid = 0;
			processlist.at(i).initialized = true;

			return true;
		}
	}
	return false;
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
	{
		ProcessCommand proc;
		proc.name = "CameraStream";
		proc.max_restarts = -1;
		processlist.push_back(proc);
	}
	
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
