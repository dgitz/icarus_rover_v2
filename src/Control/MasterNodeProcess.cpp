#include "MasterNodeProcess.h"
eros::diagnostic MasterNodeProcess::set_filepaths(std::string t_system_filepath, std::string t_device_filepath)
{
	eros::diagnostic diag = root_diagnostic;
	device_filepath = t_device_filepath;
	system_filepath = t_system_filepath;
	diag = update_diagnostic(DATA_STORAGE, INFO, INITIALIZING, "Config File Paths Set");
	return diag;
}
eros::diagnostic MasterNodeProcess::new_devicemsg(__attribute__((unused)) const eros::device::ConstPtr &device)
{
	eros::diagnostic diag = root_diagnostic;
	return diag;
}
eros::diagnostic MasterNodeProcess::finish_initialization()
{
	reset();
	eros::diagnostic diag = root_diagnostic;
	SerialMessageHandler *serialmessagehandler = new SerialMessageHandler;
	device_temperature = -100.0;
	diag = load_devicefile(device_filepath);
	diag = update_diagnostic(diag);
	load_factor.stamp = convert_time(ros_time);
	load_factor.DeviceName = get_mydevice().DeviceName;
	load_factor.loadfactor.push_back(-1.0);
	load_factor.loadfactor.push_back(-1.0);
	load_factor.loadfactor.push_back(-1.0);
	uptime.DeviceName = get_mydevice().DeviceName;
	uptime.stamp = convert_time(ros_time);
	std::string tempstr = exec("nproc",true);
	boost::trim_right(tempstr);
	diag = process_cpucount(tempstr);
	diag = update_diagnostic(diag);
	if (diag.Level > NOTICE)
	{
		return diag;
	}
	diag = load_systemfile(system_filepath);
	diag = update_diagnostic(diag);
	if (diag.Level > NOTICE)
	{
		return diag;
	}
	return diag;
}
eros::diagnostic MasterNodeProcess::process_cpucount(std::string cmd)
{
	eros::diagnostic diag = root_diagnostic;
	processor_count = std::atoi(cmd.c_str());
	diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Processor Count Updated.");
	return diag;
}
eros::diagnostic MasterNodeProcess::process_loadfactor()
{
	eros::diagnostic diag = root_diagnostic;
	{
		std::ifstream file( "/proc/loadavg" );
		if(!file)
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = DROPPING_PACKETS;
			diag.Description = "Unable to read /proc/loadavg";
			diag = update_diagnostic(diag);
			return diag;
		}
		std::string line;
		getline(file,line);
		std::vector <std::string> fields;
		boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
		if(fields.size() != 5)
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = DROPPING_PACKETS;
			diag.Description = "Unable to process /prod/loadavg with: " + line;
			diag = update_diagnostic(diag);
			return diag;
		}
		try
		{
			load_factor.stamp = convert_time(ros_time);
			for(std::size_t i = 0; i < 3; ++i)
			{
				double v = std::atof(fields.at(i).c_str());
				if(v == 0.0)
				{
					char output[512];
					sprintf(output,"Unable to process load factor: %s",line.c_str());
					diag = update_diagnostic(SOFTWARE,ERROR,DROPPING_PACKETS,std::string(output));
					return diag;
				}
				load_factor.loadfactor[i] = v/(double)processor_count;
			}
		}
		catch(std::exception e)
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = DROPPING_PACKETS;
			diag.Description = "Unable to process /prod/loadavg with line: " + line + " exception: " + e.what();
			diag = update_diagnostic(diag);
			return diag;
		}
	}	
	diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Load Factor Updated.");
	return diag;
}
eros::diagnostic MasterNodeProcess::process_uptime()
{
	eros::diagnostic diag = root_diagnostic;
	{
		std::ifstream file( "/proc/uptime" );
		if(!file)
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = DROPPING_PACKETS;
			diag.Description = "Unable to read /proc/uptime";
			diag = update_diagnostic(diag);
			return diag;
		}
		std::string line;
		getline(file,line);
		std::vector <std::string> fields;
		boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
		if(fields.size() != 2)
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = DROPPING_PACKETS;
			diag.Description = "Unable to process /proc/uptime with: " + line;
			diag = update_diagnostic(diag);
			return diag;
		}
		try
		{
			uptime.uptime = (double)(std::atof(fields.at(0).c_str()));
			uptime.stamp = convert_time(ros_time);
			uptime.runtime = run_time;
			diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Uptime Updated.");
		}
		catch(std::exception e)
		{
			diag.Level = ERROR;
			diag.Diagnostic_Message = DROPPING_PACKETS;
			diag.Description = "Unable to process /proc/uptime with: " + line + " and exception: " + e.what();
			diag = update_diagnostic(diag);
			return diag;
		}
	}
	return diag;
}
eros::diagnostic MasterNodeProcess::slowupdate()
{	
	eros::diagnostic diag = root_diagnostic;
	{
		diag = process_loadfactor();
		if(diag.Level > WARN)
		{
			return diag;
		}
		diag = process_uptime();
		if(diag.Level > WARN)
		{
			return diag;
		}
	}
	return diag;
	/*
	std::ifstream file( "/proc/meminfo" );
	if(!file)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Unable to read /proc/meminfo";
		RAMFree_kB = -1;
		return RAMFree_kB;
	}
	int found_entry_count = 0;
	for( std::string line; getline( file, line ); )
	{
		std::vector <string> fields;
	
	 */
	/*
	{
		std::string tempstr = exec("cat /proc/loadavg",true);
		boost::trim_right(tempstr);
		diag =  process_loadfactormsg(tempstr);
	}
	{
		std::string tempstr = exec("cat /proc/uptime",true);
		boost::trim_right(tempstr);
		diag =  process_uptimemsg(tempstr);
	}
	*/
	return diag;
		
}
eros::diagnostic MasterNodeProcess::update(double t_dt, double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	diag = update_baseprocess(t_dt, t_ros_time);
	if (diag.Level <= NOTICE)
	{
		diag = update_diagnostic(SOFTWARE, INFO, NOERROR, "Node Running.");
	}
	if(get_taskstate() == TASKSTATE_RUNNING)
	{
		update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error");
	}
	return diag;
}
std::vector<eros::diagnostic> MasterNodeProcess::new_commandmsg(const eros::command::ConstPtr &t_msg)
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
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		diag = update_diagnostic(diaglist.at(i));
	}
	return diaglist;
}
std::vector<eros::diagnostic> MasterNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true)
	{
		diag = update_diagnostic(SOFTWARE, INFO, DIAGNOSTIC_PASSED, "Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	}
	else
	{
		diag = update_diagnostic(SOFTWARE, WARN, DIAGNOSTIC_FAILED, "Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
bool MasterNodeProcess::new_serialmessage(std::string serialport, std::string baudrate, unsigned char *message, int length)
{
	unsigned char uchar1;
	unsigned long ulong1;
	if (length > 4)
	{
		if (message[0] == 0xAB)
		{
			int id = message[1];
			int packet_length = (int)message[2];
			if ((length - packet_length) == 5)
			{
				int computed_checksum = 0;
				for (int i = 3; i < length - 2; i++)
				{
					computed_checksum ^= message[i];
				}
				if (computed_checksum == message[length - 2])
				{
					if (id == SERIAL_ID_ID)
					{
						serialmessagehandler->decode_IDSerial(message, &uchar1, &ulong1);
						std::string pn = boost::lexical_cast<std::string>(ulong1);
						for (std::size_t i = 0; i < serialports.size(); i++)
						{
							if (serialports.at(i).file == serialport)
							{
								serialports.at(i).baudrate = baudrate;
								serialports.at(i).id = uchar1;
								serialports.at(i).pn = pn;
								serialports.at(i).available = true;
								serialports.at(i).checked = true;
								return true;
							}
						}
					}
				}
			}
		}
	}
	return false;
}
eros::diagnostic MasterNodeProcess::set_serialportlist(std::vector<std::string> list)
{
	eros::diagnostic diag = root_diagnostic;
	std::string serial_usb = "ttyUSB";
	std::string serial_acm = "ttyACM";
	//std::string serial = "ttyS"; //Get rid of this one
	for (std::size_t i = 0; i < list.size(); i++)
	{
		std::string name = list.at(i);
		std::size_t found_usb = name.find(serial_usb);
		std::size_t found_acm = name.find(serial_acm);
		//std::size_t found_serial = name.find(serial);
		if (found_usb != std::string::npos)
		{
			SerialPort port;
			port.porttype = USB;
			port.file = "/dev/" + name;

			serialports.push_back(port);
		}
		else if (found_acm != std::string::npos)
		{
			SerialPort port;
			port.porttype = ACM;
			port.file = "/dev/" + name;
			serialports.push_back(port);
		}
		/*
        else if(found_serial != std::string::npos)
        {
            SerialPort port;
            port.porttype = SERIAL;
            port.file = name;
            serialports.push_back(port);
        }
		 */
		else
		{
			char tempstr[256];
			sprintf(tempstr, "Serial Port: %s is not supported.", name.c_str());
			diag = update_diagnostic(SOFTWARE, FATAL, INITIALIZING_ERROR, std::string(tempstr));
			return diag;
		}
	}
	for (std::size_t i = 0; i < serialports.size(); i++)
	{
		serialports.at(i).id = 0;
		serialports.at(i).pn = "";
		serialports.at(i).available = false;
		serialports.at(i).checked = false;
	}
	diag = update_diagnostic(COMMUNICATIONS, NOTICE, INITIALIZING, "Created SerialPort List.");
	return diag;
}
bool MasterNodeProcess::create_nodelist(std::string nodelist_path, std::string activenode_path)
{
	std::string line;
	std::ifstream allnodelist_file(nodelist_path.c_str());
	std::ofstream process_file;
	process_file.open(activenode_path.c_str());
	if (process_file.is_open() == false)
	{
		return false;
	}
	if (allnodelist_file.is_open())
	{
		while (getline(allnodelist_file, line))
		{
			std::vector<std::string> items;
			boost::split(items, line, boost::is_any_of(":\t"));
			std::string host = items.at(1).substr(2, items.at(1).size());
			if (host_name == host)
			{
				std::vector<std::string> items2;
				boost::split(items2, items.at(3), boost::is_any_of("/"));
				for (std::size_t i = 0; i < items2.size(); i++)
				{
				}
				std::string node = items2.at(items2.size() - 1);
				if (node != "rosout")
				{
					time_t rawtime;
					struct tm *timeinfo;
					char datebuffer[80];
					time(&rawtime);
					timeinfo = localtime(&rawtime);
					strftime(datebuffer, 80, "%d/%m/%Y %I:%M:%S", timeinfo);
					process_file << "Node:\t" << node << "\tLaunched:\t" << datebuffer << std::endl;
				}
			}
		}
		allnodelist_file.close();
	}
	process_file.close();
	return true;
}
eros::diagnostic MasterNodeProcess::load_systemfile(std::string path)
{
	eros::diagnostic diag = root_diagnostic;
	char tempstr[512];
	sprintf(tempstr, "Unable to load: %s", path.c_str());
	TiXmlDocument doc(path);
	bool systemfile_loaded = doc.LoadFile();
	if (systemfile_loaded == false)
	{
		diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
		return diag;
	}
	TiXmlElement *l_pRootElement = doc.RootElement();
	if (NULL != l_pRootElement)
	{
		TiXmlElement *l_pLeverArmList = l_pRootElement->FirstChildElement("LeverArmList");

		if (NULL != l_pLeverArmList)
		{
			TiXmlElement *l_pLeverArm = l_pLeverArmList->FirstChildElement("LeverArm");

			while (l_pLeverArm)
			{
				eros::leverarm la;
				TiXmlElement *l_pName = l_pLeverArm->FirstChildElement("Name");
				if (NULL != l_pName)
				{
					la.name = l_pName->GetText();
				}
				else
				{
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
					return diag;
				}

				TiXmlElement *l_pReference = l_pLeverArm->FirstChildElement("ReferenceLeverArm");
				if (NULL != l_pReference)
				{
					la.reference = l_pReference->GetText();
				}
				else
				{
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
					return diag;
				}

				if (la.reference != "BodyOrigin")
				{
					char tempstr2[512];
					sprintf(tempstr2, "LeverArm: %s Reference: %s Not Supported.", la.name.c_str(), la.reference.c_str());
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr2));
					return diag;
				}

				TiXmlElement *l_pX = l_pLeverArm->FirstChildElement("x");
				if (NULL != l_pX)
				{
					la.x.value = std::atof(l_pX->GetText());
				}
				else
				{
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
					return diag;
				}

				TiXmlElement *l_pY = l_pLeverArm->FirstChildElement("y");
				if (NULL != l_pY)
				{
					la.y.value = std::atof(l_pY->GetText());
				}
				else
				{
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
					return diag;
				}

				TiXmlElement *l_pZ = l_pLeverArm->FirstChildElement("z");
				if (NULL != l_pZ)
				{
					la.z.value = std::atof(l_pZ->GetText());
				}
				else
				{
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
					return diag;
				}

				TiXmlElement *l_pRoll = l_pLeverArm->FirstChildElement("roll");
				if (NULL != l_pRoll)
				{
					la.roll.value = std::atof(l_pRoll->GetText());
				}
				else
				{
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
					return diag;
				}

				TiXmlElement *l_pPitch = l_pLeverArm->FirstChildElement("pitch");
				if (NULL != l_pPitch)
				{
					la.pitch.value = std::atof(l_pPitch->GetText());
				}
				else
				{
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
					return diag;
				}

				TiXmlElement *l_pYaw = l_pLeverArm->FirstChildElement("yaw");
				if (NULL != l_pYaw)
				{
					la.yaw.value = std::atof(l_pYaw->GetText());
				}
				else
				{
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
					return diag;
				}

				leverarms.push_back(la);
				l_pLeverArm = l_pLeverArm->NextSiblingElement("LeverArm");
			}
		}
		else
		{
			diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
			return diag;
		}
	}
	else
	{
		diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
		return diag;
	}
	char tempstr2[512];
	sprintf(tempstr2, "Loaded: %s", path.c_str());
	diag = update_diagnostic(DATA_STORAGE, NOTICE, INITIALIZING, std::string(tempstr2));
	return diag;
}
eros::diagnostic MasterNodeProcess::load_devicefile(std::string path)
{
	eros::diagnostic diag = root_diagnostic;
	TiXmlDocument doc(path);
	bool devicefile_loaded = doc.LoadFile();
	bool mydevice_assigned = false;
	if (devicefile_loaded == false)
	{
		char tempstr[512];
		sprintf(tempstr, "Unable to load: %s", path.c_str());
		diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
		return diag;
	}
	TiXmlElement *l_pRootElement = doc.RootElement();

	if (NULL != l_pRootElement)
	{
		TiXmlElement *l_pDeviceList = l_pRootElement->FirstChildElement("DeviceList");

		if (NULL != l_pDeviceList)
		{
			TiXmlElement *l_pDevice = l_pDeviceList->FirstChildElement("Device");

			while (l_pDevice)
			{
				eros::device newDevice;
				std::vector<eros::pin> pins;
				pins.clear();
				std::vector<std::string> input_topics;
				input_topics.clear();
				std::vector<std::string> output_topics;
				output_topics.clear();
				std::vector<std::string> topic_modes;
				topic_modes.clear();
				TiXmlElement *l_pDeviceParent = l_pDevice->FirstChildElement("ParentDevice");
				if (NULL != l_pDeviceParent)
				{
					newDevice.DeviceParent = l_pDeviceParent->GetText();
				}

				TiXmlElement *l_pDeviceName = l_pDevice->FirstChildElement("DeviceName");
				if (NULL != l_pDeviceName)
				{
					newDevice.DeviceName = l_pDeviceName->GetText();
				}

				TiXmlElement *l_pID = l_pDevice->FirstChildElement("ID");
				if (NULL != l_pID)
				{
					newDevice.ID = atoi(l_pID->GetText());
				}

				TiXmlElement *l_pDeviceType = l_pDevice->FirstChildElement("DeviceType");
				if (NULL != l_pDeviceType)
				{
					newDevice.DeviceType = l_pDeviceType->GetText();
				}

				TiXmlElement *l_pDevicePN = l_pDevice->FirstChildElement("PartNumber");
				if (NULL != l_pDevicePN)
				{
					newDevice.PartNumber = l_pDevicePN->GetText();
				}
				else
				{
					newDevice.PartNumber = "";
				}

				TiXmlElement *l_pDevicePrimaryIP = l_pDevice->FirstChildElement("PrimaryIP");
				if (NULL != l_pDevicePrimaryIP)
				{
					newDevice.PrimaryIP = l_pDevicePrimaryIP->GetText();
				}
				else
				{
					newDevice.PrimaryIP = "";
				}

				TiXmlElement *l_pDeviceArchitecture = l_pDevice->FirstChildElement("Architecture");
				if (NULL != l_pDeviceArchitecture)
				{
					newDevice.Architecture = l_pDeviceArchitecture->GetText();
				}

				TiXmlElement *l_pBoardCount = l_pDevice->FirstChildElement("BoardCount");
				if (NULL != l_pBoardCount)
				{
					newDevice.BoardCount = atoi(l_pBoardCount->GetText());
				}

				TiXmlElement *l_pSensorCount = l_pDevice->FirstChildElement("SensorCount");
				if (NULL != l_pSensorCount)
				{
					newDevice.SensorCount = atoi(l_pSensorCount->GetText());
				}

				TiXmlElement *l_pHatCount = l_pDevice->FirstChildElement("HatCount");
				if (NULL != l_pHatCount)
				{
					newDevice.HatCount = atoi(l_pHatCount->GetText());
				}

				TiXmlElement *l_pShieldCount = l_pDevice->FirstChildElement("ShieldCount");
				if (NULL != l_pShieldCount)
				{
					newDevice.ShieldCount = atoi(l_pShieldCount->GetText());
				}

				TiXmlElement *l_pCapability = l_pDevice->FirstChildElement("Capability");
				std::vector<std::string> capabilities;
				while (l_pCapability)
				{
					std::string capability = l_pCapability->GetText();
					capabilities.push_back(capability);
					l_pCapability = l_pCapability->NextSiblingElement("Capability");
				}
				newDevice.Capabilities = capabilities;
				TiXmlElement *l_pPin = l_pDevice->FirstChildElement("Pin");
				while (l_pPin)
				{
					eros::pin newpin;
					newpin.ParentDevice = newDevice.DeviceName;

					TiXmlElement *l_pPinName = l_pPin->FirstChildElement("Name");
					if (NULL != l_pPinName)
					{
						newpin.Name = l_pPinName->GetText();
					}
					TiXmlElement *l_pPinFunction = l_pPin->FirstChildElement("Function");
					if (NULL != l_pPinFunction)
					{
						newpin.Function = l_pPinFunction->GetText();
					}

					TiXmlElement *l_pPinConnectedDevice = l_pPin->FirstChildElement("ConnectedDevice");
					if (NULL != l_pPinConnectedDevice)
					{
						newpin.ConnectedDevice = l_pPinConnectedDevice->GetText();
					}
					else
					{
						newpin.ConnectedDevice = "";
					}

					TiXmlElement *l_pPinConnectedSensor = l_pPin->FirstChildElement("ConnectedSensor");
					if (NULL != l_pPinConnectedSensor)
					{
						newpin.ConnectedSensor = l_pPinConnectedSensor->GetText();
					}
					else
					{
						newpin.ConnectedSensor = "";
					}

					TiXmlElement *l_pPinDefaultValue = l_pPin->FirstChildElement("DefaultValue");
					if (NULL != l_pPinDefaultValue)
					{
						newpin.DefaultValue = atoi(l_pPinDefaultValue->GetText());
					}
					else
					{
						newpin.DefaultValue = 0;
					}

					TiXmlElement *l_pPinMaxValue = l_pPin->FirstChildElement("MaxValue");
					if (NULL != l_pPinMaxValue)
					{
						newpin.MaxValue = atoi(l_pPinMaxValue->GetText());
					}
					else
					{
						newpin.MaxValue = 0;
					}

					TiXmlElement *l_pPinMinValue = l_pPin->FirstChildElement("MinValue");
					if (NULL != l_pPinMinValue)
					{
						newpin.MinValue = atoi(l_pPinMinValue->GetText());
					}
					else
					{
						newpin.MinValue = 0;
					}

					TiXmlElement *l_pPinAuxTopic = l_pPin->FirstChildElement("AuxTopic");
					if (NULL != l_pPinAuxTopic)
					{
						newpin.AuxTopic = l_pPinAuxTopic->GetText();
					}
					else
					{
						newpin.AuxTopic = "";
					}

					TiXmlElement *l_pPinScaleFactor = l_pPin->FirstChildElement("ScaleFactor");
					if (NULL != l_pPinScaleFactor)
					{
						newpin.ScaleFactor = atof(l_pPinScaleFactor->GetText());
					}
					else
					{
						newpin.ScaleFactor = 1.0;
					}

					l_pPin = l_pPin->NextSiblingElement("Pin");
					pins.push_back(newpin);
				}

				newDevice.pins = pins;
				if (newDevice.DeviceName == mydevice.DeviceName)
				{
					//This is me.
					mydevice = newDevice;
					mydevice_assigned = true;
				}
				allDevices.push_back(newDevice);
				l_pDevice = l_pDevice->NextSiblingElement("Device");
			}
		}
	}
	if (build_childDevices() == false)
	{
		diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, "Couldn't build Child Devices.");
		return diag;
	}
	else if (mydevice_assigned == false)
	{
		diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, "Unable to find my Device in DeviceFile.xml");
		return diag;
	}
	else
	{
		diag = update_diagnostic(DATA_STORAGE, NOTICE, INITIALIZING, "Processed DeviceFile.xml");
		return diag;
	}
}

void MasterNodeProcess::print_device(std::vector<eros::device> devices)
{
	for (std::size_t i = 0; i < devices.size(); i++)
	{
		print_device(devices.at(i));
	}
}
void MasterNodeProcess::print_device(eros::device device)
{
	printf("Device: %s\n", device.DeviceName.c_str());
}
void MasterNodeProcess::print_leverarm(std::vector<eros::leverarm> leverarms)
{
	for (std::size_t i = 0; i < leverarms.size(); i++)
	{
		print_leverarm(leverarms.at(i));
	}
}
void MasterNodeProcess::print_leverarm(eros::leverarm la)
{
	printf("LeverArm %s Reference: %s X: %f Y: %f Z: %f Roll: %f Pitch: %f Yaw: %f\n",
		   la.name.c_str(),
		   la.reference.c_str(),
		   la.x.value, la.y.value, la.z.value,
		   la.roll.value, la.pitch.value, la.yaw.value);
}
void MasterNodeProcess::print_leverarm(std::string name, std::string reference, eros::leverarm leverarm)
{
	printf("LeverArm %s Reference: %s X: %f Y: %f Z: %f Roll: %f Pitch: %f Yaw: %f\n",
		   name.c_str(),
		   reference.c_str(),
		   leverarm.x.value, leverarm.y.value, leverarm.z.value,
		   leverarm.roll.value, leverarm.pitch.value, leverarm.yaw.value);
}
bool MasterNodeProcess::get_leverarm(eros::leverarm *leverarm, std::string name)
{
	for (std::size_t i = 0; i < leverarms.size(); i++)
	{
		if (leverarms.at(i).name == name)
		{
			*leverarm = leverarms.at(i);
			return true;
		}
	}
	return false;
}
bool MasterNodeProcess::build_childDevices()
{
	for (std::size_t i = 0; i < allDevices.size(); i++)
	{
		//printf("a: %s\n",allDevices.at(i).DeviceName.c_str());
		bool add_child = false;
		std::string local_parent = allDevices.at(i).DeviceParent;
		if (local_parent == host_name)
		{
			add_child = true;
		}
		else
		{
			int level_counter = 0;
			bool search = true;
			bool new_local_parent_found = false;
			while (search == true)
			{
				//printf("local: %s\n",local_parent.c_str());

				level_counter++;
				if (level_counter > 25)
				{
					printf("[MasterNode]: Searched more than 25 levels. Exiting.\n");
					search = false;
					return false;
				}

				for (std::size_t j = 0; j < allDevices.size(); j++)
				{
					if (allDevices.at(j).DeviceParent == "None")
					{
						break;
					}
					if (local_parent == "None")
					{
						search = false;
						break;
					}
					if (local_parent == host_name)
					{
						add_child = true;
						search = false;
						break;
					}
					if (local_parent == allDevices.at(j).DeviceName)
					{
						local_parent = allDevices.at(j).DeviceParent;
						new_local_parent_found = true;
						break;
					}
					//if(allDevices.at(j).
				}
				if (new_local_parent_found == false)
				{
					search = false;
					break;
				}
			}
		}
		if (add_child == true)
		{
			childDevices.push_back(allDevices.at(i));
		}
	}
	for (std::size_t i = 0; i < childDevices.size(); i++)
	{
		std::string baudrate = "";
		bool serialdevice = false;
		if ((childDevices.at(i).PartNumber == "110012") ||
			(childDevices.at(i).PartNumber == "810090"))
		{
			serialdevice = true;
			baudrate = "115200";
		}
		if (serialdevice)
		{
			bool add = true;
			for (std::size_t i = 0; i < serialport_baudrates.size(); i++)
			{
				if (serialport_baudrates.at(i) == baudrate)
				{
					add = false;
					break;
				}
			}
			if (add)
			{
				serialport_baudrates.push_back(baudrate);
			}
		}
	}
	return true;
}
