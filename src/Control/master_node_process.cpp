#include "master_node_process.h"
/*! \brief Constructor
 */
MasterNodeProcess::MasterNodeProcess()
{
	initialized = false;
	run_time = 0.0;
	SerialMessageHandler *serialmessagehandler = new SerialMessageHandler;
}
/*! \brief Deconstructor
 */
MasterNodeProcess::~MasterNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic MasterNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic MasterNodeProcess::update(double dt)
{
	run_time += dt;
	icarus_rover_v2::diagnostic diag = diagnostic;
	diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "Node Running";
	diagnostic = diag;
	return diag;
}
/*! \brief Process Command Message
 */
std::vector<icarus_rover_v2::diagnostic> MasterNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
	if (cmd.Command ==  DIAGNOSTIC_ID)
	{
		if(cmd.Option1 == LEVEL1)
		{
			diaglist.push_back(diag);
			return diaglist;
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
std::vector<icarus_rover_v2::diagnostic> MasterNodeProcess::check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag=diagnostic;
	bool status = true;
	if(initialized == false)
	{
		status = false;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = INITIALIZING;
		diag.Description = "Node Not Initialized Yet.";
		diaglist.push_back(diag);
	}
	if(allDevices.size() == 0)
	{
		status = false;
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "No Devices read at all in DeviceFile.xml";
		diaglist.push_back(diag);
	}
	if(status == true)
	{

		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = NOTICE;
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
bool MasterNodeProcess::new_serialmessage(std::string serialport,std::string baudrate,unsigned char* message,int length)
{
	unsigned char uchar1;
	unsigned long ulong1;
	if(length > 4)
	{
		if(message[0] == 0xAB)
		{
			int id = message[1];
			int packet_length = (int)message[2];
			if((length-packet_length) == 5)
			{
				int computed_checksum = 0;
				for(int i = 3; i < length - 2; i++)
				{
					computed_checksum ^= message[i];
				}
				if(computed_checksum == message[length-2])
				{
					if(id == SERIAL_ID_ID)
					{
						serialmessagehandler->decode_IDSerial(message,&uchar1,&ulong1);
						std::string pn = boost::lexical_cast<std::string>(ulong1);
						for(std::size_t i = 0; i < serialports.size(); i++)
						{
							if(serialports.at(i).file == serialport)
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
icarus_rover_v2::diagnostic MasterNodeProcess::set_serialportlist(std::vector<std::string> list)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    std::string serial_usb = "ttyUSB";
    std::string serial_acm = "ttyACM";
    //std::string serial = "ttyS"; //Get rid of this one
    for(std::size_t i = 0; i < list.size(); i++)
    {
        std::string name = list.at(i);
        std::size_t found_usb = name.find(serial_usb);
        std::size_t found_acm = name.find(serial_acm);
        //std::size_t found_serial = name.find(serial);
        if(found_usb != std::string::npos)
        {
            SerialPort port;
            port.porttype = USB;
            port.file = "/dev/" + name;
            
            serialports.push_back(port);
        }
        else if(found_acm != std::string::npos)
        {
            SerialPort port;
            port.porttype = ACM;
            port.file =  "/dev/" + name;
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
            diag.Diagnostic_Type = SOFTWARE;
            diag.Level = FATAL;
            diag.Diagnostic_Message = INITIALIZING_ERROR;
            char tempstr[256];
            sprintf(tempstr,"Serial Port: %s is not supported.",name.c_str());
            diag.Description = std::string(tempstr);
            return diag;
        }
        
    }
    for(std::size_t i = 0; i < serialports.size(); i++)
    {
        serialports.at(i).id = 0;
        serialports.at(i).pn = "";
        serialports.at(i).available = false;
        serialports.at(i).checked = false;
    }
    diag.Diagnostic_Type = NOERROR;
    diag.Level = NOTICE;
    diag.Diagnostic_Message = INITIALIZING;
    diag.Description = "Created SerialPort List.";
    return diag;
}
bool MasterNodeProcess::update_nodelist(std::string nodelist_path,std::string activenode_path)
{
    std::string line;
    std::ifstream allnodelist_file(nodelist_path.c_str());
    std::ofstream process_file;
    process_file.open(activenode_path.c_str());
    if(process_file.is_open() == false)
   	{
   		return false;

   	}
    if(allnodelist_file.is_open())
    {
    	while(getline(allnodelist_file,line))
    	{
    		std::vector<std::string> items;
    		boost::split(items,line,boost::is_any_of(":\t"));
    		std::string host = items.at(1).substr(2,items.at(1).size());
    		if(myhostname == host)
    		{
    			std::vector<std::string> items2;
    			boost::split(items2,items.at(3),boost::is_any_of("/"));
    			for(int i = 0; i < items2.size();i++)
    			{
    			}
    			std::string node = items2.at(items2.size()-1);
    			if(node != "rosout")
    			{
    				time_t rawtime;
    				struct tm * timeinfo;
    				char datebuffer[80];
    				time (&rawtime);
    				timeinfo = localtime(&rawtime);
    				strftime(datebuffer,80,"%d/%m/%Y %I:%M:%S",timeinfo);
    				process_file << "Node:\t" << node << "\tLaunched:\t" << datebuffer <<  std::endl;
    			}
    		}
    	}
    	allnodelist_file.close();
    }
    process_file.close();
    return true;
}
icarus_rover_v2::diagnostic MasterNodeProcess::load_systemfile(std::string path)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::diagnostic diag_error = diag;
	diag_error.Diagnostic_Type = SOFTWARE;
	diag_error.Level = FATAL;
	diag_error.Diagnostic_Message = INITIALIZING_ERROR;
	char tempstr[512];
	sprintf(tempstr,"Unable to load: %s",path.c_str());
	diag_error.Description = std::string(tempstr);

	TiXmlDocument doc(path);
	bool systemfile_loaded = doc.LoadFile();
	if(systemfile_loaded == false)
	{
		return diag_error;
	}
	TiXmlElement *l_pRootElement = doc.RootElement();
	if( NULL != l_pRootElement )
	{
		TiXmlElement *l_pLeverArmList = l_pRootElement->FirstChildElement( "LeverArmList" );

		if ( NULL != l_pLeverArmList )
		{
			TiXmlElement *l_pLeverArm = l_pLeverArmList->FirstChildElement( "LeverArm" );

			while( l_pLeverArm )
			{
				LeverArm la;
				TiXmlElement *l_pName = l_pLeverArm->FirstChildElement( "Name" );
				if ( NULL != l_pName )
				{
					la.name = l_pName->GetText();
				}
				else { return diag_error; }

				TiXmlElement *l_pReference = l_pLeverArm->FirstChildElement( "ReferenceLeverArm" );
				if ( NULL != l_pReference )
				{
					la.reference = l_pReference->GetText();
				}
				else { return diag_error; }
				if(la.reference != "BodyOrigin")
				{
					char tempstr2[512];
					sprintf(tempstr2,"LeverArm: %s Reference: %s Not Supported.",la.name.c_str(),la.reference.c_str());
					diag_error.Description = std::string(tempstr2);
					return diag_error;
				}

				TiXmlElement *l_pX = l_pLeverArm->FirstChildElement( "x" );
				if ( NULL != l_pX )
				{
					la.leverarm.x.value = std::atof(l_pX->GetText());
				}
				else { return diag_error; }

				TiXmlElement *l_pY = l_pLeverArm->FirstChildElement( "y" );
				if ( NULL != l_pY )
				{
					la.leverarm.y.value = std::atof(l_pY->GetText());
				}
				else { return diag_error; }

				TiXmlElement *l_pZ = l_pLeverArm->FirstChildElement( "z" );
				if ( NULL != l_pZ )
				{
					la.leverarm.z.value = std::atof(l_pZ->GetText());
				}
				else { return diag_error; }

				TiXmlElement *l_pRoll = l_pLeverArm->FirstChildElement( "roll" );
				if ( NULL != l_pRoll )
				{
					la.leverarm.roll.value = std::atof(l_pRoll->GetText());
				}
				else { return diag_error; }

				TiXmlElement *l_pPitch = l_pLeverArm->FirstChildElement( "pitch" );
				if ( NULL != l_pPitch )
				{
					la.leverarm.pitch.value = std::atof(l_pPitch->GetText());
				}
				else { return diag_error; }

				TiXmlElement *l_pYaw = l_pLeverArm->FirstChildElement( "yaw" );
				if ( NULL != l_pYaw )
				{
					la.leverarm.yaw.value = std::atof(l_pYaw->GetText());
				}
				else { return diag_error; }

				leverarms.push_back(la);
				l_pLeverArm = l_pLeverArm->NextSiblingElement( "LeverArm" );
			}
		}
		else { return diag_error; }
	}
	else { return diag_error; }

	diag.Diagnostic_Type = NOERROR;
	diag.Level = NOTICE;
	diag.Diagnostic_Message = INITIALIZING;
	char tempstr2[512];
	sprintf(tempstr2,"Loaded: %s",path.c_str());
	diag.Description = std::string(tempstr2);
	return diag;
}
icarus_rover_v2::diagnostic MasterNodeProcess::load_devicefile(std::string path)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    TiXmlDocument doc(path);
    bool devicefile_loaded = doc.LoadFile();
    bool mydevice_assigned = false;
    if(devicefile_loaded == false)
    {
        diag.Diagnostic_Type = SOFTWARE;
        diag.Level = FATAL;
        diag.Diagnostic_Message = INITIALIZING_ERROR;
        char tempstr[512];
        sprintf(tempstr,"Unable to load: %s",path.c_str());
        diag.Description = std::string(tempstr);
        return diag;
    }
    TiXmlElement *l_pRootElement = doc.RootElement();

	if( NULL != l_pRootElement )
	{
	    TiXmlElement *l_pDeviceList = l_pRootElement->FirstChildElement( "DeviceList" );

	    if ( NULL != l_pDeviceList )
	    {
	        TiXmlElement *l_pDevice = l_pDeviceList->FirstChildElement( "Device" );

	        while( l_pDevice )
	        {
	        	icarus_rover_v2::device newDevice;
	        	std::vector<icarus_rover_v2::pin> pins;
                pins.clear();
				std::vector<std::string> input_topics;
                input_topics.clear();
				std::vector<std::string> output_topics;
                output_topics.clear();
				std::vector<std::string> topic_modes;
                topic_modes.clear();
	        	TiXmlElement *l_pDeviceParent = l_pDevice->FirstChildElement( "ParentDevice" );
				if ( NULL != l_pDeviceParent )
				{
					newDevice.DeviceParent = l_pDeviceParent->GetText();
				}

	            TiXmlElement *l_pDeviceName = l_pDevice->FirstChildElement( "DeviceName" );
	            if ( NULL != l_pDeviceName )
	            {
	                newDevice.DeviceName = l_pDeviceName->GetText();
	            }

	            TiXmlElement *l_pID = l_pDevice->FirstChildElement( "ID" );
	            if ( NULL != l_pID )
	            {
	            	newDevice.ID = atoi(l_pID->GetText());
	            }

	            TiXmlElement *l_pDeviceType = l_pDevice->FirstChildElement( "DeviceType" );
	            if ( NULL != l_pDeviceType )
	            {
	                newDevice.DeviceType = l_pDeviceType->GetText();
	            }

	            TiXmlElement *l_pDevicePN = l_pDevice->FirstChildElement("PartNumber");
	            if( NULL != l_pDevicePN)
	            {
	            	newDevice.PartNumber = l_pDevicePN->GetText();
	            }
	            else
	            {
	            	newDevice.PartNumber = "";
	            }

	            TiXmlElement *l_pDevicePrimaryIP = l_pDevice->FirstChildElement( "PrimaryIP" );
	            if ( NULL != l_pDevicePrimaryIP )
	            {
	            	newDevice.PrimaryIP = l_pDevicePrimaryIP->GetText();
	            }
	            else
	            {
	            	newDevice.PrimaryIP = "";
	            }

	            TiXmlElement *l_pDeviceArchitecture = l_pDevice->FirstChildElement( "Architecture" );
				if ( NULL != l_pDeviceArchitecture )
				{
					newDevice.Architecture = l_pDeviceArchitecture->GetText();
				}

				TiXmlElement *l_pBoardCount = l_pDevice->FirstChildElement( "BoardCount" );
				if ( NULL != l_pBoardCount )
				{
					newDevice.BoardCount = atoi(l_pBoardCount->GetText());
				}

				TiXmlElement *l_pSensorCount = l_pDevice->FirstChildElement( "SensorCount" );
				if ( NULL != l_pSensorCount )
				{
					newDevice.SensorCount = atoi(l_pSensorCount->GetText());
				}

				TiXmlElement *l_pShieldCount = l_pDevice->FirstChildElement( "ShieldCount" );
				if ( NULL != l_pShieldCount )
				{
					newDevice.ShieldCount = atoi(l_pShieldCount->GetText());
				}

				TiXmlElement *l_pCapability = l_pDevice->FirstChildElement("Capability");
				std::vector<std::string> capabilities;
				while( l_pCapability )
				{
					std::string capability = l_pCapability->GetText();
					capabilities.push_back(capability);
					l_pCapability = l_pCapability->NextSiblingElement( "Capability" );
				}
				newDevice.Capabilities = capabilities;
				TiXmlElement *l_pPin = l_pDevice->FirstChildElement( "Pin" );
				while( l_pPin )
				{
					icarus_rover_v2::pin newpin;
					newpin.ParentDevice = newDevice.DeviceName;

					TiXmlElement *l_pPinName = l_pPin->FirstChildElement( "Name" );
					if ( NULL != l_pPinName )
					{
						newpin.Name = l_pPinName->GetText();
					}
					
					TiXmlElement *l_pPinNumber = l_pPin->FirstChildElement( "Number" );
					if ( NULL != l_pPinNumber )
					{
						newpin.Number = atoi(l_pPinNumber->GetText());
					}

					TiXmlElement *l_pPinFunction = l_pPin->FirstChildElement( "Function" );
					if ( NULL != l_pPinFunction )
					{
						newpin.Function = l_pPinFunction->GetText();
					}

					TiXmlElement *l_pPinConnectedDevice = l_pPin->FirstChildElement( "ConnectedDevice" );
					if ( NULL != l_pPinConnectedDevice )
					{
						newpin.ConnectedDevice = l_pPinConnectedDevice->GetText();
					}
					else
					{
						newpin.ConnectedDevice = "";
					}

					TiXmlElement *l_pPinConnectedSensor = l_pPin->FirstChildElement( "ConnectedSensor" );
					if ( NULL != l_pPinConnectedSensor )
					{
						newpin.ConnectedSensor = l_pPinConnectedSensor->GetText();
					}
					else
					{
						newpin.ConnectedSensor = "";
					}

                    TiXmlElement *l_pPinDefaultValue = l_pPin->FirstChildElement( "DefaultValue" );
					if ( NULL != l_pPinDefaultValue )
					{
						newpin.DefaultValue = atoi(l_pPinDefaultValue->GetText());
					}
					else
					{
						newpin.DefaultValue = 0;
					}
					
					TiXmlElement *l_pPinMaxValue = l_pPin->FirstChildElement( "MaxValue" );
					if ( NULL != l_pPinMaxValue )
					{
						newpin.MaxValue = atoi(l_pPinMaxValue->GetText());
					}
					else
					{
						newpin.MaxValue = 0;
					}
					
					TiXmlElement *l_pPinMinValue = l_pPin->FirstChildElement( "MinValue" );
					if ( NULL != l_pPinMinValue )
					{
						newpin.MinValue = atoi(l_pPinMinValue->GetText());
					}
					else
					{
						newpin.MinValue = 0;
					}

					TiXmlElement *l_pPinAuxTopic = l_pPin->FirstChildElement( "AuxTopic" );
					if ( NULL != l_pPinAuxTopic )
					{
						newpin.AuxTopic = l_pPinAuxTopic->GetText();
					}
					else
					{
						newpin.AuxTopic = "";
					}

					TiXmlElement *l_pPinScaleFactor = l_pPin->FirstChildElement( "ScaleFactor" );
					if ( NULL != l_pPinScaleFactor )
					{
						newpin.ScaleFactor = atof(l_pPinScaleFactor->GetText());
					}
					else
					{
						newpin.ScaleFactor = 1.0;
					}


					l_pPin = l_pPin->NextSiblingElement( "Pin" );
					pins.push_back(newpin);

				}

				newDevice.pins = pins;
	            if(newDevice.DeviceName == mydevice.DeviceName)
	            {
	            	//This is me.
	            	mydevice = newDevice;
                    mydevice_assigned = true;
	            }
	            allDevices.push_back(newDevice);
	            l_pDevice = l_pDevice->NextSiblingElement( "Device" );
	        }
	    }
	}
    if(build_childDevices() == false)
    {
        diag.Diagnostic_Type = SOFTWARE;
        diag.Level = FATAL;
        diag.Diagnostic_Message = INITIALIZING_ERROR;
        diag.Description = "Couldn't build Child Devices.";
        return diag;
    }
    else if(mydevice_assigned == false)
    {
        diag.Diagnostic_Type = SOFTWARE;
        diag.Level = FATAL;
        diag.Diagnostic_Message = INITIALIZING_ERROR;
        diag.Description = "Unable to load find my Device in DeviceFile.xml";
        return diag;
    }
    else
    {
        diag.Diagnostic_Type = NOERROR;
        diag.Level = NOTICE;
        diag.Diagnostic_Message = INITIALIZING;
        diag.Description = "Processed DeviceFile.xml";
        return diag;
    }
}

void MasterNodeProcess::print_device(std::vector<icarus_rover_v2::device> devices)
{
    for(std::size_t i = 0; i < devices.size(); i++)
    {
        print_device(devices.at(i));
    }
    
}
void MasterNodeProcess::print_device(icarus_rover_v2::device device)
{
    printf("Device: %s\n",device.DeviceName.c_str());
}
void MasterNodeProcess::print_leverarm(std::vector<LeverArm> leverarms)
{
	for(std::size_t i = 0; i < leverarms.size(); i++)
	{
		print_leverarm(leverarms.at(i));
	}
}
void MasterNodeProcess::print_leverarm(LeverArm la)
{
	printf("LeverArm %s Reference: %s X: %f Y: %f Z: %f Roll: %f Pitch: %f Yaw: %f\n",
			la.name.c_str(),
			la.reference.c_str(),
			la.leverarm.x.value,la.leverarm.y.value,la.leverarm.z.value,
			la.leverarm.roll.value,la.leverarm.pitch.value,la.leverarm.yaw.value);
}
void MasterNodeProcess::print_leverarm(std::string name,std::string reference,icarus_rover_v2::leverarm leverarm)
{
	printf("LeverArm %s Reference: %s X: %f Y: %f Z: %f Roll: %f Pitch: %f Yaw: %f\n",
				name.c_str(),
				reference.c_str(),
				leverarm.x.value,leverarm.y.value,leverarm.z.value,
				leverarm.roll.value,leverarm.pitch.value,leverarm.yaw.value);
}
bool MasterNodeProcess::get_leverarm(icarus_rover_v2::leverarm *leverarm,std::string name)
{
	for(std::size_t i = 0; i < leverarms.size(); i++)
	{
		if(leverarms.at(i).name == name)
		{
			*leverarm = leverarms.at(i).leverarm;
			return true;
		}
	}
	return false;
}
bool MasterNodeProcess::build_childDevices()
{
    for(std::size_t i = 0; i < allDevices.size(); i++)
    {
        //printf("a: %s\n",allDevices.at(i).DeviceName.c_str());
        bool add_child = false;
        std::string local_parent = allDevices.at(i).DeviceParent;
        if(local_parent == myhostname) 
        { 
            add_child = true;
        }
        else
        {
            int level_counter = 0;
            bool search = true;
            bool new_local_parent_found = false;
            while(search == true)
            {
                //printf("local: %s\n",local_parent.c_str());
                
                level_counter++;
                if(level_counter > 25)
                {
                    printf("[MasterNode]: Searched more than 25 levels. Exiting.\n");
                    search = false;
                    return false;
                }
                
                
                for(std::size_t j = 0; j < allDevices.size(); j++)
                {
                    if(allDevices.at(j).DeviceParent == "None")
                    {
                        break;
                    }
                    if(local_parent == "None")
                    {
                        search = false;
                        break;
                    }
                    if(local_parent == myhostname)
                    {
                        add_child = true;
                        search = false;
                        break;
                    }
                    if(local_parent == allDevices.at(j).DeviceName)
                    {
                        local_parent = allDevices.at(j).DeviceParent;
                        new_local_parent_found = true;
                        break;
                    }
                    //if(allDevices.at(j).
                    
                }
                if(new_local_parent_found == false)
                {
                    search = false;
                    break;
                }
                
            }
        }
        if(add_child == true)
        {
            childDevices.push_back(allDevices.at(i));
        }
    }
    for(std::size_t i = 0; i < childDevices.size(); i++)
    {
        std::string baudrate = "";
        bool serialdevice = false;
        if((childDevices.at(i).PartNumber == "110012") ||
           (childDevices.at(i).PartNumber == "810090"))
        {
            serialdevice = true;
            baudrate = "115200";
        }
        if(serialdevice)
        {
            bool add = true;
            for(std::size_t i = 0; i < serialport_baudrates.size(); i++)
            {
                if(serialport_baudrates.at(i) == baudrate)
                {
                    add = false;
                    break;
                }
            }
            if(add) { serialport_baudrates.push_back(baudrate); }
        }
    }
    return true;
}
