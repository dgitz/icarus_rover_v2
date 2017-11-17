#include "master_node_process.h"

MasterNodeProcess::MasterNodeProcess()
{
}
MasterNodeProcess::~MasterNodeProcess()
{

}
icarus_rover_v2::diagnostic MasterNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;

	return diagnostic;
}
icarus_rover_v2::diagnostic MasterNodeProcess::update(double dt)
{
   
	return diagnostic;
}

icarus_rover_v2::diagnostic MasterNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{

	return diagnostic;
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
        diag.Description = "Unable to load DeviceFile.xml";
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
	            else
	            {
	            	allDevices.push_back(newDevice);
	            }
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
    return true;
}