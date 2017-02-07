#include "powermonitor_node_process.h"

PowerMonitorNodeProcess::PowerMonitorNodeProcess()
{
	all_device_info_received = false;
	run_time = 0.0;
    battery_count = 0;
    connection_method = "";
    batteries.clear();
}
PowerMonitorNodeProcess::~PowerMonitorNodeProcess()
{

}
icarus_rover_v2::diagnostic PowerMonitorNodeProcess::init(icarus_rover_v2::diagnostic indiag,
		Logger *log,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mylogger = log;
	mydevice.DeviceName = hostname;
    bool file_loaded = false;
    TiXmlDocument system_doc("/home/robot/config/SystemFile.xml");
    bool devicefile_loaded = system_doc.LoadFile();
    std::string errorstring;
    if(devicefile_loaded == true)
    {
        diagnostic = parse_systemfile(diagnostic,system_doc);
        if(diagnostic.Level <= NOTICE)
        {
            file_loaded = true;
        }
        if(connection_method == "Sequential")
        {
            batteries.at(0).active = true;
        }
        else if(connection_method == "Parallel")
        {
            for(int i = 0; i < batteries.size(); i++)
            {
                batteries.at(i).active = true;
            }
        }
        else
        {
            diagnostic.Diagnostic_Type = POWER;
            diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
            diagnostic.Level = ERROR;
            char tempstr[512];
            sprintf(tempstr,"Connection Method: %s Not Recognized",connection_method.c_str());
            diagnostic.Description = std::string(connection_method);   
            return diagnostic;
        }
    }
    if(file_loaded == false)
    {
        mylogger->log_error(diagnostic.Description);
        diagnostic.Diagnostic_Type = POWER;
        diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
        diagnostic.Level = ERROR;
        diagnostic.Description = "Unable to load SystemFile.xml";        
    }
	return diagnostic;
}
icarus_rover_v2::diagnostic PowerMonitorNodeProcess::update(double dt)
{
    run_time += dt;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Node Executing.";
	return diagnostic;
}
icarus_rover_v2::diagnostic PowerMonitorNodeProcess::new_pinmsg(icarus_rover_v2::pin pinmsg)
{
	return diagnostic;
}
icarus_rover_v2::diagnostic PowerMonitorNodeProcess::new_commandmsg(icarus_rover_v2::command msg)
{
	return diagnostic;
}

icarus_rover_v2::diagnostic PowerMonitorNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	if((newdevice.DeviceName == myhostname) && (all_device_info_received == false))
	{
		mydevice = newdevice;
		all_device_info_received = true;
	}
	return diagnostic;
}
double PowerMonitorNodeProcess::time_diff(struct timeval timea, struct timeval timeb)
{
	long mtime, seconds, useconds;
	seconds  = timeb.tv_sec  - timea.tv_sec;
	useconds = timeb.tv_usec - timea.tv_usec;

	mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	return (double)(mtime)/1000.0;
}
icarus_rover_v2::diagnostic PowerMonitorNodeProcess::parse_systemfile(icarus_rover_v2::diagnostic indiag,TiXmlDocument doc)
{
    icarus_rover_v2::diagnostic diag = indiag;
	TiXmlElement *l_pRootElement = doc.RootElement();

	if( NULL != l_pRootElement )
	{
	    TiXmlElement *l_pPower = l_pRootElement->FirstChildElement( "Power" );
	    if ( NULL != l_pPower )
	    {
	        TiXmlElement *l_pBatteryCount = l_pPower->FirstChildElement( "BatteryCount" );
            if ( NULL != l_pBatteryCount )
            {
                battery_count = atoi(l_pBatteryCount->GetText());
            }
            else
            {
                diag.Level = ERROR;
                diag.Description = "Could not load Tag: BatteryCount";
            }
            
            TiXmlElement *l_pConnectionMethod = l_pPower->FirstChildElement( "ConnectionMethod" );
            if ( NULL != l_pConnectionMethod )
            {
                connection_method = l_pConnectionMethod->GetText();
            }
            else
            {
                diag.Level = ERROR;
                diag.Description = "Could not load Tag: ConnectionMethod";
            }
	    }
        TiXmlElement *l_pDevice = l_pRootElement->FirstChildElement( "Device" );
        while( l_pDevice )
	    {
            
            TiXmlElement *l_pDeviceType = l_pDevice->FirstChildElement( "DeviceType" );
            std::string devicetype = "";
            if ( NULL != l_pDeviceType )
            {
                devicetype = l_pDeviceType->GetText();
            }
            else
            {
                diag.Level = ERROR;
                diag.Description = "Could not load Tag: DeviceType";
            }
            if(devicetype == "Battery")
            {
                Battery newbattery;
                TiXmlElement *l_pBatteryName = l_pDevice->FirstChildElement( "DeviceName" );
                if ( NULL != l_pBatteryName )
                {
                    newbattery.name = l_pBatteryName->GetText();
                }
                else
                {
                    diag.Level = ERROR;
                    diag.Description = "Could not load Tag: DeviceName";
                }
                
                TiXmlElement *l_pBatteryId = l_pDevice->FirstChildElement( "ID" );
                if ( NULL != l_pBatteryId )
                {
                    newbattery.id = atoi(l_pBatteryId->GetText());
                }
                else
                {
                    diag.Level = ERROR;
                    diag.Description = "Could not load Tag: ID";
                }
                
                TiXmlElement *l_pChemistry = l_pDevice->FirstChildElement( "Chemistry" );
                if ( NULL != l_pChemistry )
                {
                    newbattery.chemistry = l_pChemistry->GetText();
                }
                else
                {
                    diag.Level = ERROR;
                    diag.Description = "Could not load Tag: Chemistry";
                }
                
                TiXmlElement *l_pCapacity = l_pDevice->FirstChildElement( "Capacity" );
                if ( NULL != l_pCapacity )
                {
                    newbattery.rated_capacity_Ah = atof(l_pCapacity->GetText());
                    newbattery.actual_capacity_Ah = newbattery.rated_capacity_Ah;
                    newbattery.capacity_level_perc = 100.0;
                }
                else
                {
                    diag.Level = ERROR;
                    diag.Description = "Could not load Tag: Capacity";
                }
                
                TiXmlElement *l_pVoltage = l_pDevice->FirstChildElement( "Voltage" );
                if ( NULL != l_pVoltage )
                {
                    newbattery.voltage = atof(l_pVoltage->GetText());
                }
                else
                {
                    diag.Level = ERROR;
                    diag.Description = "Could not load Tag: Voltage";
                }
                newbattery.active = false;
                batteries.push_back(newbattery);
            }
            l_pDevice = l_pDevice->NextSiblingElement( "Device" );
        }
	}
	return diag;
}