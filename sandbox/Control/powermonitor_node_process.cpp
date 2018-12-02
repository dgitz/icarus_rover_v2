#include "powermonitor_node_process.h"

PowerMonitorNodeProcess::PowerMonitorNodeProcess()
{
	all_device_info_received = false;
	run_time = 0.0;
    battery_count = 0;
    connection_method = "";
    batteries.clear();
    power_state = POWERSTATE_NORMAL;
    active_battery.name = "";
    active_battery.active = false;
    active_battery.recharging = false;
    active_battery.id = -1;
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
            active_battery = batteries.at(0);
        }
        /*
        else if(connection_method == "Parallel")
        {
            for(int i = 0; i < batteries.size(); i++)
            {
                batteries.at(i).active = true;

            }
        }
        */
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
    //State Machine Logic
    bool case_any_battery_above_recharge = false;
    bool case_anyother_battery_above_charged = false;
    bool case_allother_below_switchlevel = true;
    bool case_all_below_switchlevel = true;
    for(int i = 0; i < batteries.size(); i++)
    {
    	if(batteries.at(i).capacity_level_perc > BATTERYLEVEL_RECHARGE)
    	{
    		case_any_battery_above_recharge = true;
    	}
    	if(batteries.at(i).name != active_battery.name)
    	{
    		if(batteries.at(i).capacity_level_perc > BATTERYLEVEL_CHARGED)
    		{
    			case_anyother_battery_above_charged = true;
    		}
    	}
    	if(batteries.at(i).name != active_battery.name)
    	{
    		if(batteries.at(i).capacity_level_perc > BATTERYLEVEL_TO_SWITCH)
    		{
    			case_allother_below_switchlevel = false;
    		}
    	}
    	if(batteries.at(i).capacity_level_perc > BATTERYLEVEL_TO_SWITCH)
    	{
    		case_all_below_switchlevel = false;
    	}
    }

    if(power_state == POWERSTATE_CHANGINGACTIVEBATTERY)
    {
    	power_state = POWERSTATE_NORMAL;
    }

    if(active_battery.capacity_level_perc > BATTERYLEVEL_CHARGED)
    {
    	power_state = POWERSTATE_NORMAL;
    }
    else if(case_all_below_switchlevel == true)
    {
    	power_state = POWERSTATE_EMERGENCY;
    }
    else if((active_battery.capacity_level_perc <= BATTERYLEVEL_TO_SWITCH) && (case_anyother_battery_above_charged == true))
    {
    	power_state = POWERSTATE_CHANGINGACTIVEBATTERY;
    	Battery newbattery = get_bestbattery();
    	if(newbattery.name != "")
    	{
    		newbattery.active = true;
    		active_battery = newbattery;
    	}
    }
    else if((active_battery.capacity_level_perc <= BATTERYLEVEL_RECHARGE) && (case_allother_below_switchlevel == true))
    {
    	power_state = POWERSTATE_REQUIRERECHARGE;
    }

    diagnostic.Diagnostic_Type = POWER;
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
Battery PowerMonitorNodeProcess::get_bestbattery()
{
	int best_index = -1;
	double highest_capacity = 0.0;
	for(int i = 0; i < batteries.size(); i++)
	{
		if(batteries.at(i).capacity_level_perc > highest_capacity)
		{
			best_index = i;
			highest_capacity = batteries.at(i).capacity_level_perc;
		}
	}
	if(best_index < 0)
	{
		Battery battery;
		battery.name = "";
		battery.active = false;
		battery.recharging = false;
		battery.id = -1;
		return battery;
	}
	else
	{
		return batteries.at(best_index);
	}
}
double PowerMonitorNodeProcess::time_diff(struct timeval timea, struct timeval timeb)
{
	long mtime, seconds, useconds;
	seconds  = timeb.tv_sec  - timea.tv_sec;
	useconds = timeb.tv_usec - timea.tv_usec;

	mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	return (double)(mtime)/1000.0;
}
icarus_rover_v2::diagnostic PowerMonitorNodeProcess::new_batterymsg(Battery battery)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	bool found = false;
	for(int i = 0; i < batteries.size(); i++)
	{
		if(batteries.at(i).name == battery.name)
		{
			batteries.at(i) = battery;
			found = true;
		}
	}
	if(battery.name == active_battery.name)
	{
		active_battery = battery;
	}
	if(found == false)
	{
		diag.Diagnostic_Type = POWER;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		diag.Level = WARN;
		char tempstr[512];
		sprintf(tempstr,"Received message about Battery: %s but not recognized.",battery.name.c_str());
		mylogger->log_warn(tempstr);
		diag.Description = std::string(tempstr);
	}
	else
	{
		diag.Diagnostic_Type = POWER;
		diag.Diagnostic_Message = NOERROR;
		diag.Level = INFO;
		char tempstr[512];
		sprintf(tempstr,"Received message about Battery: %s.",battery.name.c_str());
		mylogger->log_info(tempstr);
		diag.Description = std::string(tempstr);
	}
	return diag;

}
void PowerMonitorNodeProcess::print_batteryinfo()
{
	for(int i = 0; i < batteries.size(); i++)
	{
		printf("%s\t",batteries.at(i).name.c_str());
	}
	printf("\n");
	for(int i = 0; i < batteries.size(); i++)
	{
		printf("%f\t",batteries.at(i).capacity_level_perc);
	}
	printf("\n");
}
std::string PowerMonitorNodeProcess::map_PowerState_ToString(int v)
{
	switch(v)
	{
	case POWERSTATE_UNDEFINED: 				return "Undefined";					break;
	case POWERSTATE_NORMAL:					return "Normal";					break;
	case POWERSTATE_STANDBY:				return "Standby";					break;
	case POWERSTATE_CHANGINGACTIVEBATTERY:	return "Changing Active Battery";	break;
	case POWERSTATE_REQUIRERECHARGE:		return "Require Recharge";			break;
	case POWERSTATE_EMERGENCY:				return "Emergency";					break;

	default:
		std::string tempstr;
		tempstr = "Power State: " + boost::lexical_cast<std::string>(v) + " Not Supported";
		return tempstr;
	}
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
                newbattery.recharging = false;
                batteries.push_back(newbattery);
            }
            l_pDevice = l_pDevice->NextSiblingElement( "Device" );
        }
	}
	return diag;
}
