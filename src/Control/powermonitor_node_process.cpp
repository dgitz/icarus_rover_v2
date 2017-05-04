#include "powermonitor_node_process.h"

PowerMonitorNodeProcess::PowerMonitorNodeProcess()
{
	all_device_info_received = false;
	run_time = 0.0;
    my_batteries.clear();
    power_state = POWERSTATE_NORMAL;
    active_battery.name = "";
    active_battery.active = false;
    active_battery.recharging = false;
    active_battery.id = -1;
}
PowerMonitorNodeProcess::~PowerMonitorNodeProcess()
{

}
std::vector<icarus_rover_v2::battery> PowerMonitorNodeProcess::get_batteries()
{
    return my_batteries;
}
icarus_rover_v2::diagnostic PowerMonitorNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
    bool file_loaded = false;
    TiXmlDocument system_doc("/home/robot/config/DeviceFile.xml");
    bool devicefile_loaded = system_doc.LoadFile();
    std::string errorstring;
    if(devicefile_loaded == true)
    {
        file_loaded = parse_devicefile(diagnostic,system_doc);
        if(file_loaded == false)
        {
            diagnostic.Diagnostic_Type = SOFTWARE;
            diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
            diagnostic.Level = ERROR;
            char tempstr[512];
            sprintf(tempstr,"Device File not Parsed Correctly.");
            diagnostic.Description = std::string(tempstr);   
            return diagnostic;
        }
        else
        {
            diagnostic.Diagnostic_Type = SOFTWARE;
            diagnostic.Diagnostic_Message = NOERROR;
            diagnostic.Level = INFO;
            char tempstr[512];
            sprintf(tempstr,"Device File Parsed.");
            diagnostic.Description = std::string(tempstr);   
            return diagnostic;
        }
    }
    else
    {
        diagnostic.Diagnostic_Type = SOFTWARE;
        diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
        diagnostic.Level = ERROR;
        char tempstr[512];
        sprintf(tempstr,"Device File not Loaded Correctly.");
        diagnostic.Description = std::string(tempstr);   
        return diagnostic;
    }
}
icarus_rover_v2::diagnostic PowerMonitorNodeProcess::update(double dt)
{

    run_time += dt;
    //State Machine Logic
    bool case_any_battery_above_recharge = false;
    bool case_anyother_battery_above_charged = false;
    bool case_allother_below_switchlevel = true;
    bool case_all_below_switchlevel = true;
    for(std::size_t i = 0; i < my_batteries.size(); i++)
    {
        double voltage = 0.0;
        for(std::size_t j = 0; j < my_batteries.at(i).cells.size(); j++)
        {
            voltage += my_batteries.at(i).cells.at(j).current_voltage;
        }
        my_batteries.at(i).current_voltage = voltage;
    }
    for(int i = 0; i < my_batteries.size(); i++)
    {
    	if(my_batteries.at(i).capacity_level_perc > BATTERYLEVEL_RECHARGE)
    	{
    		case_any_battery_above_recharge = true;
    	}
    	if(my_batteries.at(i).name != active_battery.name)
    	{
    		if(my_batteries.at(i).capacity_level_perc > BATTERYLEVEL_CHARGED)
    		{
    			case_anyother_battery_above_charged = true;
    		}
    	}
    	if(my_batteries.at(i).name != active_battery.name)
    	{
    		if(my_batteries.at(i).capacity_level_perc > BATTERYLEVEL_TO_SWITCH)
    		{
    			case_allother_below_switchlevel = false;
    		}
    	}
    	if(my_batteries.at(i).capacity_level_perc > BATTERYLEVEL_TO_SWITCH)
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
    	icarus_rover_v2::battery newbattery = get_bestbattery();
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

    diagnostic.Diagnostic_Type = SOFTWARE;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Node Executing.";
	return diagnostic;
}
icarus_rover_v2::battery PowerMonitorNodeProcess::get_bestbattery()
{
    if(my_batteries.size() > 0)
    {
        return my_batteries.at(0);
    }
    else
    {
        icarus_rover_v2::battery emptybattery;
        emptybattery.name = "";
        emptybattery.id = -1;
        return emptybattery;
    }
}
icarus_rover_v2::diagnostic PowerMonitorNodeProcess::new_pinmsg(icarus_rover_v2::pin pinmsg)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    bool found = false;
    for(std::size_t i = 0; i < my_batteries.size(); i++)
    {
        for(std::size_t j = 0; j < my_batteries.at(i).cells.size(); j++)
        {
            if(pinmsg.ConnectedDevice == my_batteries.at(i).cells.at(j).name)
            {
                found = true;
                my_batteries.at(i).cells.at(j).current_voltage = pinmsg.Value;
            }
        }
    }
    if(found == true)
    {
        diag.Diagnostic_Type = SOFTWARE;
        diag.Level = INFO;
        diag.Diagnostic_Message = NOERROR;
        char tempstr[512];
        sprintf(tempstr,"%s Updated",pinmsg.ConnectedDevice.c_str());
        diag.Description = std::string(tempstr);
    }
    else
    {
        diag.Diagnostic_Type = SOFTWARE;
        diag.Level = INFO;
        diag.Diagnostic_Message = NOERROR;
        char tempstr[512];
        sprintf(tempstr,"No Cell Updated.");
        diag.Description = std::string(tempstr);
    }
	return diag;
}
icarus_rover_v2::diagnostic PowerMonitorNodeProcess::new_commandmsg(icarus_rover_v2::command msg)
{
	return diagnostic;
}
float PowerMonitorNodeProcess::get_voltage(std::string name)
{
    for(std::size_t i = 0; i < my_batteries.size(); i++)
    {
        if(my_batteries.at(i).name == name)
        {
            return my_batteries.at(i).current_voltage;
        }
        for(std::size_t j = 0; j < my_batteries.at(i).cells.size(); j++)
        {
            if(my_batteries.at(i).cells.at(j).name == name)
            {
                return my_batteries.at(i).cells.at(j).current_voltage;
            }
        }
    }
    return -1;
    
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
std::string PowerMonitorNodeProcess::print_batteryinfo()
{
    char tempstr[2048];
    sprintf(tempstr,"");
	for(std::size_t i = 0; i < my_batteries.size(); i++)
	{
		sprintf(tempstr,"%s%s Voltage: %4.2f/%4.2f Capacity: %4.2f/%4.2f (Ah)\n",tempstr,my_batteries.at(i).name.c_str(),
            my_batteries.at(i).current_voltage,my_batteries.at(i).rated_voltage,
            my_batteries.at(i).current_capacity_Ah,my_batteries.at(i).rated_capacity_Ah);
        for(std::size_t j = 0; j < my_batteries.at(i).cells.size(); j++)
        {
            sprintf(tempstr,"%s|---%s Voltage: %4.2f/%4.2f Capacity: %4.2f/%4.2f (Ah)\n",
                tempstr,
                my_batteries.at(i).cells.at(j).name.c_str(),
                my_batteries.at(i).cells.at(j).current_voltage,
                my_batteries.at(i).cells.at(j).rated_voltage,
                my_batteries.at(i).cells.at(j).current_capacity_Ah,
                my_batteries.at(i).cells.at(j).rated_capacity_Ah);
        }
	}
    return std::string(tempstr);
	
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
bool PowerMonitorNodeProcess::parse_devicefile(icarus_rover_v2::diagnostic indiag,TiXmlDocument doc)
{
    TiXmlElement *l_pRootElement = doc.RootElement();
    bool found_battery = false;
	if( NULL != l_pRootElement )
	{
	    TiXmlElement *l_pDeviceList = l_pRootElement->FirstChildElement( "DeviceList" );

	    if ( NULL != l_pDeviceList )
	    {
	        TiXmlElement *l_pDevice = l_pDeviceList->FirstChildElement( "Device" );

	        while( l_pDevice )
	        {
	        	std::string devicetype;
                TiXmlElement *l_pDeviceType = l_pDevice->FirstChildElement( "DeviceType" );
	            if ( NULL != l_pDeviceType )
	            {
	                devicetype = l_pDeviceType->GetText();
	            }
                
                if(devicetype == "Battery")
                {
                    found_battery = true;
                    icarus_rover_v2::battery newbattery;
                    newbattery.cells.clear();

                    TiXmlElement *l_pDeviceName = l_pDevice->FirstChildElement("DeviceName");
                    if ( NULL != l_pDeviceName )
                    {
                        newbattery.name = l_pDeviceName->GetText();
                    }
                    else { return false; }
                    
                    TiXmlElement *l_pDeviceId = l_pDevice->FirstChildElement("ID");
                    if ( NULL != l_pDeviceId )
                    {
                        newbattery.id = atoi(l_pDeviceId->GetText());
                    }
                    else { return false; }
                    
                    TiXmlElement *l_pChemistry = l_pDevice->FirstChildElement("Capacity");
                    if ( NULL != l_pChemistry )
                    {
                        newbattery.chemistry = l_pChemistry->GetText();
                    }
                    else { return false; }
                    
                    TiXmlElement *l_pDeviceSeriesCount = l_pDevice->FirstChildElement("Series");
                    if ( NULL != l_pDeviceSeriesCount )
                    {
                        newbattery.cells_in_series = atoi(l_pDeviceSeriesCount->GetText());
                    }
                    else { return false; }
                    
                    TiXmlElement *l_pDeviceParallelCount = l_pDevice->FirstChildElement("Parallel");
                    if ( NULL != l_pDeviceParallelCount )
                    {
                        newbattery.cells_in_parallel = atoi(l_pDeviceParallelCount->GetText());
                    }
                    else { return false; }
                    
                    TiXmlElement *l_pCapacity = l_pDevice->FirstChildElement("Capacity");
                    if ( NULL != l_pCapacity )
                    {
                        newbattery.rated_capacity_Ah = atof(l_pCapacity->GetText());
                    }
                    else { return false; }
                    
                    TiXmlElement *l_pVoltage = l_pDevice->FirstChildElement("Voltage");
                    if ( NULL != l_pVoltage )
                    {
                        newbattery.rated_voltage = atof(l_pVoltage->GetText());
                    }
                    else { return false; }
                    
                    newbattery.current_voltage = newbattery.rated_voltage;
                    newbattery.current_capacity_Ah = newbattery.rated_capacity_Ah;
                    newbattery.capacity_level_perc = 100.0;
                    newbattery.active = false;
                    newbattery.recharging = false;
                    
                    TiXmlElement *l_pCell = l_pDevice->FirstChildElement( "Cell" );
                    while( l_pCell )
                    {
                        icarus_rover_v2::cell newcell;
                        TiXmlElement *l_pCellName = l_pCell->FirstChildElement("Name");
                        if ( NULL != l_pCellName )
                        {
                            std::string name = l_pCellName->GetText();
                            newcell.name = newbattery.name + "/" + name;
                        }
                        else { return false; }
                        TiXmlElement *l_pCellVoltage = l_pCell->FirstChildElement("Voltage");
                        if ( NULL != l_pCellVoltage )
                        {
                            newcell.rated_voltage = atof(l_pCellVoltage->GetText());
                        }
                        else { return false; }
                        
                        TiXmlElement *l_pCellCapacity = l_pCell->FirstChildElement("Capacity");
                        if ( NULL != l_pCellCapacity )
                        {
                            newcell.rated_capacity_Ah = atof(l_pCellCapacity->GetText());
                        }
                        else { return false; }
                        
                        newcell.current_voltage = newcell.rated_voltage;
                        newcell.current_capacity_Ah = newcell.rated_capacity_Ah;
                        
                        newbattery.cells.push_back(newcell);
                        l_pCell = l_pCell->NextSiblingElement( "Cell" );
                    }
                    if(newbattery.cells.size() == 0) { return false; }
                    my_batteries.push_back(newbattery);
                    l_pDevice = l_pDevice->NextSiblingElement( "Device" );
                    
                }
                else
                {
                    l_pDevice = l_pDevice->NextSiblingElement( "Device" );
                }
            }
        }
    }
    return found_battery;
}
/*
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

	            

}
*/