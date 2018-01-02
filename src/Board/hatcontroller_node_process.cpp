#include "hatcontroller_node_process.h"
/*! \brief Constructor
 */
HatControllerNodeProcess::HatControllerNodeProcess()
{
	run_time = 0.0;
	initialized = false;
    ready = false;

    ready_to_arm = false;
   	armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
    hats.clear();
    hats_running.clear();
    time_sincelast_pps = 0.0;
    pps_counter = 0;
    analyze_timing = false;
    timing_diff.clear();
}
/*! \brief Deconstructor
 */
HatControllerNodeProcess::~HatControllerNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic HatControllerNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
   	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	return diagnostic;
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::update(double dt)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    time_sincelast_pps+=dt;
    run_time += dt;
	if((mydevice.BoardCount == 0) and (mydevice.SensorCount == 0))
    {
        if(initialized == true) { ready = true; }
    }
    bool hats_ready = true;
    bool pps_ok = false;
    
    //See if hats are ready yet
    for(std::size_t i = 0; i < hats_running.size(); i++)
    {
        if(hats_running.at(i) == true) { hats_ready = hats_ready and true; }
        else { hats_ready = false; }
    }
    if(hats_running.size() == 0) { hats_ready = false; }
    
    if((pps_counter > 0) and (time_sincelast_pps < 5.0)) { pps_ok = true; }
    else { pps_ok = false; }
        
    bool status = true;    
    if((hats_ready == true) ||
    		((mydevice.BoardCount == 0) and (mydevice.SensorCount == 0)))
    {
        status = status and true;
		ready = true;
    }
    else
    {
        status = false;
        diag.Level = NOTICE;
        diag.Diagnostic_Type = SOFTWARE;
        diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
        char tempstr[512];
        sprintf(tempstr,"All info for Hats not received yet.");
        diag.Description = std::string(tempstr);     
    }
    
    if((pps_ok == true) || (run_time < 1.0))
    {
        status = status and true;
    }
    else
    {
        status = false;
        diag.Level = WARN;
        diag.Diagnostic_Type = COMMUNICATIONS;
        diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
        char tempstr[512];
        sprintf(tempstr,"PPS Counter: %ld Time since last: %f",pps_counter,time_sincelast_pps);
        diag.Description = std::string(tempstr);   
    }
    
    ready_to_arm = status;
    if(status == false) 
    { 
        armed_state = ARMEDSTATUS_DISARMED_CANNOTARM; 
    }
    else
    {
        diag.Level = INFO;
        diag.Diagnostic_Type = SOFTWARE;
        diag.Diagnostic_Message = NOERROR;
        diag.Description = "Node Running";
    }
    diagnostic = diag;
    return diag;
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
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
                if(board_present(newdevice) == true)
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
                    if(newdevice.DeviceType == "ServoHat")
                    {
                        for(std::size_t i = 0; i < newdevice.pins.size(); i++)
                        {
                            if((newdevice.pins.at(i).Function == "PWMOutput") or 
                               (newdevice.pins.at(i).Function == "PWMOutput-NonActuator")) {}
                            else
                            {
                                diag.Level = ERROR;
                                diag.Diagnostic_Type = SOFTWARE;
                                diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
                                char tempstr[512];
                                sprintf(tempstr,"Hat Type: %s Pin Function: %s Not supported.",
                                    newdevice.DeviceType.c_str(),newdevice.pins.at(i).Function.c_str());
                                diag.Description = std::string(tempstr);
                                return diag;
                            }
                        }
                    }
                    else if(newdevice.DeviceType == "TerminalHat")
                    {
                        for(std::size_t i = 0; i < newdevice.pins.size(); i++)
                        {
                            if((newdevice.pins.at(i).Function == "DigitalInput") or 
                               (newdevice.pins.at(i).Function == "DigitalOutput-NonActuator") or 
                               (newdevice.pins.at(i).Function == "DigitalOutput")) {}
                            else
                            {
                                diag.Level = ERROR;
                                diag.Diagnostic_Type = SOFTWARE;
                                diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
                                char tempstr[512];
                                sprintf(tempstr,"Hat Type: %s Pin Function: %s Not supported.",
                                    newdevice.DeviceType.c_str(),newdevice.pins.at(i).Function.c_str());
                                diag.Description = std::string(tempstr);
                                return diag;
                            }
                        }
                    }
                    else
                    {
                        diag.Level = ERROR;
                        diag.Diagnostic_Type = SOFTWARE;
                        diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
                        char tempstr[512];
                        sprintf(tempstr,"Hat Type: %s Not supported.",newdevice.DeviceType.c_str());
                        diag.Description = std::string(tempstr);
                        return diag;
                    }
                    hats.push_back(newdevice);
                    hats_running.push_back(false);
                    if(hats.size() == mydevice.BoardCount) { ready = true; }
                }
            }
        }
        else
        {

        }
    }
    return diag;
}
/*! \brief Process Command Message
 */
std::vector<icarus_rover_v2::diagnostic> HatControllerNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
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
std::vector<icarus_rover_v2::diagnostic> HatControllerNodeProcess::check_program_variables()
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
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_ppsmsg(std_msgs::Bool msg)
{
    time_sincelast_pps = 0.0;
    pps_counter++;
    icarus_rover_v2::diagnostic diag = diagnostic;
    diag.Level = INFO;
    diag.Diagnostic_Type = COMMUNICATIONS;
    diag.Diagnostic_Message = NOERROR;
    char tempstr[512];
    sprintf(tempstr,"Received PPS.");
    diag.Description = std::string(tempstr);
    return diag;
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_pinmsg(icarus_rover_v2::pin msg)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    int found = -1;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if(hats.at(i).DeviceName == msg.ParentDevice)
		{
			found = 0;
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				if(hats.at(i).pins.at(j).Number == msg.Number)
				{
					found = 1;
					hats.at(i).pins.at(j) = msg;
					found = true;
					break;
				}
			}
		}
	}


    if(found == -1)
    {
        diag.Level = INFO;
        diag.Diagnostic_Type = COMMUNICATIONS;
        diag.Diagnostic_Message = NOERROR;
        char tempstr[512];
        sprintf(tempstr,"Pin Msg for Device: %s but not for me.",msg.ParentDevice.c_str());
        diag.Description = std::string(tempstr);
    }
    else if(found == 0)
    {
        diag.Level = WARN;
        diag.Diagnostic_Type = COMMUNICATIONS;
        diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
        char tempstr[512];
        sprintf(tempstr,"Pin Msg for My Hat but Pin not found: %s:%d",msg.ParentDevice.c_str(),msg.Number);
        diag.Description = std::string(tempstr);
    }
    else
    {
    	if(analyze_timing == true)
    	{
    		struct timeval now;
    		gettimeofday(&now,NULL);
    		timing_diff.push_back(measure_time_diff(msg.stamp,now));
    		if(timing_diff.size() > TIMING_BUFFER_LENGTH)
    		{
    			timing_diff.erase(timing_diff.begin());
    		}
    	}

    	diag.Level = INFO;
    	diag.Diagnostic_Type = COMMUNICATIONS;
    	diag.Diagnostic_Message = NOERROR;
            char tempstr[512];
            sprintf(tempstr,"Updated Pin %s:%d to value: %d.",msg.ParentDevice.c_str(),msg.Number,msg.Value);
            diag.Description = std::string(tempstr);

    }
    return diag;

}
double HatControllerNodeProcess::get_timedelay()
{
	std::vector<double> temp = timing_diff;
	double t = 0.0;
	if(temp.size() <= (TIMING_BUFFER_LENGTH/2))
	{
		return 0.0;
	}
	for(std::size_t i = 0; i < temp.size(); i++)
	{
		t += temp.at(i);
	}
	return t/(double)(temp.size());
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_pinsmsg(icarus_rover_v2::iopins msg)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    int found = -1;
    for(std::size_t k = 0; k < msg.pins.size(); k++)
    {
        for(std::size_t i = 0; i < hats.size(); i++)
        {
            if(hats.at(i).DeviceName == msg.pins.at(k).ParentDevice)
            {
                found = 0;
                for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
                {
                    if(hats.at(i).pins.at(j).Number == msg.pins.at(k).Number)
                    {
                        found = 1;
                        hats.at(i).pins.at(j) = msg.pins.at(k);
                        found = true;
                        break;
                    }
                }
            }
        }
    }
    /*
    if(found == -1)
    {
        diag.Level = INFO;
        diag.Diagnostic_Type = COMMUNICATIONS;
        diag.Diagnostic_Message = NOERROR;
        char tempstr[512];
        sprintf(tempstr,"Pin Msg for Device: %s but not for me.",msg.ParentDevice.c_str());
        diag.Description = std::string(tempstr);
    }
    else if(found == 0)
    {
        diag.Level = WARN;
        diag.Diagnostic_Type = COMMUNICATIONS;
        diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
        char tempstr[512];
        sprintf(tempstr,"Pin Msg for My Hat but Pin not found: %s:%d",msg.ParentDevice.c_str(),msg.Number);
        diag.Description = std::string(tempstr);
    }
    else
    {
        diag.Level = INFO;
        diag.Diagnostic_Type = COMMUNICATIONS;
        diag.Diagnostic_Message = NOERROR;
        char tempstr[512];
        sprintf(tempstr,"Updated Pin %s:%d to value: %d.",msg.ParentDevice.c_str(),msg.Number,msg.Value);
        diag.Description = std::string(tempstr);
    }
    */
    diag.Level = INFO;
    diag.Diagnostic_Type = COMMUNICATIONS;
    diag.Diagnostic_Message = NOERROR;
    char tempstr[512];
    sprintf(tempstr,"Updated Pins %d",(int)msg.pins.size());
    diag.Description = std::string(tempstr);
    return diag;
    
}


icarus_rover_v2::diagnostic HatControllerNodeProcess::new_armedstatemsg(uint8_t msg)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	armed_state = msg;
	diag.Level = INFO;
	diag.Diagnostic_Type = SOFTWARE;
	diag.Diagnostic_Message = NOERROR;
	char tempstr[512];
	sprintf(tempstr,"Rover Armed State: %d Processed.",msg);
	diag.Description = std::string(tempstr);
	return diag;
}
bool HatControllerNodeProcess::is_servohat_running(uint16_t id)
{
    bool found = false;
    for(std::size_t i = 0; i < hats.size(); i++)
    {
        if((hats.at(i).DeviceType == "ServoHat") and (hats.at(i).ID == id))
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
icarus_rover_v2::diagnostic HatControllerNodeProcess::set_servohat_running(uint16_t id)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    bool found = false;
    for(std::size_t i = 0; i < hats.size(); i++)
    {
        if((hats.at(i).DeviceType == "ServoHat") and (hats.at(i).ID == id))
        {
            found = true;
            hats_running.at(i) = true;
        }
    }
    if(found == false)
    {
        diag.Level = WARN;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[512];
		sprintf(tempstr,"Hat: %d Not Found",id);
		diag.Description = std::string(tempstr);
    }
    else
    {
        diag.Level = INFO;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"Hat: %d is now Initialized",id);
		diag.Description = std::string(tempstr);
    }
    return diag;
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::set_terminalhat_initialized()
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
        diag.Level = INFO;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"TerminalHat is now Initialized");
		diag.Description = std::string(tempstr);
    }
    return diag;
}
std::vector<uint16_t> HatControllerNodeProcess::get_servohataddresses()
{
    std::vector<uint16_t> addresses;
    addresses.clear();
    for(std::size_t i = 0; i < hats.size(); i++)
    {
        if(hats.at(i).DeviceType == "ServoHat")
        {
            addresses.push_back(hats.at(i).ID);
        }
    }
    return addresses;
}
std::vector<icarus_rover_v2::pin> HatControllerNodeProcess::get_terminalhatpins(std::string Function)
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
                if(hats.at(i).pins.at(j).Function == Function)
                {
                    pin = hats.at(i).pins.at(j);
                    if((Function == "DigitalOutput") and (armed_state != ARMEDSTATUS_ARMED))
                    {
                        pin.Value = hats.at(i).pins.at(j).DefaultValue;
                    }
                    else
                    {
                        pin.Value = hats.at(i).pins.at(j).Value;
                    }
                    pins.push_back(pin);
                }                
            }
        }
    }
    return pins;
}
std::vector<icarus_rover_v2::pin> HatControllerNodeProcess::get_servohatpins(uint16_t id)
{
    std::vector<icarus_rover_v2::pin> pins;
    for(std::size_t i = 0; i < hats.size(); i++)
    {
        if((hats.at(i).DeviceType == "ServoHat") and (hats.at(i).ID == id))
        {
            for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
            {
            	icarus_rover_v2::pin pin;
                if(hats.at(i).pins.at(j).Function == "PWMOutput-NonActuator")
                {
                    pin = hats.at(i).pins.at(j);
                    pins.push_back(pin);

                }
                else if(armed_state == ARMEDSTATUS_ARMED)
                {
                    if(hats.at(i).pins.at(j).Function == "PWMOutput")
                    {
                    	pin = hats.at(i).pins.at(j);
                        pins.push_back(pin);
                    }
                    else
                    {
                        pin = hats.at(i).pins.at(j);
                        pin.Value = hats.at(i).pins.at(j).DefaultValue;
                        pins.push_back(pin);
                    }
                }
                else
                {
                    pin = hats.at(i).pins.at(j);
                    pin.Value = hats.at(i).pins.at(j).DefaultValue;
                    pins.push_back(pin);
                }
            }
        }
    }
    return pins;
}
bool HatControllerNodeProcess::board_present(icarus_rover_v2::device device)
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
double HatControllerNodeProcess::measure_time_diff(struct timeval start, struct timeval end)
{
    double t1 = (double)(start.tv_sec)+((double)(start.tv_usec)/1000000.0);
    double t2 = (double)(end.tv_sec)+((double)(end.tv_usec)/1000000.0);
    return t2-t1;
}
double HatControllerNodeProcess::measure_time_diff(ros::Time start, struct timeval end)
{
    double t1 = (double)start.sec + (double)(start.nsec)/1000000000.0;
    double t2 = (double)(end.tv_sec)+((double)(end.tv_usec)/1000000.0);
    return t2-t1;
}
double HatControllerNodeProcess::measure_time_diff(double start, struct timeval end)
{
    double t2 = (double)(end.tv_sec)+((double)(end.tv_usec)/1000000.0);
    return t2-start;
}
double HatControllerNodeProcess::measure_time_diff(struct timeval start, double end)
{
    double t1 = (double)(start.tv_sec)+((double)(start.tv_usec)/1000000.0);
    return end-t1;
}
double HatControllerNodeProcess::measure_time_diff(double start, double end)
{
    return end-start;
}
ros::Time HatControllerNodeProcess::convert_time(struct timeval t_)
{
    ros::Time t;
    t.sec = t_.tv_sec;
    t.nsec = t_.tv_usec*1000;
    return t;
}
