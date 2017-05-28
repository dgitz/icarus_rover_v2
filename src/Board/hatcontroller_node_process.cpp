#include "hatcontroller_node_process.h"
HatControllerNodeProcess::HatControllerNodeProcess()
{
}
HatControllerNodeProcess::~HatControllerNodeProcess()
{

}
void HatControllerNodeProcess::init(std::string name,icarus_rover_v2::diagnostic diag)
{
    diagnostic = diag;
    armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
    ready_to_arm = false;
    initialized = false;
    ready = false;
    hats.clear();
    hats_running.clear();
    hostname = name;
    time_sincelast_pps = 0.0;
    pps_counter = 0;
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
        diag.Level = INFO;
        diag.Diagnostic_Type = COMMUNICATIONS;
        diag.Diagnostic_Message = NOERROR;
        char tempstr[512];
        sprintf(tempstr,"Updated Pin %s:%d to value: %d.",msg.ParentDevice.c_str(),msg.Number,msg.Value);
        diag.Description = std::string(tempstr);
    }
    return diag;
    
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    if(initialized == false)
    {
        if(hostname == newdevice.DeviceName)
        {
            mydevice = newdevice;
            initialized = true;
        }
    }
    else
    {
        if(ready == false)
        {
            if(newdevice.DeviceParent == hostname)
            {
                std::size_t hat_message = newdevice.DeviceType.find("Hat");
                if(hat_message != std::string::npos)
                {
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
    diag.Level = DEBUG;
    diag.Diagnostic_Type = COMMUNICATIONS;
    diag.Diagnostic_Message = NOERROR;
    char tempstr[512];
    sprintf(tempstr,"Initialized: %d Ready: %d",initialized,ready);
    diag.Description = std::string(tempstr);
    return diag;
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::update(double dt)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    time_sincelast_pps+=dt;
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
    if(hats_ready == true)
    {
        status = status and true;
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
    
    if(pps_ok == true)
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
    if(status == false) { armed_state = ARMEDSTATUS_DISARMED_CANNOTARM; }
    return diag;
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_commandmsg(icarus_rover_v2::command msg)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
    if(msg.Command == ROVERCOMMAND_ARM)
    {
        armed_state = ARMEDSTATUS_ARMED;
    }
    else if(msg.Command == ROVERCOMMAND_DISARM)
    {
        armed_state = ARMEDSTATUS_DISARMED;
    }
    
    diag.Level = INFO;
    diag.Diagnostic_Type = SOFTWARE;
    diag.Diagnostic_Message = NOERROR;
    char tempstr[512];
    sprintf(tempstr,"Rover Command: %d Processed.",msg.Command);
    diag.Description = std::string(tempstr);
    return diag;
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::set_servohat_initialized(uint16_t id)
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

                }
                else if(armed_state == ARMEDSTATUS_ARMED)
                {
                    if(hats.at(i).pins.at(j).Function == "PWMOutput")
                    {
                    	pin = hats.at(i).pins.at(j);
                    }
                    else
                    {
                        pin = hats.at(i).pins.at(j);
                        pin.Value = hats.at(i).pins.at(j).DefaultValue;
                    }
                }
                else
                {
                    pin = hats.at(i).pins.at(j);
                    pin.Value = hats.at(i).pins.at(j).DefaultValue;
                }
                pins.push_back(pin);
            }
        }
    }
    return pins;
}
