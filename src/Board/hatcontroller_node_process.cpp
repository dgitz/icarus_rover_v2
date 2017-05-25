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
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
	printf("Got device msg: %s:%d\n",newdevice.DeviceName.c_str(),(int)newdevice.ID);
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
    }
    return diag;
    /*
	if(hat_message != std::string::npos)
	{
		if(newdevice.ID == get_boardid())
		{
			my_boardname = newdevice.DeviceName;
			shield_count = newdevice.ShieldCount;
		}
	}

	else if ((shield_message!=std::string::npos) && (my_boardname != ""))
	{
		if(my_boardname == newdevice.DeviceParent)
		{
			bool add_me = true;
			for(int i = 0; i < myshields.size();i++)
			{
				if(myshields.at(i).ID == newdevice.ID)
				{

					add_me = false;
				}
			}
			if(add_me == true)
			{
				myshields.push_back(newdevice);
				char tempstr[255];
				sprintf(tempstr,"Adding Shield: %s:%d to Board: %s:%d",
						newdevice.DeviceName.c_str(),
						(int)newdevice.ID,
						my_boardname.c_str(),
						(int)get_boardid());
				printf("%s\n",tempstr);
				bool status =  configure_port(newdevice.ID,newdevice.pins);
				if(status == false)
				{
					diagnostic.Diagnostic_Type = SENSORS;
					diagnostic.Level = FATAL;
					diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
					char tempstr[1024];
					sprintf(tempstr,"Unable to configure Port/Pin Combination for: %s:%d",newdevice.DeviceName.c_str(),(int)newdevice.ID);
					diagnostic.Description = tempstr;
					return diagnostic;
				}


			}
		}

	}
	else
	{
		char tempstr[255];
		sprintf(tempstr,"Device: %s not currently supported.",newdevice.DeviceName.c_str());
	}
	return diagnostic;
    */
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::update(double dt)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    if(ready_to_arm == false) //Checks to see if I can arm
    {
        //See if hats are ready yet
        bool hats_ready = true;
        for(std::size_t i = 0; i < hats_running.size(); i++)
        {
            if(hats_running.at(i) == true) { hats_ready = hats_ready and true; }
            else { hats_ready = false; }
        }
        if(hats_running.size() == 0) { hats_ready = false; }
        
        if(hats_ready == true)
        {
            ready_to_arm = true;
        }
    }
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
icarus_rover_v2::diagnostic HatControllerNodeProcess::set_hat_initialized(uint16_t id)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    bool found = false;
    for(std::size_t i = 0; i < hats.size(); i++)
    {
        if(hats.at(i).ID == id)
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
std::vector<ChannelValue> HatControllerNodeProcess::get_servohatvalues(uint16_t id)
{
    std::vector<ChannelValue> cv_vector;
    for(std::size_t i = 0; i < hats.size(); i++)
    {
        if((hats.at(i).DeviceType == "ServoHat") and (hats.at(i).ID == id))
        {
            for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
            {
                ChannelValue cv;
                if(hats.at(i).pins.at(j).Function == "PWMOutput-NonActuator")
                {
                    cv.channel = hats.at(i).pins.at(j).Number;
                    cv.value = hats.at(i).pins.at(j).Value;
                }
                else if(armed_state == ARMEDSTATUS_ARMED)
                {
                    if(hats.at(i).pins.at(j).Function == "PWMOutput")
                    {
                        cv.channel = hats.at(i).pins.at(j).Number;
                        cv.value = hats.at(i).pins.at(j).Value;
                    }
                    else
                    {
                        cv.channel = hats.at(i).pins.at(j).Number;
                        cv.value = hats.at(i).pins.at(j).DefaultValue;
                    }
                }
                else
                {
                    cv.channel = hats.at(i).pins.at(j).Number;
                    cv.value = hats.at(i).pins.at(j).DefaultValue;
                }
                cv_vector.push_back(cv);
            }
        }
    }
    return cv_vector;
}
