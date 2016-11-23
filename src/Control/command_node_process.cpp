#include "command_node_process.h"

CommandNodeProcess::CommandNodeProcess()
{
	ms_timer = 0;
	timeout_value_ms = 0;
    init_time = ros::Time::now();
    armeddisarmed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
    ReadyToArmList.clear();
    readytoarm = false;
}
CommandNodeProcess::~CommandNodeProcess()
{

}
icarus_rover_v2::diagnostic CommandNodeProcess::init(icarus_rover_v2::diagnostic indiag,
		Logger *log,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mylogger = log;
	mydevice.DeviceName = hostname;
	return diagnostic;
}
icarus_rover_v2::diagnostic CommandNodeProcess::init_readytoarm_list(std::vector<std::string> topics)
{
    for(int i = 0; i < topics.size();i++)
    {
        ReadyToArm newarm;
        newarm.topic = topics.at(i);
        newarm.ready_to_arm = false;
        ReadyToArmList.push_back(newarm);
    }
    diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Initialied Ready To Arm List.";
    return diagnostic;
}
icarus_rover_v2::diagnostic CommandNodeProcess::new_readytoarmmsg(std::string topic, bool value)
{
    for(int i = 0; i < ReadyToArmList.size();i++)
    {
        if(ReadyToArmList.at(i).topic == topic)
        {
            ReadyToArmList.at(i).ready_to_arm = value;
        }
    }
    diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Processed ready_to_arm topic.";
    return diagnostic;
}
icarus_rover_v2::diagnostic new_armcommandmsg(uint8_t value)
{
    if(armeddisarmed_state == ARMEDSTATUS_UNDEFINED)
    {
        diagnostic.Diagnostic_Type = REMOTE_CONTROL;
        diagnostic.Level = FATAL;
        diagnostic.Description = "Armed Status is UNDEFINED!";
        diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
        return diagnostic;
    }
    else if(armeddisarmed_state == ARMEDSTATUS_DISARMED_CANNOTARM)
    {
        diagnostic.Diagnostic_Type = REMOTE_CONTROL;
        diagnostic.Level = WARN;
        diagnostic.Description = "Cannot set new Arm Command";
        diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
        return diagnostic;
    }
    else if(armeddisarmed_state == ARMEDSTATUS_DISARMED)
    {
        if(value == ARMEDCOMMAND_ARM)
        {
            
        }
        else if(value == ARMEDCOMMAND_DISARM)
        {
            
        }
    }
    else if(armeddisarmed_state == ARMEDSTATUS_ARMED)
    {
        if(value == ARMEDCOMMAND_ARM)
        {
            
        }
        else if(value == ARMEDCOMMAND_DISARM)
        {
            
        }
    }
}
icarus_rover_v2::diagnostic CommandNodeProcess::update(long dt)
{
    bool temp = true;
    for(int i = 0; i < ReadyToArmList.size();i++)
    {
        if(ReadyToArmList.at(i).ready_to_arm == false)
        {
            temp = false;
            diagnostic.Level = WARN;
            diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
            char tempstr[255];
            sprintf(tempstr,"Topic: %s Reports is Unable to Arm.",ReadyToArmList.at(i).topic.c_str());
            mylogger->log_warn(tempstr);
            diagnostic.Description = std::string(tempstr);
        }
    }
    readytoarm = temp;
    if(readytoarm == false)
    {
        armeddisarmed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
    }
	ms_timer += dt;
	if(ms_timer >= timeout_value_ms) { timer_timeout = true; }
	if(timer_timeout == true)
	{
		timer_timeout = false;
		//printf("Mode: %d,%d\n",node_state,board_state);
		/*if((node_state == GPIO_MODE_INITIALIZING) && (board_state == GPIO_MODE_INITIALIZING))
		{
			//printf("Setting to true.\n");
			send_configure_DIO_PortA.trigger = true;
			send_configure_DIO_PortB.trigger = true;
			prev_node_state = node_state;
			node_state = GPIO_MODE_INITIALIZED;
		}
		*/

	}


	//send_nodemode.trigger = true;
	
	return diagnostic;
}


icarus_rover_v2::diagnostic CommandNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	if((newdevice.DeviceName == myhostname) && (all_device_info_received == false))
	{
		mydevice = newdevice;

		all_device_info_received = true;
	}
	return diagnostic;
}
double CommandNodeProcess::time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
