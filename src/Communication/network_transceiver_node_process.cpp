#include "network_transceiver_node_process.h"
/*! \brief Constructor
 */
NetworkTransceiverNodeProcess::NetworkTransceiverNodeProcess()
{
    run_time = 0.0;
	initialized = false;
    init_messages();
}
/*! \brief Deconstructor
 */
NetworkTransceiverNodeProcess::~NetworkTransceiverNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic NetworkTransceiverNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic NetworkTransceiverNodeProcess::update(double dt)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	run_time += dt;
    for(std::size_t i = 0; i < messages.size(); i++)
    {
        if(messages.at(i).sent_counter > 0)
        {
            messages.at(i).sent_rate = (double)(messages.at(i).sent_counter)/run_time;
        }
        
        if(messages.at(i).recv_counter > 0)
        {
            messages.at(i).recv_rate = (double)(messages.at(i).recv_counter)/run_time;
        }
    }
	
	diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "Node Running";
	diagnostic = diag;
	return diag;
}
/*! \brief Setup Process Device info
 */
icarus_rover_v2::diagnostic NetworkTransceiverNodeProcess::new_devicemsg(icarus_rover_v2::device device)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
	bool new_device = true;
	if(device.DeviceName == myhostname)
	{

    }
    return diag;
}
/*! \brief Process Command Message
 */
std::vector<icarus_rover_v2::diagnostic> NetworkTransceiverNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
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
std::vector<icarus_rover_v2::diagnostic> NetworkTransceiverNodeProcess::check_program_variables()
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
	bool any_message_sent = false;
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		if(messages.at(i).sent_counter > 0)
		{
			any_message_sent = true;
		}
	}
	if(any_message_sent == false)
	{
		status = false;
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = WARN;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		diag.Description = "Have not sent any UDP Packets Yet.";
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

icarus_rover_v2::diagnostic NetworkTransceiverNodeProcess::new_message_sent(uint16_t id)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    for(std::size_t i = 0; i < messages.size(); i++)
    {
        if(messages.at(i).id == id)
        {
            messages.at(i).sent_counter++;
        }
    }
    return diag;
}
icarus_rover_v2::diagnostic NetworkTransceiverNodeProcess::new_message_recv(uint16_t id)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    for(std::size_t i = 0; i < messages.size(); i++)
    {
        if(messages.at(i).id == id)
        {
            messages.at(i).recv_counter++;
        }
    }
    return diag;
}
void NetworkTransceiverNodeProcess::init_messages()
{
    {
        Message newmessage;
        newmessage.id = ESTOP_ID;
        newmessage.name = "EStop";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = POWER_ID;
        newmessage.name = "Power";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = CONFIGURE_ANA_PORT_ID;
        newmessage.name = "Configure ANA Port ID";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = PPS_ID;
        newmessage.name = "PPS";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = FINDTARGET_ID;
        newmessage.name = "Find Target";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = SET_DIO_PORT_DEFAULTVALUE_ID;
        newmessage.name = "Set DIO Port Default Value";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = HEARTBEAT_ID;
        newmessage.name = "Heartbeat";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = ARM_STATUS_ID;
        newmessage.name = "Arm Status";
        messages.push_back(newmessage);
    }
    /*
    {
        Message newmessage;
        newmessage.id = TUNE_CONTROLGROUP_ID;
        newmessage.name = "Tune Control Group";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = SETUP_CONTROLGROUP_ID;
        newmessage.name = "Setup Control Group";
        messages.push_back(newmessage);
    }
    */
    {
        Message newmessage;
        newmessage.id = ARMCONTROL_ID;
        newmessage.name = "Arm Control";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = FIRMWAREVERSION_ID;
        newmessage.name = "Firmware Version";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = GET_ANA_PORT1_ID;
        newmessage.name = "Get ANA Port1";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = GET_DIO_PORT1_ID;
        newmessage.name = "Get DIO Port1";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = SET_DIO_PORT_ID;
        newmessage.name = "Set DIO Port";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = MODE_ID;
        newmessage.name = "Mode";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = CONFIGURE_DIO_PORT_ID;
        newmessage.name = "Configure DIO Port";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = TESTMESSAGECOMMAND_ID;
        newmessage.name = "Test Message Command";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = TESTMESSAGECOUNTER_ID;
        newmessage.name = "Test Message Counter";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = DEVICE_ID;
        newmessage.name = "Device";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = DIAGNOSTIC_ID;
        newmessage.name = "Diagnostic";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = RESOURCE_ID;
        newmessage.name = "Resource";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = REMOTECONTROL_ID;
        newmessage.name = "Remote Control";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = COMMAND_ID;
        newmessage.name = "Command";
        messages.push_back(newmessage);
    }
    {
        Message newmessage;
        newmessage.id = USERMESSAGE_ID;
        newmessage.name = "User Message";
        messages.push_back(newmessage);
    }
    {
    	Message newmessage;
    	newmessage.id = TUNECONTROLGROUP_ID;
    	newmessage.name = "Tune ControlGroup";
    	messages.push_back(newmessage);
    }
    
    for(std::size_t i = 0; i < messages.size(); i++)
    {
        messages.at(i).sent_counter = 0;
        messages.at(i).recv_counter = 0;
        messages.at(i).sent_rate = 0.0;
        messages.at(i).recv_rate = 0.0;
    }
    
}
std::string NetworkTransceiverNodeProcess::get_messageinfo(bool v)
{
    char tempstr[2048];
    sprintf(tempstr,"\n");
    
    for(std::size_t i = 0; i < messages.size(); i++)
    {
        if((v == true) || (messages.at(i).sent_counter > 0) || (messages.at(i).recv_counter > 0))
        {
            sprintf(tempstr,"%sMessage: %s(%0X) Sent: %d @%0.2f Hz Recv: %d @%0.2f Hz\n",tempstr,
                messages.at(i).name.c_str(),
                messages.at(i).id,
                messages.at(i).sent_counter,
                messages.at(i).sent_rate,
                messages.at(i).recv_counter,
                messages.at(i).recv_rate);
        }            
    }
    return std::string(tempstr);
}

