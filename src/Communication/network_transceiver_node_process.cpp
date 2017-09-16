#include "network_transceiver_node_process.h"

NetworkTransceiverNodeProcess::NetworkTransceiverNodeProcess()
{
    run_time = 0.0;
    init_messages();
}
NetworkTransceiverNodeProcess::~NetworkTransceiverNodeProcess()
{

}
icarus_rover_v2::diagnostic NetworkTransceiverNodeProcess::init(icarus_rover_v2::diagnostic diag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = diag;
	return diagnostic;
}
icarus_rover_v2::diagnostic NetworkTransceiverNodeProcess::update(double dt)
{
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
	return diagnostic;
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
