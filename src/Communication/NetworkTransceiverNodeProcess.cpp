#include "NetworkTransceiverNodeProcess.h"
eros::diagnostic  NetworkTransceiverNodeProcess::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	init_messages();
	return diagnostic;
}
void NetworkTransceiverNodeProcess::set_networkconfiguration(std::string t_multicast_group,int t_send_multicast_port,int t_recv_unicast_port)
	{
		multicast_group = t_multicast_group;
		send_multicast_port = t_send_multicast_port;
		recv_unicast_port = t_recv_unicast_port;
	}
eros::diagnostic NetworkTransceiverNodeProcess::update(double t_dt,double t_ros_time)
{
	if(initialized == true)
	{
		ready = true;

	}
	eros::diagnostic diag = diagnostic;
	diag = update_baseprocess(t_dt,t_ros_time);
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
	if(diag.Level <= NOTICE)
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Node Running.";
		diagnostic = diag;
	}

	return diag;
}
eros::diagnostic NetworkTransceiverNodeProcess::check_remoteHeartbeats()
{
	eros::diagnostic diag = diagnostic;
	bool heartbeat_pass = true;
	double now = ros_time;
	for(int i = 0; i < remote_devices.size();++i)
	{
		double time_since_last = now-remote_devices.at(i).current_beatepoch_sec;
		if(time_since_last > 1.0)
		{
			heartbeat_pass = false;
			char tempstr[255];
			sprintf(tempstr,"Haven't received Heartbeat from: %s in %f Seconds. Disarming.",
					remote_devices.at(i).Name.c_str(),time_since_last);
			diag.Diagnostic_Type = COMMUNICATIONS;
			diag.Level = ERROR;
			diag.Diagnostic_Message = MISSING_HEARTBEATS;
			diag.Description = std::string(tempstr);

		}
	}
	if(remote_devices.size() == 0)
	{
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = WARN;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		diag.Description = "No Remote UI Devices Found Yet.";
		heartbeat_pass = false;

	}
	if(heartbeat_pass == true)
	{
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Remote Heartbeat Check Passed.";
	}
	remote_heartbeat_pass = heartbeat_pass;
	return diag;
}
eros::diagnostic NetworkTransceiverNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = diagnostic;
	return diag;
}
std::vector<eros::diagnostic> NetworkTransceiverNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = diagnostic;
	if (t_msg->Command == ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		if (t_msg->Option1 == LEVEL1)
		{
			diaglist.push_back(diag);
		}
		else if (t_msg->Option1 == LEVEL2)
		{
			diaglist = check_programvariables();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL3)
		{
			diaglist = run_unittest();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL4)
		{
		}
	}
	return diaglist;
}
std::vector<eros::diagnostic> NetworkTransceiverNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = diagnostic;
	bool status = true;
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
	eros::diagnostic diag_heartbeat = check_remoteHeartbeats();
	if(diag_heartbeat.Level > NOTICE)
	{
		status = false;
		diaglist.push_back(diag_heartbeat);
	}
	else
	{
		diaglist.push_back(diag_heartbeat);
	}
	if (status == true) {
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED.";
		diaglist.push_back(diag);
	} else {
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED.";
		diaglist.push_back(diag);
	}
	return diaglist;
}
eros::diagnostic NetworkTransceiverNodeProcess::new_remoteheartbeatmsg(double timestamp,std::string name,double current_beat,double expected_beat)
{
	eros::diagnostic diag = diagnostic;
	std::vector<eros::diagnostic> diag_list;
	bool add_new_entry = true;
	for(int i = 0; i < remote_devices.size(); i++)
	{
		if(name == remote_devices.at(i).Name)
		{
			add_new_entry = false;
			remote_devices.at(i).current_beatepoch_sec = timestamp;
			remote_devices.at(i).expected_beatepoch_sec = expected_beat;
			break;
		}
	}
	if(add_new_entry == true)
	{
		RemoteDevice dev;
		dev.Name = name;
		dev.current_beatepoch_sec = timestamp;
		dev.expected_beatepoch_sec = expected_beat;
		dev.offset_sec = 0.0;//timestamp-dev.current_beatepoch_sec;
		remote_devices.push_back(dev);
	}
	if(add_new_entry == true)
	{
		diag.Level = NOTICE;
		char tempstr[512];
		sprintf(tempstr,"Remote Heartbeat Device: %s Added.",name.c_str());
		diag.Description = std::string(tempstr);
		diag.Diagnostic_Message = INITIALIZING;
	}
	else
	{
		diag.Level = INFO;
		char tempstr[512];
		sprintf(tempstr,"Remote Heartbeat Device: %s Updated.",name.c_str());
		diag.Description = std::string(tempstr);
		diag.Diagnostic_Message = NOERROR;
	}

	return diag;


}
eros::diagnostic NetworkTransceiverNodeProcess::new_message_sent(uint16_t id)
{
	eros::diagnostic diag = diagnostic;
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		if(messages.at(i).id == id)
		{
			messages.at(i).sent_counter++;
		}
	}
	return diag;
}
eros::diagnostic NetworkTransceiverNodeProcess::new_message_recv(uint16_t id)
{
	eros::diagnostic diag = diagnostic;
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		if(messages.at(i).id == id)
		{
			messages.at(i).recv_counter++;
		}
	}
	return diag;
}
std::vector<NetworkTransceiverNodeProcess::QueueElement> NetworkTransceiverNodeProcess::get_sendqueue(uint8_t level)
{
	std::vector<NetworkTransceiverNodeProcess::QueueElement> buffer;
	if (level == PriorityLevel::HIGH)
	{
		buffer = sendqueue_highpriority;
		sendqueue_highpriority.clear();
	}
	else if (level == PriorityLevel::MEDIUM)
	{
		buffer= sendqueue_mediumpriority;
		sendqueue_mediumpriority.clear();
	}
	else if(level == PriorityLevel::LOW)
	{
		buffer= sendqueue_lowpriority;
		sendqueue_lowpriority.clear();
	}
	else
	{

	}
	return buffer;
}
bool NetworkTransceiverNodeProcess::push_sendqueue(uint16_t id,std::string msg)
{
	bool found = false;
	bool state = false;
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		if(messages.at(i).id == id)
		{
			found = true;
			if (messages.at(i).priority_level == PriorityLevel::HIGH)
			{

				state = push_sendhighqueue(id,msg);
			}
			else if (messages.at(i).priority_level == PriorityLevel::MEDIUM)
			{
				state = push_sendmediumqueue(id,msg);
			}
			else if (messages.at(i).priority_level == PriorityLevel::LOW)
			{
				state = push_sendlowqueue(id,msg);
			}

		}
	}
	return found and state;
}
void NetworkTransceiverNodeProcess::init_messages()
{

	{
		Message newmessage;
		newmessage.id = POWER_ID;
		newmessage.name = "Power";
		newmessage.priority_level = PriorityLevel::MEDIUM;
		newmessage.target_sendrate = 1.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = CONFIGURE_ANA_PORT_ID;
		newmessage.name = "Configure ANA Port ID";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 1.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = PPS_ID;
		newmessage.name = "PPS";
		newmessage.priority_level = PriorityLevel::HIGH;
		newmessage.target_sendrate = 1.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = FINDTARGET_ID;
		newmessage.name = "Find Target";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 0.5;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = SET_DIO_PORT_DEFAULTVALUE_ID;
		newmessage.name = "Set DIO Port Default Value";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 1.0;
		messages.push_back(newmessage);
	}

	{
		Message newmessage;
		newmessage.id = HEARTBEAT_ID;
		newmessage.name = "Heartbeat";
		newmessage.priority_level = PriorityLevel::HIGH;
		newmessage.target_sendrate = 5.0;
		messages.push_back(newmessage);
	}

	{
		Message newmessage;
		newmessage.id = ARM_STATUS_ID;
		newmessage.name = "Arm Status";
		newmessage.priority_level = PriorityLevel::HIGH;
		newmessage.target_sendrate = 10.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = ARMCONTROL_ID;
		newmessage.name = "Arm Control";
		newmessage.priority_level = PriorityLevel::MEDIUM;
		newmessage.target_sendrate = 2.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = FIRMWAREVERSION_ID;
		newmessage.name = "Firmware Version";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 1.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = GET_ANA_PORT1_ID;
		newmessage.name = "Get ANA Port1";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 1.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = GET_DIO_PORT1_ID;
		newmessage.name = "Get DIO Port1";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 1.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = SET_DIO_PORT_ID;
		newmessage.name = "Set DIO Port";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 1.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = MODE_ID;
		newmessage.name = "Mode";
		newmessage.priority_level = PriorityLevel::MEDIUM;
		newmessage.target_sendrate = 2.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = CONFIGURE_DIO_PORT_ID;
		newmessage.name = "Configure DIO Port";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 1.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = TESTMESSAGECOMMAND_ID;
		newmessage.name = "Test Message Command";
		newmessage.priority_level = PriorityLevel::MEDIUM;
		newmessage.target_sendrate = 2.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = TESTMESSAGECOUNTER_ID;
		newmessage.name = "Test Message Counter";
		newmessage.priority_level = PriorityLevel::MEDIUM;
		newmessage.target_sendrate = 2.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = DEVICE_ID;
		newmessage.name = "Device";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 2.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = DIAGNOSTIC_ID;
		newmessage.name = "Diagnostic";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 2.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = RESOURCE_ID;
		newmessage.name = "Resource";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 1.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = REMOTECONTROL_ID;
		newmessage.name = "Remote Control";
		newmessage.priority_level = PriorityLevel::HIGH;
		newmessage.target_sendrate = 10.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = COMMAND_ID;
		newmessage.name = "Command";
		newmessage.priority_level = PriorityLevel::HIGH;
		newmessage.target_sendrate = 10.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = USERMESSAGE_ID;
		newmessage.name = "User Message";
		newmessage.priority_level = PriorityLevel::LOW;
		newmessage.target_sendrate = 1.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = TUNECONTROLGROUP_ID;
		newmessage.name = "Tune ControlGroup";
		newmessage.priority_level = PriorityLevel::MEDIUM;
		newmessage.target_sendrate = 10.0;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = SUBSYSTEMDIAGNOSTIC_ID;
		newmessage.name = "Subsystem Diagnostic";
		newmessage.priority_level = PriorityLevel::MEDIUM;
		newmessage.target_sendrate = 5.0;
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
