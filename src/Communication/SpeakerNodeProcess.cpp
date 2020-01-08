#include "SpeakerNodeProcess.h"

SpeakerNodeProcess::SpeakerNodeProcess()
{
	ready_to_speek = false;
	is_speaking = false;
	is_paused = false;
	speech_output = "";
	time_left_to_finish = 0.0;
	pause_timer = 0.0;
}
SpeakerNodeProcess::~SpeakerNodeProcess()
{

}
eros::diagnostic  SpeakerNodeProcess::finish_initialization()
{
    eros::diagnostic diag = root_diagnostic;

	diag = update_diagnostic(COMMUNICATIONS,WARN,DROPPING_PACKETS,"SoundPlay Node not running yet.");
	diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error");
    return diag;
}
eros::diagnostic SpeakerNodeProcess::update(double t_dt,__attribute__((unused))double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if(initialized == true)
	{
		ready = true;
	}
	if(is_paused == true)
	{
		pause_timer += t_dt;
		if(pause_timer > PAUSE_BETWEEN)
		{
			is_paused = false;
		}
	}
	if(ready_to_speek == true)
	{
		is_speaking = true;
		ready_to_speek = false;
	}
	if(is_speaking == true)
	{
		time_left_to_finish -= t_dt;
		if(time_left_to_finish <- 0.0)
		{
			time_left_to_finish = 0.0;
			is_speaking = false;
			is_paused = true;
		}
	}
	else if((ready_to_speek == false) && (is_speaking == false) && (is_paused == false))
	{
		speech_output= "";
		if(level4_messages.size() > 0)
		{
			eros::usermessage usermsg = level4_messages.front();
			level4_messages.erase(level4_messages.begin());
			ready_to_speek = true;
			speech_output = usermsg.message;
			time_left_to_finish = compute_timetospeak(speech_output);
		}
		else if(level3_messages.size() > 0)
		{
			eros::usermessage usermsg = level3_messages.front();
			level3_messages.erase(level3_messages.begin());
			ready_to_speek = true;
			speech_output = usermsg.message;
			time_left_to_finish = compute_timetospeak(speech_output);
		}
		else if(level2_messages.size() > 0)
		{
			eros::usermessage usermsg = level2_messages.front();
			level2_messages.erase(level2_messages.begin());
			ready_to_speek = true;
			speech_output = usermsg.message;
			time_left_to_finish = compute_timetospeak(speech_output);
		}
		else if(level1_messages.size() > 0)
		{
			eros::usermessage usermsg = level1_messages.front();
			level1_messages.erase(level1_messages.begin());
			ready_to_speek = true;
			speech_output = usermsg.message;
			time_left_to_finish = compute_timetospeak(speech_output);
			if(level1_messages.size() > 3)
			{
				level1_messages.erase(level1_messages.begin(),level1_messages.end());
			}
		}
	}

	ms_timer += (t_dt)*1000.0;
	if(ms_timer >= timeout_value_ms) { timer_timeout = true; }
	if(timer_timeout == true)
	{
		timer_timeout = false;

	}
	diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Updated.");
	return diag;
}
eros::diagnostic SpeakerNodeProcess::new_soundplaystatus()
{
	eros::diagnostic diag = root_diagnostic;
	diag = update_diagnostic(COMMUNICATIONS,INFO,NOERROR,"Received SoundPlay Status.");
	return diag;
}
eros::diagnostic SpeakerNodeProcess::new_devicemsg(__attribute__((unused)) const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Updated Device");
	return diag;
}
std::vector<eros::diagnostic> SpeakerNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
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
	for(std::size_t i = 0; i < diaglist.size(); ++i)
	{
		diag = update_diagnostic(diaglist.at(i));
	}
	return diaglist;
}
double SpeakerNodeProcess::compute_timetospeak(std::string s)
{
	return (double)(s.size()/(double)SPEECH_RATE);
}

eros::diagnostic SpeakerNodeProcess::new_usermessage(const eros::usermessage::ConstPtr& t_msg)
{
	eros::diagnostic diag = root_diagnostic;
	if(t_msg->Level == LEVEL1)
	{
		level1_messages.push_back(convert_fromptr(t_msg));
		if(level1_messages.size() > LEVEL_MESSAGE_MAXCOUNT)
		{
			level1_messages.erase(level1_messages.begin());
		}
	}
	else if(t_msg->Level == LEVEL2)
	{
		level2_messages.push_back(convert_fromptr(t_msg));
		if(level2_messages.size() > LEVEL_MESSAGE_MAXCOUNT)
		{
			level2_messages.erase(level2_messages.begin());
		}
	}
	else if(t_msg->Level == LEVEL3)
	{
		level3_messages.push_back(convert_fromptr(t_msg));
		if(level3_messages.size() > LEVEL_MESSAGE_MAXCOUNT)
		{
			level3_messages.erase(level3_messages.begin());
		}
	}
	else if(t_msg->Level == LEVEL4)
	{
		level4_messages.push_back(convert_fromptr(t_msg));
		if(level4_messages.size() > LEVEL_MESSAGE_MAXCOUNT)
		{
			level4_messages.erase(level4_messages.begin());
		}
	}
	else
	{
		diag = update_diagnostic(COMMUNICATIONS,WARN,DROPPING_PACKETS,"Received Level: " + std::to_string(t_msg->Level) + " But not Currently Supported.");
		return diag;
	}
	diag = update_diagnostic(COMMUNICATIONS,INFO,NOERROR,"Processed UserMessage");
	return diag;
}
std::vector<eros::usermessage> SpeakerNodeProcess::get_usermessages(uint8_t level)
{
	if(level == LEVEL1)
	{
		return level1_messages;
	}
	else if(level == LEVEL2)
	{
		return level2_messages;
	}
	else if(level == LEVEL3)
	{
		return level3_messages;
	}
	else if(level == LEVEL4)
	{
		return level4_messages;
	}
	else
	{
		std::vector<eros::usermessage> emptylist;
		return emptylist;
	}
}
std::vector<eros::diagnostic> SpeakerNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true) {
		diag = update_diagnostic(SOFTWARE,INFO,DIAGNOSTIC_PASSED,"Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	} else {
		diag = update_diagnostic(SOFTWARE,WARN,DIAGNOSTIC_FAILED,"Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
/*
void SpeakerNodeProcess::initialize_stateack_messages()
{
	send_speechmessage.name = "Send Speech Messagge";
	send_speechmessage.trigger = false;
	send_speechmessage.state = false;
	gettimeofday(&send_speechmessage.orig_send_time,NULL);
	send_speechmessage.retries = 0;
	send_speechmessage.timeout_counter = 0;
	send_speechmessage.retry_mode = false;
	send_speechmessage.failed = false;
	send_speechmessage.flag1 = 0;
	send_speechmessage.stream_rate = -1.0;  //Don't stream this
}
*//*
state_ack SpeakerNodeProcess::get_stateack(std::string name)
{

	if(name == send_speechmessage.name)
	{
		return send_speechmessage;
	}
	else
	{
		state_ack emptystateack;
		emptystateack.name = "";
		return emptystateack;
	}
}
bool SpeakerNodeProcess::set_stateack(state_ack stateack)
{
	if(stateack.name == "Send Speech Message")
	{
		send_speechmessage = stateack;
	}
	else
	{
		return false;
	}
	return true;
}
bool SpeakerNodeProcess::checkTriggers(std::vector<std::vector<unsigned char > > &tx_buffers)
{
	bool nothing_triggered = true;
	//send_nodemode.trigger = true;
	if(send_speechmessage.trigger == true)
	{
		bool send_me = false;
		if(send_nodemode.stream_rate < 0) //Normal operation, should send
		{
			send_me = true;
			send_nodemode.trigger = false;
		}
		else
		{
			struct timeval now;
			gettimeofday(&now,NULL);
			double etime = time_diff(send_nodemode.orig_send_time,now);
			double delay = 1.0/(send_nodemode.stream_rate);
			if(etime > delay){ send_me = true; }
			else{ send_me = false; }
		}
		if(send_me == true)
		{
			nothing_triggered = false;
			//printf("send_nodemode Triggered.\n");
			char buffer[16];
			int length;
			int computed_checksum;
			int tx_status = serialmessagehandler->encode_ModeSerial(buffer,&length,0,0,node_state);
			bool status = gather_message_info(SERIAL_Mode_ID, "transmit");
			tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));
			send_nodemode.state = true;
			if (send_nodemode.retrying == false)
			{
				gettimeofday(&send_nodemode.orig_send_time,NULL);
				send_nodemode.retries = 0;
			}
		}

	}
	if(nothing_triggered == true)
	{
		tx_buffers.clear();
		return false;
	}
	else
	{
		return true;
	}
}
*/