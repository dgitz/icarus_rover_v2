#include "speaker_node_process.h"

SpeakerNodeProcess::SpeakerNodeProcess()
{
	all_device_info_received = false;
	ready_to_speek = false;
	is_speaking = false;
	is_paused = false;
	speech_output = "";
	time_left_to_finish = 0.0;
	pause_timer = 0.0;
	run_time = 0.0;
}
SpeakerNodeProcess::~SpeakerNodeProcess()
{

}
icarus_rover_v2::diagnostic SpeakerNodeProcess::init(icarus_rover_v2::diagnostic indiag,
		std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	return diagnostic;
}
icarus_rover_v2::diagnostic SpeakerNodeProcess::update(double dt)
{
	if(is_paused == true)
	{
		pause_timer += dt;
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
		time_left_to_finish -= dt;
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
			icarus_rover_v2::usermessage usermsg = level4_messages.front();
			level4_messages.erase(level4_messages.begin());
			ready_to_speek = true;
			speech_output = usermsg.message;
			time_left_to_finish = compute_timetospeak(speech_output);
		}
		else if(level3_messages.size() > 0)
		{
			icarus_rover_v2::usermessage usermsg = level3_messages.front();
			level3_messages.erase(level3_messages.begin());
			ready_to_speek = true;
			speech_output = usermsg.message;
			time_left_to_finish = compute_timetospeak(speech_output);
		}
		else if(level2_messages.size() > 0)
		{
			printf("t: %f doing msg2\n",run_time);
			icarus_rover_v2::usermessage usermsg = level2_messages.front();
			level2_messages.erase(level2_messages.begin());
			ready_to_speek = true;
			speech_output = usermsg.message;
			time_left_to_finish = compute_timetospeak(speech_output);
		}
		else if(level1_messages.size() > 0)
		{
			icarus_rover_v2::usermessage usermsg = level1_messages.front();
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

	run_time += dt;
	ms_timer += dt;
	if(ms_timer >= timeout_value_ms) { timer_timeout = true; }
	if(timer_timeout == true)
	{
		timer_timeout = false;

	}
	return diagnostic;
}
double SpeakerNodeProcess::compute_timetospeak(std::string s)
{
	return (double)(s.size()/(double)SPEECH_RATE);
}
/*
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
icarus_rover_v2::diagnostic SpeakerNodeProcess::new_usermessage(icarus_rover_v2::usermessage usermsg)
{
	if(usermsg.Level == LEVEL1)
	{
		level1_messages.push_back(usermsg);
	}
	else if(usermsg.Level == LEVEL2)
	{
		level2_messages.push_back(usermsg);
	}
	else if(usermsg.Level == LEVEL3)
	{
		level3_messages.push_back(usermsg);
	}
	else if(usermsg.Level == LEVEL4)
	{
		level4_messages.push_back(usermsg);
	}
	else
	{
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Level = WARN;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		char tempstr[512];
		sprintf(tempstr,"Received Level: %d But not Currently Supported.",usermsg.Level);
		diagnostic.Description = std::string(tempstr);
		return diagnostic;
	}
	diagnostic.Diagnostic_Type = SOFTWARE;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Processed UserMessage";
	return diagnostic;
}
std::vector<icarus_rover_v2::usermessage> SpeakerNodeProcess::get_usermessages(uint8_t level)
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
		std::vector<icarus_rover_v2::usermessage> emptylist;
		return emptylist;
	}
}
icarus_rover_v2::diagnostic SpeakerNodeProcess::new_commandmsg(icarus_rover_v2::command newcommand)
{
	
	return diagnostic;
}
icarus_rover_v2::diagnostic SpeakerNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	if((newdevice.DeviceName == myhostname) && (all_device_info_received == false))
	{
		mydevice = newdevice;
		all_device_info_received = true;

		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Description = "Node Initialized.";
	}
	else
	{
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Level = NOTICE;
		diagnostic.Diagnostic_Message = INITIALIZING;
		diagnostic.Description = "Node Still Initializing.";
	}
	return diagnostic;
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
*/
double SpeakerNodeProcess::time_diff(struct timeval timea, struct timeval timeb)
{
	long mtime, seconds, useconds;
	seconds  = timeb.tv_sec  - timea.tv_sec;
	useconds = timeb.tv_usec - timea.tv_usec;

	mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	return (double)(mtime)/1000.0;

}
