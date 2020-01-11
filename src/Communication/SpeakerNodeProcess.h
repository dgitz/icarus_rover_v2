#ifndef SPEAKERNODEPROCESS_H
#define SPEAKERNODEPROCESS_H
#include "../../include/Base/BaseNodeProcess.cpp"
#include <math.h>

#include "actionlib_msgs/GoalStatusArray.h"
#include <sound_play/SoundRequest.h>
#define SPEECH_RATE 12 //Characters per Second
#define LEVEL_MESSAGE_MAXCOUNT 20
#define PAUSE_BETWEEN 0.25f
class SpeakerNodeProcess: public BaseNodeProcess
{
public:

	///Initialization Functions
	SpeakerNodeProcess();
	~SpeakerNodeProcess();
	void initialize_stateack_messages();
	eros::diagnostic finish_initialization();
	void reset()
	{
		ready_to_speek = false;
		is_speaking = false;
		is_paused = false;
		speech_output = "";
		time_left_to_finish = 0.0;
		pause_timer = 0.0;
		level1_messages.clear();
		level2_messages.clear();
		level3_messages.clear();
		level4_messages.clear();
	}
	//Update Functions
	eros::diagnostic update(double t_dt,__attribute__((unused))double t_ros_time);
	int push_topiclist(std::string type,std::string name)
	{
		if(type == "eros/diagnostic")
		{
			for(std::size_t i = 0; i < diagnostic_topics.size();i++)
			{
				if(diagnostic_topics.at(i) == name)
				{
					return 0;
				}
			}
			diagnostic_topics.push_back(name);
			return 1;
		}
		return -1;
	}
	//Attribute Functions
	bool readytospeak() { return ready_to_speek; }
	bool isspeaking() { return is_speaking; }
	bool ispaused() { return is_paused; }
	std::string get_speechoutput() { return speech_output; }
	sound_play::SoundRequest get_speechoutput_client()
	{
		sound_play::SoundRequest msg;
		msg.arg = get_speechoutput();
		msg.sound = sound_play::SoundRequest::SAY;
		msg.command = sound_play::SoundRequest::PLAY_ONCE;
		msg.volume = 1.0;
		return msg;
	}
	double get_timeleft_tofinishspeaking() { return time_left_to_finish; }
	std::vector<eros::usermessage> get_usermessages(uint8_t level);

	//Message Functions
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_usermessage(const eros::usermessage::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);
	eros::diagnostic new_soundplaystatus();
	//Support Functions
	bool checkTriggers(std::vector<std::vector<unsigned char > > &tx_buffers);

	//Printing Functions
	
    //state_ack get_stateack(std::string name);
	//bool set_stateack(state_ack stateack);
	
protected:
	//state_ack send_speechmessage;

private:
	double compute_timetospeak(std::string s);
	std::vector<eros::diagnostic> check_programvariables();
	int id;
	long ms_timer;
	long timeout_value_ms;
	struct timeval init_time;
	bool timer_timeout;
    
	std::vector<eros::usermessage> level1_messages;
	std::vector<eros::usermessage> level2_messages;
	std::vector<eros::usermessage> level3_messages;
	std::vector<eros::usermessage> level4_messages;

	bool ready_to_speek;
	bool is_speaking;
	bool is_paused;
	std::string speech_output;
	double time_left_to_finish;
	double pause_timer;
	std::vector<std::string> diagnostic_topics;
};
#endif
