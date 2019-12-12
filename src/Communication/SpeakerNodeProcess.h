#ifndef SPEAKERNODEPROCESS_H
#define SPEAKERNODEPROCESS_H
#include "../../include/Base/BaseNodeProcess.cpp"
#include <serialmessage.h>
#include <math.h>

#include "actionlib_msgs/GoalStatusArray.h"
#define SPEECH_RATE 12 //Characters per Second
#define PAUSE_BETWEEN 0.25f
/*
struct state_ack
{
	std::string name;
	bool state;
	bool trigger;
	bool retrying;
	struct timeval orig_send_time;
	struct timeval retry_send_time;
	uint16_t retries;
	uint16_t timeout_counter;
	bool retry_mode;
	bool failed;
	int flag1;  //Various Purposes
	double stream_rate;
};
*/
class SpeakerNodeProcess: public BaseNodeProcess
{
public:

	///Initialization Functions
	SpeakerNodeProcess();
	~SpeakerNodeProcess();
	void initialize_stateack_messages();
	eros::diagnostic finish_initialization();
	//Update Functions
	eros::diagnostic update(double t_dt,double t_ros_time);
	
	//Attribute Functions
	bool readytospeak() { return ready_to_speek; }
	bool isspeaking() { return is_speaking; }
	bool ispaused() { return is_paused; }
	std::string get_speechoutput() { return speech_output; }
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
};
#endif
