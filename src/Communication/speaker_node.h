// Derived class
#include "SpeakerNodeProcess.cpp"
#include "../../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
#include <boost/algorithm/string/replace.hpp>
//ROS Base Functionality
//ROS Messages

#include <sound_play/sound_play.h>


//Project
/*! \class SpeakerNode speaker_node.h "speaker_node.h"
 *  \brief This is a SpeakerNode class.  Used for the speaker_node node.
 *
 */
class SpeakerNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "speaker_node";

	const uint8_t MAJOR_RELEASE_VERSION = 3;
	const uint8_t MINOR_RELEASE_VERSION = 1;
	const uint8_t BUILD_NUMBER = 0;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 11-January-2020";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = COMMUNICATION_NODE;
	~SpeakerNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	SpeakerNodeProcess* get_process() { return process; }
	void thread_loop();

	//Cleanup
	void cleanup();
private:
	//Initialization Functions
	/*! \brief Read Node Specific Launch Parameters
	 *
	 */
	eros::diagnostic read_launchparameters();
	eros::diagnostic rescan_topics();
	/*! \brief Setup other pubs/subs, other node specific init stuff
	 *
	 */
	eros::diagnostic finish_initialization();
	//Update Functions
	bool run_001hz();
	bool run_01hz();
	bool run_01hz_noisy();
	bool run_1hz();
	bool run_10hz();
	bool run_loop1();
	bool run_loop2();
	bool run_loop3();
	//Attribute Functions
	//Message Functions
	void PPS1_Callback(const std_msgs::Bool::ConstPtr& t_msg);
	void Command_Callback(const eros::command::ConstPtr& t_msg);
	bool new_devicemsg(std::string query,eros::device t_device);
	void UserMessage_Callback(const eros::usermessage::ConstPtr& msg);
    void diagnostic_Callback(const eros::diagnostic::ConstPtr& msg);
	void SoundPlayStatus_Callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
	//Support Functions



	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	ros::Subscriber soundplaystatus_sub;
	SpeakerNodeProcess *process;
	ros::Subscriber UserMessage_sub;
	//boost::shared_ptr<sound_play::SoundClient> sc;
	ros::Time last_time_speech_ended;
	std::vector<std::string> speechbuffer;
	double initial_speech_wait;
    std::vector<ros::Subscriber> diagnostic_subs;
	ros::Publisher robot_sound_pub;
};
