#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
/*! \class NetworkTransceiverNodeProcess NetworkTransceiverNodeProcess.h "NetworkTransceiverNodeProcess.h"
 *  \brief This is a NetworkTransceiverNodeProcess class.  Used for the networktransceiver_node node.
 *
 */
class NetworkTransceiverNodeProcess: public BaseNodeProcess {
public:
	//Constants
	//Enumstouch
	enum PriorityLevel
	{
		UNDEFINED = 0,
		HIGH = 1,
		MEDIUM =2,
		LOW = 3
	};
	//Structs
	struct Message
	{
		uint16_t id;
		std::string name;
		uint32_t sent_counter;
		uint32_t recv_counter;
		double sent_rate;
		double recv_rate;
		double target_sendrate;
		uint8_t priority_level;
	};
	struct QueueElement
	{
		uint16_t id;
		std::string item;
	};
	struct RemoteDevice
	{
		std::string Name;
		double current_beatepoch_sec;
		double expected_beatepoch_sec;
		double offset_sec;

	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	icarus_rover_v2::diagnostic finish_initialization();
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	icarus_rover_v2::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	void set_networkconfiguration(std::string t_multicast_group,int t_send_multicast_port,int t_recv_unicast_port);
	std::string get_multicast_group() { return multicast_group; }
	int get_send_multicast_port() { return send_multicast_port; }
	int get_recv_unicast_port() { return recv_unicast_port; }
	void set_UIMode(std::string t_UIMode) { UIMode = t_UIMode; }
	std::string get_UIMode() { return UIMode; }
	std::string get_messageinfo(bool v);
	std::vector<Message> get_messages() { return messages; };
	bool get_remoteheartbeatresult() { return remote_heartbeat_pass; }
	//Update Functions
	/*! \brief Push to the Topic List.
	 *  Returns -1 if the topic type was not found.  Returns 0 if the topic was already in the list, and 1 if the topic was added.
	 */
	int push_topiclist(std::string type,std::string name)
	{
		if(type == "icarus_rover_v2/resource")
		{
			bool found = true;
			for(int i = 0; i < resource_topics.size();i++)
			{
				if(resource_topics.at(i) == name)
				{
					return 0;
				}
			}
			resource_topics.push_back(name);
			return 1;
		}
		else if(type == "icarus_rover_v2/diagnostic")
		{
			bool found = true;
			for(int i = 0; i < diagnostic_topics.size();i++)
			{
				if(diagnostic_topics.at(i) == name)
				{
					return 0;
				}
			}
			diagnostic_topics.push_back(name);
			return 1;
		}
		else if(type == "icarus_rover_v2/device")
		{
			bool found = true;
			for(int i = 0; i < device_topics.size();i++)
			{
				if(device_topics.at(i) == name)
				{
					return 0;
				}
			}
			device_topics.push_back(name);
			return 1;
		}
		else if(type == "icarus_rover_v2/firmware")
		{
			bool found = true;
			for(int i = 0; i < firmware_topics.size();i++)
			{
				if(firmware_topics.at(i) == name)
				{
					return 0;
				}
			}
			firmware_topics.push_back(name);
			return 1;
		}

		return -1;
	}
	std::vector<std::string> get_topiclist(std::string type)
							{
		if(type == "icarus_rover_v2/resource")
		{
			return resource_topics;
		}
		else if(type == "icarus_rover_v2/diagnostic")
		{
			return diagnostic_topics;
		}
		else if(type == "icarus_rover_v2/device")
		{
			return device_topics;
		}
		else if(type == "icarus_rover_v2/firmware")
		{
			return firmware_topics;
		}

		std::vector<std::string> empty_list;
		return empty_list;
							}
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg);
	icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& device);
	icarus_rover_v2::diagnostic new_remoteheartbeatmsg(double timestamp,std::string name,double current_beat,double expected_beat);
	icarus_rover_v2::diagnostic new_message_sent(uint16_t id);
	icarus_rover_v2::diagnostic new_message_recv(uint16_t id);


	//Support Functions
	bool push_sendqueue(uint16_t id,std::string msg);
	std::vector<QueueElement> get_sendqueue(uint8_t level);

	//Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();
	icarus_rover_v2::diagnostic check_remoteHeartbeats();
	void init_messages();
	Message get_messagebyid(uint16_t id)
	{
		for(std::size_t i = 0; i < messages.size(); i++)
		{
			if(messages.at(i).id == id)
			{
				return messages.at(i);
			}
		}
		Message empty;
		return empty;
	}
	bool push_sendhighqueue(uint16_t id,std::string msg)
	{
		QueueElement elem;
		elem.id = id;
		elem.item = msg;
		Message m = get_messagebyid(id);
		if(m.sent_rate <= (1.5*m.target_sendrate))
		{
			sendqueue_highpriority.push_back(elem);
		}
		return true;
	}
	bool push_sendmediumqueue(uint16_t id,std::string msg)
	{
		QueueElement elem;
		elem.id = id;
		elem.item = msg;
		Message m = get_messagebyid(id);
		if(m.sent_rate <= (1.5*m.target_sendrate))
		{
			sendqueue_mediumpriority.push_back(elem);
		}

		return true;
	}
	bool push_sendlowqueue(uint16_t id,std::string msg)
	{
		QueueElement elem;
		elem.id = id;
		elem.item = msg;
		Message m = get_messagebyid(id);
		if(m.sent_rate <= (1.5*m.target_sendrate))
		{
			sendqueue_lowpriority.push_back(elem);
		}
		return true;
	}
	std::vector<Message> messages;
	std::vector<QueueElement> sendqueue_highpriority;
	std::vector<QueueElement> sendqueue_mediumpriority;
	std::vector<QueueElement> sendqueue_lowpriority;
	std::vector<RemoteDevice> remote_devices;
	bool remote_heartbeat_pass;
	std::string multicast_group;
	int send_multicast_port;
	int recv_unicast_port;
	std::string UIMode;
	std::vector<std::string> resource_topics;
	std::vector<std::string> diagnostic_topics;
	std::vector<std::string> device_topics;
	std::vector<std::string> firmware_topics;


};
