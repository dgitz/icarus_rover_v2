#include "webserver_node.h"
//Start User Code: Firmware Definition
#define WEBSERVERNODE_MAJOR_RELEASE 0
#define WEBSERVERNODE_MINOR_RELEASE 0
#define WEBSERVERNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
/*! \brief User Loop1 Code
 */
bool run_loop1_code()
{

	counter++;
	icarus_rover_v2::diagnostic diagnostic = process->update(1.0/(double)loop1_rate);
	{
	WebController::QueuedMessage msg;
	msg.priority = 3;
	diagnostic.Level = counter;
	msg.message = jsonhandler->encode_DiagnosticJSON(diagnostic);
	wc.new_pushmessagequeue(msg);
	}
	{
		WebController::QueuedMessage msg;
		msg.priority = 1;
		msg.message = jsonhandler->encode_Arm_StatusJSON(counter);
		wc.new_pushmessagequeue(msg);
	}

	if(diagnostic.Level > INFO)
	{
		diagnostic_pub.publish(diagnostic);
		logger->log_diagnostic(diagnostic);
	}
	return true;
}
/*! \brief User Loop2 Code
 */
bool run_loop2_code()
{
	wc.update(1/loop1_rate);
	return true;
}
/*! \brief User Loop3 Code
 */
bool run_loop3_code()
{

	return true;
}
/*! \brief 0.1 PULSE PER SECOND User Code
 */
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "webserver_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 5-November-2018";
	fw.Major_Release = WEBSERVERNODE_MAJOR_RELEASE;
	fw.Minor_Release = WEBSERVERNODE_MINOR_RELEASE;
	fw.Build_Number = WEBSERVERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	printf("t=%4.2f (sec) [%s]: %s\n",ros::Time::now().toSec(),node_name.c_str(),process->get_diagnostic().Description.c_str());
}
/*! \brief 1.0 PULSE PER SECOND User Code
 */
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	received_pps = true;
	if((process->get_initialized() == true) and (process->get_ready() == true))
	{
		icarus_rover_v2::diagnostic resource_diagnostic = resourcemonitor->update();
		if(resource_diagnostic.Diagnostic_Message == DEVICE_NOT_AVAILABLE)
		{
			diagnostic_pub.publish(resource_diagnostic);
			logger->log_warn("Couldn't read resources used.");
		}
		else if(resource_diagnostic.Level >= WARN)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
			diagnostic_pub.publish(resource_diagnostic);
			logger->log_diagnostic(resource_diagnostic);
		}
		else if(resource_diagnostic.Level <= NOTICE)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
	}
	else if((process->get_ready() == false) and (process->get_initialized() == true))
	{

	}
	else if(process->get_initialized() == false)
	{
		{
			icarus_rover_v2::srv_device srv;
			srv.request.query = "SELF";
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() != 1)
				{
					logger->log_error("Got unexpected device message");
				}
				else
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(0));
				}
			}
			srv.request.query = "ALL";
			if(srv_device.call(srv) == true)
			{
				bool status = new_devicelistmsg(srv.request.query,srv.response.data);
			}
		}
	}
	diagnostic_pub.publish(process->get_diagnostic());
}
void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	icarus_rover_v2::command command;
	command.Command = msg->Command;
	command.Option1 = msg->Option1;
	command.Option2 = msg->Option2;
	command.Option3 = msg->Option3;
	command.CommandText = msg->CommandText;
	command.Description = msg->Description;
	std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(command);
	for(std::size_t i = 0; i < diaglist.size(); i++)
	{
		if(diaglist.at(i).Level >= NOTICE)
		{
			logger->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
		if((diaglist.at(i).Level > NOTICE) and (process->get_runtime() > 10.0))
		{
			printf("[%s]: %s\n",node_name.c_str(),diaglist.at(i).Description.c_str());
		}
	}
}
bool WebController::queryResponse(uint8_t msg,Mongoose::CHttpConnWrapper& httpConn,const char *postBody, size_t postBodyLength,bool& keepOpen)
{
	std::string response = jsonhandler->encode_DeviceJSON(device_list);
	printf("sending: %s\n",response.c_str());
	m_currentConn->HttpSendResponse(Mongoose::HttpStatus_SUCCESS_OK, response.c_str(), false, NULL);
	return true;
}
WebController::WebController(): m_mongoose(this)
{

}
bool WebController::initialize(std::string doc_path,int port)
{
	m_mongoose.m_wwwRootFolder = doc_path.c_str();
	bool status = m_mongoose.Initialize(std::to_string(port));
	if(status == true)
	{
		printf("[WebServer]: Mongoose Initialized.\n");
	}
	else
	{
		printf("[WebServer]: Mongoose Failed.\n");
	}
	return status;
}
bool WebController::new_pushmessagequeue(QueuedMessage msg)
{
	message_queue.push_back(msg);
	return true;
}
bool WebController::update(double dt)
{
	m_mongoose.unregisterExpiredSessions();
	m_mongoose.CallServiceEvents(dt);
	if(message_queue.size() > 0)
	{
		QueuedMessage msg = message_queue.front();
		message_queue.erase(message_queue.begin());
		m_mongoose.BroadcastWebSocketPacket(msg.message.c_str());
	}

	return true;
}
bool WebController::refreshConn(Mongoose::CHttpConnWrapper& httpConn)
{
	m_currentConn_Mutex.lock();
	if(m_currentConn != NULL)
	{
		delete m_currentConn;
		m_currentConn = NULL;
	}
	m_currentConn = new Mongoose::CHttpConnWrapper(httpConn);
	m_currentConn_Mutex.unlock();
	return true;
}
void WebController::HttpServicePostCommandCallback(Mongoose::CHttpConnWrapper& httpConn, const std::string& URI, const std::string& URIParameters,
		const char *postBody, size_t postBodyLength, bool& keepOpen )
{
	refreshConn(httpConn);
	processing_command = true;
	if( postBody  &&  (postBodyLength > 0) )
	{
		printf("Post Data and length: %p (size %d)",postBody, postBodyLength);
	}
	uint8_t command;
	std::string response = "";
	queryResponse(command,httpConn,postBody,postBodyLength,keepOpen);

}
//End User Code: Functions
bool run_10Hz_code()
{
	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
	if(process->get_diagnostic().Level > NOTICE)
	{
		diagnostic_pub.publish(process->get_diagnostic());
		logger->log_diagnostic(process->get_diagnostic());
	}
	return true;
}





int main(int argc, char **argv)
{
	counter = 0;
	node_name = "webserver_node";
	ros::init(argc, argv, node_name);
	n.reset(new ros::NodeHandle);
	node_name = ros::this_node::getName();
	ros::NodeHandle n;

	if(initializenode() == false)
	{
		char tempstr[256];
		sprintf(tempstr,"Unable to Initialize. Exiting.");
		printf("[%s]: %s\n",node_name.c_str(),tempstr);
		logger->log_fatal(tempstr);
		kill_node = 1;
	}
	ros::Rate loop_rate(ros_rate);
	boot_time = ros::Time::now();
	last_10Hz_timer = ros::Time::now();
	double mtime;
	while (ros::ok() && (kill_node == 0))
	{
		bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
		if(ok_to_start == true)
		{
			if(run_loop1 == true)
			{
				mtime = measure_time_diff(ros::Time::now(),last_loop1_timer);
				if(mtime >= (1.0/loop1_rate))
				{
					run_loop1_code();
					last_loop1_timer = ros::Time::now();
				}
			}
			if(run_loop2 == true)
			{
				mtime = measure_time_diff(ros::Time::now(),last_loop2_timer);
				if(mtime >= (1.0/loop2_rate))
				{
					run_loop2_code();
					last_loop2_timer = ros::Time::now();
				}
			}
			if(run_loop3 == true)
			{
				mtime = measure_time_diff(ros::Time::now(),last_loop3_timer);
				if(mtime >= (1.0/loop3_rate))
				{
					run_loop3_code();
					last_loop3_timer = ros::Time::now();
				}
			}

			mtime = measure_time_diff(ros::Time::now(),last_10Hz_timer);
			if(mtime >= 0.1)
			{
				run_10Hz_code();
				last_10Hz_timer = ros::Time::now();
			}
		}
		else
		{
			logger->log_warn("Waiting on PPS to Start.");
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	logger->log_notice("Node Finished Safely.");
	return 0;
}
bool initializenode()
{
	//Start Template Code: Initialization, Parameters and Topics
	kill_node = 0;
	signal(SIGINT,signalinterrupt_handler);
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  n->advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,5);
	icarus_rover_v2::diagnostic diagnostic;
	diagnostic.DeviceName = hostname;
	diagnostic.Node_Name = node_name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = TIMING_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = n->advertise<icarus_rover_v2::resource>(resource_topic,5);

	std::string param_verbosity_level = node_name +"/verbosity_level";
	if(n->getParam(param_verbosity_level,verbosity_level) == false)
	{
		logger = new Logger("WARN",ros::this_node::getName());
		logger->log_warn("Missing Parameter: verbosity_level");
		return false;
	}
	else
	{
		logger = new Logger(verbosity_level,ros::this_node::getName());
	}
	std::string param_disabled = node_name +"/disable";
	bool disable_node;
	if(n->getParam(param_disabled,disable_node) == true)
	{
		if(disable_node == true)
		{
			logger->log_notice("Node Disabled in Launch File.  Exiting.");
			printf("[%s]: Node Disabled in Launch File. Exiting.\n",node_name.c_str());
			return false;
		}
	}

	std::string heartbeat_topic = "/" + node_name + "/heartbeat";
	heartbeat_pub = n->advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,5);
	beat.Node_Name = node_name;

	std::string param_startup_delay = node_name + "/startup_delay";
	double startup_delay = 0.0;
	if(n->getParam(param_startup_delay,startup_delay) == false)
	{
		logger->log_notice("Missing Parameter: startup_delay.  Using Default: 0.0 sec.");
	}
	else
	{
		char tempstr[128];
		sprintf(tempstr,"Using Parameter: startup_delay = %4.2f sec.",startup_delay);
		logger->log_notice(std::string(tempstr));
	}
	printf("[%s] Using Parameter: startup_delay = %4.2f sec.\n",node_name.c_str(),startup_delay);
	ros::Duration(startup_delay).sleep();

	std::string device_topic = "/" + std::string(hostname) + "_master_node/srv_device";
	srv_device = n->serviceClient<icarus_rover_v2::srv_device>(device_topic);

	pps01_sub = n->subscribe<std_msgs::Bool>("/01PPS",5,PPS01_Callback);
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",5,PPS1_Callback);
	command_sub = n->subscribe<icarus_rover_v2::command>("/command",5,Command_Callback);
	std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
	if(n->getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
	std::string firmware_topic = "/" + node_name + "/firmware";
	firmware_pub =  n->advertise<icarus_rover_v2::firmware>(firmware_topic,1);

	double max_rate = 0.0;
	std::string param_loop1_rate = node_name + "/loop1_rate";
	if(n->getParam(param_loop1_rate,loop1_rate) == false)
	{
		logger->log_warn("Missing parameter: loop1_rate.  Not running loop1 code.");
		run_loop1 = false;
	}
	else
	{
		last_loop1_timer = ros::Time::now();
		run_loop1 = true;
		if(loop1_rate > max_rate) { max_rate = loop1_rate; }
	}

	std::string param_loop2_rate = node_name + "/loop2_rate";
	if(n->getParam(param_loop2_rate,loop2_rate) == false)
	{
		logger->log_warn("Missing parameter: loop2_rate.  Not running loop2 code.");
		run_loop2 = false;
	}
	else
	{
		last_loop2_timer = ros::Time::now();
		run_loop2 = true;
		if(loop2_rate > max_rate) { max_rate = loop2_rate; }
	}

	std::string param_loop3_rate = node_name + "/loop3_rate";
	if(n->getParam(param_loop3_rate,loop3_rate) == false)
	{
		logger->log_warn("Missing parameter: loop3_rate.  Not running loop3 code.");
		run_loop3 = false;
	}
	else
	{
		last_loop3_timer = ros::Time::now();
		run_loop3 = true;
		if(loop3_rate > max_rate) { max_rate = loop3_rate; }
	}
	ros_rate = max_rate * 50.0;
	if(ros_rate < 100.0) { ros_rate = 100.0; }
	char tempstr[512];
	sprintf(tempstr,"Running Node at Rate: %f",ros_rate);
	logger->log_notice(std::string(tempstr));
	//End Template Code: Initialization and Parameters

	//Start User Code: Initialization and Parameters
	processing_command = false;
	process = new WebServerNodeProcess;

	diagnostic = process->init(diagnostic,std::string(hostname));
	if(diagnostic.Level > NOTICE)
	{
		logger->log_fatal(diagnostic.Description);
		printf("[%s]: %s\n",node_name.c_str(),diagnostic.Description.c_str());
		return false;
	}
	std::string doc_path;
	std::string param_docpath = node_name + "/www";
	if(n->getParam(param_docpath,doc_path) == false)
	{
		logger->log_error("Missing parameter: www.  Exiting.");
		return false;
	}
	int port;
	std::string param_port = node_name + "/ServerPort";
	if(n->getParam(param_port,port) == false)
	{
		logger->log_error("Missing parameter: ServerPort.  Exiting.");
		return false;
	}
	if(wc.initialize(doc_path,port) == false)
	{
		std::string tempstr = "Mongoose Server Did not Initialize. Exiting.";
		printf("%s\n",tempstr.c_str());
		logger->log_error(tempstr);
		return false;
	}

	//Finish User Code: Initialization and Parameters

	//Start Template Code: Final Initialization.
	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Node Initialized";
	process->set_diagnostic(diagnostic);
	diagnostic_pub.publish(diagnostic);
	logger->log_info("Initialized!");
	return true;
	//End Template Code: Finish Initialization.
}
//Start Template Code: Functions
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
bool new_devicemsg(std::string query,icarus_rover_v2::device device)
{

	if(query == "SELF")
	{
		if((device.DeviceName == hostname))
		{
			resourcemonitor = new ResourceMonitor(process->get_diagnostic(),device.Architecture,device.DeviceName,node_name);
			process->set_mydevice(device);
		}
	}
	if((process->get_initialized() == true))
	{
		icarus_rover_v2::diagnostic diag = process->new_devicemsg(device);
	}
	return true;
}
bool new_devicelistmsg(std::string query,std::vector<icarus_rover_v2::device> list)
{

	if(query == "ALL")
	{
		process->update_systemdevicelist(list);
		wc.set_devicelist(list);
	}
	return true;
}
void signalinterrupt_handler(int sig)
{
	exit(0);
	kill_node = 1;
}
//End Template Code: Functions
