#include "database_node.h"
bool kill_node = false;
bool DatabaseNode::start(int argc, char **argv)
{
	bool status = false;
	process = new DatabaseNodeProcess();
	set_basenodename(BASE_NODE_NAME);
	initialize_firmware(MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
	diagnostic = preinitialize_basenode(argc, argv);
	if (diagnostic.Level > WARN)
	{
		return false;
	}
	diagnostic = read_launchparameters();
	if (diagnostic.Level > WARN)
	{
		return false;
	}

	process->initialize(get_basenodename(), get_nodename(), get_hostname(), DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	diagnostic = finish_initialization();
	if (diagnostic.Level > WARN)
	{
		return false;
	}
	if (diagnostic.Level < WARN)
	{
		diagnostic.Diagnostic_Type = NOERROR;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Description = "Node Configured.  Initializing.";
		get_logger()->log_diagnostic(diagnostic);
	}

	status = true;
	return status;
}

eros::diagnostic DatabaseNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic DatabaseNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS", 1, &DatabaseNode::PPS1_Callback, this);
	command_sub = n->subscribe<eros::command>("/command", 1, &DatabaseNode::Command_Callback, this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	std::string database_path;
	std::string param_database_path = node_name +"/database_path";
	if(n->getParam(param_database_path,database_path) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing Parameter: database_path. Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	db_fd = sqlite3_open(database_path.c_str(), &db);
	if( db_fd ) 
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Can't Open Database at: " + database_path + " with error: " +
			std::string(sqlite3_errmsg(db)) + " Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	} 
	else 
	{
		diag = process->update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,"Opened database at: " + database_path + ".");
		logger->log_diagnostic(diag);
	}

	std::string srv_sql_topic = "/ConfigDatabase/srv_sql";
	sql_srv = n->advertiseService(srv_sql_topic,&DatabaseNode::sql_service,this);
	
	return diagnostic;
}
bool DatabaseNode::run_001hz()
{
	
	return true;
}
bool DatabaseNode::run_01hz()
{
	return true;
}
bool DatabaseNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool DatabaseNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if ((process->is_initialized() == true) and (process->is_ready() == true))
	{
	}
	else if ((process->is_ready() == false) and (process->is_initialized() == true))
	{
	}
	else if (process->is_initialized() == false)
	{
		{
			eros::srv_device srv;
			srv.request.query = "SELF";
			if (srv_device.call(srv) == true)
			{
				if (srv.response.data.size() != 1)
				{

					get_logger()->log_error("Got unexpected device message.");
				}
				else
				{
					new_devicemsg(srv.request.query, srv.response.data.at(0));
				}
			}
			else
			{
			}
		}
	}
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		if (diaglist.at(i).Level == WARN)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}
	return true;
}
bool DatabaseNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1, ros::Time::now().toSec());
	if (diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		if (diaglist.at(i).Level > WARN)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}
	return true;
}
bool DatabaseNode::run_loop1()
{
	return true;
}
bool DatabaseNode::run_loop2()
{
	return true;
}
bool DatabaseNode::run_loop3()
{
	return true;
}
bool DatabaseNode::sql_service(eros::srv_sql::Request &req,
		eros::srv_sql::Response &res)
{
	eros::diagnostic diag = diagnostic;
	char *zErrMsg = 0;
	RecordList recordlist;  
	std::string tempstr;
	switch(req.type)
	{
		case SQLCOMMANDTYPE_DATAMANIPULATION:
			db_fd = sqlite3_exec(db, req.cmd.c_str(), DatabaseNode::database_update, &recordlist, &zErrMsg);
			if( db_fd != SQLITE_OK ) 
			{
				res.status = -1;
				sqlite3_free(zErrMsg);
			}
			break;
		case SQLCOMMANDTYPE_DATAQUERY:
			db_fd = sqlite3_exec(db, req.cmd.c_str(), DatabaseNode::database_query, &recordlist, &zErrMsg);
			if( db_fd != SQLITE_OK ) 
			{
				res.status = -1;
				sqlite3_free(zErrMsg);
			} 
			else 
			{
				res.fields = recordlist.fields;
				std::vector<std::string> list;
				for(std::size_t i = 0; i < recordlist.records.size(); ++i)
				{
					tempstr = "";
					for(std::size_t j = 0; j < recordlist.records.at(i).size(); ++j)
					{
						tempstr += recordlist.records.at(i).at(j) + ",";
					}
					list.push_back(tempstr);
				}
				if(list.size() == 0)
				{
					res.status = 0;
				}
				else
				{
					res.status = 1;
				}
				
				res.response = list;
			}
			break;
		default:
			res.status = -1;
			res.response.push_back("Unsupported Command Type: " + std::to_string(req.type));
			diag = process->update_diagnostic(DATA_STORAGE,WARN,DROPPING_PACKETS,res.response.at(0));
			logger->log_diagnostic(diag);
			break;
	}
	
	return true;
}
int DatabaseNode::database_query(void *p_data, int num_fields, char **p_fields,  char **p_col_names)
{
   	RecordList* recordlist = static_cast<RecordList*>(p_data);
	try
	{
		std::vector<std::string> fields(p_col_names, p_col_names + (num_fields)*sizeof(p_col_names)/(sizeof p_col_names[0]));
		recordlist->fields = fields;
		recordlist->records.emplace_back(p_fields,p_fields + num_fields);
	}
	catch (std::exception e) 
	{
		printf("ex: %s\n",e.what());
		return 1;
	}
   return 0;
}
int DatabaseNode::database_update(void *data, int argc, char **argv, char**azColName)
{
	return 0;
}
void DatabaseNode::PPS1_Callback(const std_msgs::Bool::ConstPtr &msg)
{
	new_ppsmsg(msg);
}

void DatabaseNode::Command_Callback(const eros::command::ConstPtr &t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg, diaglist);
}
bool DatabaseNode::new_devicemsg(std::string query, eros::device t_device)
{
	if (query == "SELF")
	{
		if (t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			process->set_mydevice(t_device);
		}
	}

	if ((process->is_initialized() == true))
	{
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::diagnostic diag = process->new_devicemsg(device_ptr);
	}
	return true;
}
void DatabaseNode::thread_loop()
{
	while (kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void DatabaseNode::cleanup()
{
	sqlite3_close(db);
	base_cleanup();
	get_logger()->log_info("Node Finished Safely.");
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	printf("Killing Node with Signal: %d", sig);
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv)
{
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	DatabaseNode *node = new DatabaseNode();
	bool status = node->start(argc, argv);
	std::thread thread(&DatabaseNode::thread_loop, node);
	while ((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->cleanup();
	thread.detach();
	return 0;
}
