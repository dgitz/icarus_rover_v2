#include "SnapshotNodeProcess.h"
eros::diagnostic SnapshotNodeProcess::set_config_filepaths(std::string t_filepath)
{
	eros::diagnostic diag = root_diagnostic;
	config_filepath = t_filepath;
	diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,"Config File Paths Set");
	return diag;
}
eros::diagnostic SnapshotNodeProcess::setInstanceMode(std::string t_mode)
{
	systemsnapshot_state = SnapshotState::NOTRUNNING;
	devicesnapshot_state = SnapshotState::NOTRUNNING;
	snapshot_name = "";
	eros::diagnostic diag = root_diagnostic;
	if(t_mode == "MASTER")
	{
		mode = InstanceMode::MASTER;
	}
	else if(t_mode == "SLAVE")
	{
		mode = InstanceMode::SLAVE;
	}
	else
	{
		mode = InstanceMode::UNKNOWN;
		char tempstr[128];
		sprintf(tempstr,"Mode: %s Not Supported.",t_mode.c_str());
		diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
		return diag;
	}
	diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,"Instance Mode OK.");
	return diag;
}
eros::diagnostic SnapshotNodeProcess::load_configfile(std::string path)
{
	eros::diagnostic diag = root_diagnostic;
	TiXmlDocument doc(path);
	bool configfile_loaded = doc.LoadFile();
	if(configfile_loaded == false)
	{
		char tempstr[512];
		sprintf(tempstr,"Unable to load: %s",path.c_str());
		diag = update_diagnostic(DATA_STORAGE,FATAL,INITIALIZING_ERROR,std::string(tempstr));
		return diag;
	}
	TiXmlElement *l_pRootElement = doc.RootElement();
	bool all_items_loaded = false;
	bool atleast_one_architecture_found = false;
	bool atleast_one_entity_found = false;
	bool destination_path_found = false;
	bool systemdestination_path_found = false;
	if( NULL != l_pRootElement )
	{
		TiXmlElement *l_pSnapshotConfig = l_pRootElement->FirstChildElement( "SnapshotConfig" );
		if(NULL != l_pSnapshotConfig)
		{
			TiXmlElement *l_pSystemSnapshotDestinationPath = l_pSnapshotConfig->FirstChildElement( "SystemSnapshotPath" );
			if(NULL != l_pSystemSnapshotDestinationPath)
			{
				systemdestination_path_found = true;
				systemsnapshot_directory = l_pSystemSnapshotDestinationPath->GetText();
			}

			TiXmlElement *l_pArchitecture = l_pSnapshotConfig->FirstChildElement( "Architecture" );
			if(NULL != l_pArchitecture)
			{
				while( l_pArchitecture )
				{
					SnapshotConfig config;
					config.architecture = l_pArchitecture->Attribute("type");
					atleast_one_architecture_found = true;
					TiXmlElement *l_pDestinationPath = l_pArchitecture->FirstChildElement( "DeviceSnapshotPath" );
					if(NULL != l_pDestinationPath)
					{
						destination_path_found = true;
						config.destination_path = l_pDestinationPath->GetText();
					}
					TiXmlElement *l_pFolder = l_pArchitecture->FirstChildElement( "Folder" );
					if(NULL != l_pFolder)
					{
						atleast_one_entity_found = true;
						while( l_pFolder )
						{

							config.folders.push_back(std::string(l_pFolder->GetText()));
							l_pFolder = l_pFolder->NextSiblingElement( "Folder" );
						}
					}
					TiXmlElement *l_pFile = l_pArchitecture->FirstChildElement( "File" );
					if(NULL != l_pFile)
					{
						atleast_one_entity_found = true;
						while( l_pFile )
						{
								
							config.files.push_back(std::string(l_pFile->GetText()));
							l_pFile = l_pFile->NextSiblingElement( "File" );
						}
					}
					TiXmlElement *l_pCommand = l_pArchitecture->FirstChildElement( "Command" );
					if(NULL != l_pCommand)
					{
						atleast_one_entity_found = true;
						while( l_pCommand )
						{
							Command cmd;
							cmd.command = l_pCommand->GetText();
							cmd.output_file = l_pCommand->Attribute("file");
							config.commands.push_back(cmd);
							//config.files.push_back(std::string(l_pFile->GetText()));
							l_pCommand = l_pCommand->NextSiblingElement( "Command" );
						}
					}
					snapshot_configlist.push_back(config);
					l_pArchitecture = l_pArchitecture->NextSiblingElement( "Architecture" );
				}
			}
		}
	}
	all_items_loaded = atleast_one_architecture_found && atleast_one_entity_found && destination_path_found && systemdestination_path_found;
	if(all_items_loaded == false)
	{
		diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Not all items loaded.");
		return diag;
	}
	else
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,"Config Loaded.");
		return diag;
	}


}
void SnapshotNodeProcess::clear_allsnapshots()
{
	char tempstr[256];
	sprintf(tempstr, "rm -r -f %s/*.zip", snapshot_config.destination_path.c_str());
	exec(tempstr, true); 
	//Do noting for now
}
void SnapshotNodeProcess::print_snapshotconfig(SnapshotNodeProcess::SnapshotConfig config)
{
	printf("---Snapshot Config: %s---\n",config.architecture.c_str());
	for(std::size_t i = 0; i < config.folders.size(); ++i)
	{
		printf("\t[%d] Folder: %s\n",(int)i,config.folders.at(i).c_str());
	}
	for(std::size_t i = 0; i < config.files.size(); ++i)
	{
		printf("\t[%d] File: %s\n",(int)i,config.files.at(i).c_str());
	}
}
eros::diagnostic  SnapshotNodeProcess::finish_initialization()
{
    eros::diagnostic diag = root_diagnostic;
	diag = load_configfile(config_filepath);
	diag = update_diagnostic(diag);
    return diag;
}
eros::diagnostic SnapshotNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if((initialized == true) and (ready == false))
	{
		bool found = false;
		for(std::size_t i = 0; i < snapshot_configlist.size(); ++i)
		{
			if(snapshot_configlist.at(i).architecture == mydevice.Architecture)
			{
				found = true;
				snapshot_config = snapshot_configlist.at(i);
			}
		}
		if(found == false)
		{
			char tempstr[256];
			sprintf(tempstr,"Unable to find Snapshot Config for My Architecture: %s",mydevice.Architecture.c_str());
			diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
			return diag;
		}
		ready = true;

	}
	diag = update_baseprocess(t_dt,t_ros_time);
	if(diag.Level <= NOTICE)
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error.");
		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running");
		
	}
	return diag;
}
eros::diagnostic SnapshotNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Updated Device");
	return diag;
}
std::vector<eros::diagnostic> SnapshotNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
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
	else if(t_msg->Command == ROVERCOMMAND_GENERATESNAPSHOT)
	{
		if(t_msg->Option1 == ROVERCOMMAND_SNAPSHOT_CLEARALL)
		{
			printf("[%s]SNAPSHOT-Clearing Snapshots.\n",node_name.c_str());
			clear_allsnapshots();
		}
		else if(t_msg->Option1 == ROVERCOMMAND_SNAPSHOT_NEWSNAPSHOT)
		{
			diaglist = createnew_snapshot();
		}
	}
	for(std::size_t i = 0; i < diaglist.size(); ++i)
	{
		diag = update_diagnostic(diaglist.at(i));
	}
	return diaglist;
}
std::vector<eros::diagnostic> SnapshotNodeProcess::check_programvariables()
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
std::vector<eros::diagnostic> SnapshotNodeProcess::createnew_snapshot()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	if((devicesnapshot_state == SnapshotState::RUNNING) or (devicesnapshot_state == SnapshotState::READY))
	{
		diag = update_diagnostic(DATA_STORAGE,DROPPING_PACKETS,WARN,"Snapshot is still being generated.");
		diaglist.push_back(diag);
		return diaglist;
	}
	
	devicesnapshot_state = SnapshotState::RUNNING;
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer,sizeof(buffer),"Snapshot_%Y_%m_%d_%H_%M_%S",timeinfo);
	std::string str(buffer);
	snapshot_name = mydevice.DeviceName + "_" + str;
	snapshot_path = snapshot_config.destination_path + snapshot_name;
	if (mkdir(snapshot_path.c_str(), 0777) == -1) 
    {
		char tempstr[512];
		sprintf(tempstr,"Unable to create snapshot folder: %s",snapshot_path.c_str());
		diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
	}
	for(std::size_t i = 0; i < snapshot_config.commands.size(); ++i)
	{
		char tempstr[1024];
		sprintf(tempstr,"%s > %s/%s",snapshot_config.commands.at(i).command.c_str(),
								   snapshot_path.c_str(),snapshot_config.commands.at(i).output_file.c_str());
		exec(tempstr,true);
	}
	//Copy all folders into new snapshot 
	for(std::size_t i = 0; i < snapshot_config.folders.size(); ++i)
	{
		char tempstr[1024];
		sprintf(tempstr, "cp -r %s %s/",  snapshot_config.folders.at(i).c_str(),snapshot_path.c_str());
		exec(tempstr, true); 
	}
	for(std::size_t i = 0; i < snapshot_config.files.size(); ++i)
	{
		char tempstr[1024];
		sprintf(tempstr, "cp %s %s/",  snapshot_config.files.at(i).c_str(),snapshot_path.c_str());
		exec(tempstr, true); 
	}
	//Zip it up
	{
		char tempstr[1024];
		sprintf(tempstr, "cd %s && zip %s.zip %s/*",  snapshot_config.destination_path.c_str(),snapshot_path.c_str(),snapshot_name.c_str());
		exec(tempstr, true); 
	}
	//Clean up folder
	{
		char tempstr[1024];
		sprintf(tempstr, "rm -r -f %s",  snapshot_path.c_str());
		exec(tempstr, true); 
	}
	char tempstr[512];
	sprintf(tempstr,"Snapshot Created at: %s",snapshot_path.c_str());
	diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,std::string(tempstr));
	diaglist.push_back(diag);
	devicesnapshot_state = SnapshotState::READY;
	return diaglist;
}
std::string SnapshotNodeProcess::exec(const char *cmd, bool wait_for_result)
{
	char buffer[512];
	std::string result = "";
	FILE *pipe = popen(cmd, "r");
	if (wait_for_result == false)
	{
		pclose(pipe);
		return "";
	}
	if (!pipe)
		throw std::runtime_error("popen() failed!");
	try
	{
		while (!feof(pipe))
		{
			if (fgets(buffer, 512, pipe) != NULL)
				result += buffer;
		}
	}
	catch (...)
	{
		pclose(pipe);
		throw;
	}
	return result;
}
std::string SnapshotNodeProcess::map_state_tostring(SnapshotState t_state)
{
	switch(t_state)
	{
		case SnapshotState::UNKNOWN:
			return "UNKNOWN";
			break;
		case SnapshotState::NOTRUNNING:
			return "NOT RUNNING";
			break;
		case SnapshotState::RUNNING:
			return "RUNNING";
			break;
		case SnapshotState::READY:
			return "READY";
			break;
		default:
			return "UNKNOWN";
			break;
	}
}