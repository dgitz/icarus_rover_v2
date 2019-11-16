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
	eros_systemsnapshot_state.state = map_state_tostring(systemsnapshot_state);
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
		
			TiXmlElement *l_pDataLog = l_pSnapshotConfig->FirstChildElement("DataLog");
			if(NULL != l_pDataLog )
			{
				TiXmlElement *l_pDevice = l_pDataLog->FirstChildElement("Device");
				if(NULL != l_pDevice)
				{
					datalog_device = l_pDevice->GetText();
				}
				else
				{
					diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Device Config Missing from SnapshotConfig.xml.");
					return diag;				
				}

				TiXmlElement *l_pStorageDirectory = l_pDataLog->FirstChildElement("StorageDirectory");
				if(NULL != l_pStorageDirectory)
				{
					datalog_directory = l_pStorageDirectory->GetText();
				}
				else
				{
					diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"StorageDirectory Config Missing from SnapshotConfig.xml.");
					return diag;
				}				
			}
			else
			{
				diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"DataLog Config Section Missing from SnapshotConfig.xml.");
				return diag;
			}
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
							std::string folder = l_pFolder->GetText();
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
	{
		devicesnapshot_state = SnapshotState::NOTRUNNING;
		systemsnapshot_state = SnapshotState::NOTRUNNING;
		char tempstr[256];
		sprintf(tempstr, "rm -r -f %s/*.zip", snapshot_config.destination_path.c_str());
		exec(tempstr, true); 
	}
	{
		char tempstr[256];
		sprintf(tempstr, "rm -r -f %s/*.zip", systemsnapshot_directory.c_str());
		exec(tempstr, true); 
	}
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
	for(std::size_t i = 0; i < config.commands.size(); ++i)
	{
		printf("\t[%d] Command: %s\n",(int)i,config.commands.at(i).command.c_str());
	}
}
eros::diagnostic  SnapshotNodeProcess::finish_initialization()
{
    eros::diagnostic diag = root_diagnostic;
	diag = load_configfile(config_filepath);
	diag = update_diagnostic(diag);
    return diag;
}
eros::diagnostic SnapshotNodeProcess::update_slow()
{
	eros::diagnostic diag = root_diagnostic;
	int v = count_files_indirectory(systemsnapshot_directory,"*.zip");
	eros_systemsnapshot_state.systemsnapshot_count = (uint16_t)v;
	if(systemsnapshot_state == SnapshotState::NOTRUNNING)
	{
		eros_systemsnapshot_state.percent_complete = 0;
	}
	return diag;
}
eros::diagnostic SnapshotNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if((initialized == true) and (ready == false))
	{
		if(mydevice.DeviceName != datalog_device)
		{
			std::string tempstr = "SnapshotConfig DataLog Device Mis-Match: " + mydevice.DeviceName + " Did not match: " + datalog_device;
			//diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,tempstr);
			//return diag;
		}
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
	if(getInstanceMode() == InstanceMode::MASTER)
	{
		if(run_systemsnapshot_timeout_timer == true)
		{
			systemsnapshot_timeout_timer += t_dt;
		}
		else
		{
			systemsnapshot_timeout_timer = 0.0;
		}
		if((systemsnapshot_state == SnapshotState::RUNNING) and (systemsnapshot_timeout_timer >= SYSTEMSNAPSHOT_TIMEOUT))
		{
			missing_snapshots = filtered_snapshot_devices;
			filtered_snapshot_devices.clear(); //Does this cause a seg fault?
			systemsnapshot_state = SnapshotState::INCOMPLETE;
			systemsnapshot_info.state = systemsnapshot_state;
			eros_systemsnapshot_state.percent_complete = 100;
			diag = finishSystemSnapshot();
		}
		else if((systemsnapshot_state == SnapshotState::COMPLETE))
		{
			systemsnapshot_info.state = systemsnapshot_state;
			filtered_snapshot_devices.clear();
			diag = finishSystemSnapshot();
		}
		if((systemsnapshot_state == SnapshotState::INCOMPLETE) or (systemsnapshot_state == SnapshotState::COMPLETE) or (systemsnapshot_state == SnapshotState::READY))
		{
			systemsnapshot_timeout_timer = 0.0;
			run_systemsnapshot_timeout_timer = false;
		}
	}
	
	if(diag.Level <= NOTICE)
	{
		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running");
		
	}
	eros_systemsnapshot_state.state = map_state_tostring(systemsnapshot_state);
	return diag;
}
eros::diagnostic SnapshotNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Updated Device");
	if(device->DeviceName != mydevice.DeviceName)
	{
		if((device->DeviceType == "ControlModule") or (device->DeviceType == "ComputeModule"))
		{
			all_snapshot_devices.push_back(device->DeviceName);
		}
	}
	
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
			clear_allsnapshots();
		}
		else if(t_msg->Option1 == ROVERCOMMAND_SNAPSHOT_NEWSNAPSHOT)
		{
			diaglist = createnew_snapshot(t_msg->Option2,t_msg->CommandText,t_msg->Description);
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
eros::diagnostic SnapshotNodeProcess::finishSystemSnapshot()
{
	eros::diagnostic diag = root_diagnostic;
	systemsnapshot_info.rostime_stop = getROSTime();
	std::string infotext = generate_systemsnapshotinfo(systemsnapshot_info);
	std::string info_filepath = systemsnapshot_path + "/SystemSnapshotInfo.txt";
	std::ofstream info_fd (info_filepath.c_str(), std::ofstream::out);
	if(info_fd.is_open() == false)
	{
		char tempstr[512];
		sprintf(tempstr,"Unable to create System Info File: %s",info_filepath.c_str());
		diag = update_diagnostic(DATA_STORAGE,WARN,DROPPING_PACKETS,std::string(tempstr));
		return diag;
	}
	info_fd << infotext;
	info_fd.close();
	//Look for bag file
	{
		bool bagfile_ready = false;
		double dt = 0.1;
		double etime = 0.0;
		while(etime < SYSTEMSNAPSHOT_TIMEOUT)
		{
			int file_founds = count_files_indirectory(datalog_directory,"*.bag");
			if(file_founds >= 1)
			{
				char tempstr[1024];
				sprintf(tempstr, "mv %s*.bag %s",  datalog_directory.c_str(),systemsnapshot_path.c_str());
				exec(tempstr, true); 
				bagfile_ready = true;
				break;
			}
			etime += dt;
		}
		if(bagfile_ready == false)
		{
			diag = update_diagnostic(DATA_STORAGE,WARN,DROPPING_PACKETS,"DataLog BAG file missing.");			
		}
	}
	//Zip it up
	if(1)
	{
		char tempstr[1024];
		sprintf(tempstr, "cd %s && zip -r %s.zip %s/*", systemsnapshot_directory.c_str(),systemsnapshot_path.c_str(),systemsnapshot_name.c_str());
		active_systemsnapshot_completepath = systemsnapshot_directory + systemsnapshot_name + ".zip";
		exec(tempstr, true); 
	}
	//Make sure it's actually there
	{
		int file_found = count_files_indirectory(systemsnapshot_directory,systemsnapshot_name+".zip");
		if(file_found != 1)
		{
			char tempstr[1024];
			sprintf(tempstr,"System Snapshot not created: %s",systemsnapshot_name.c_str());
			diag = update_diagnostic(DATA_STORAGE,ERROR,DROPPING_PACKETS,std::string(tempstr));
			return diag;
		}
	}
	
	//Clean up folder
	{
		char tempstr[1024];
		sprintf(tempstr, "rm -r -f %s",  systemsnapshot_path.c_str());
		exec(tempstr, true); 
	}
	char tempstr[512];
	if(systemsnapshot_state == SnapshotState::COMPLETE)
	{
		sprintf(tempstr,"System Snapshot: %s Generated Completely.",systemsnapshot_name.c_str());
		diag = update_diagnostic(DATA_STORAGE,NOTICE,NOERROR,std::string(tempstr));
		systemsnapshot_state = SnapshotState::READY;
		
	}
	else if(systemsnapshot_state == SnapshotState::INCOMPLETE)
	{
		std::string tempstr2;
		for(std::size_t i = 0; i < missing_snapshots.size(); ++i)
		{
			tempstr2 += missing_snapshots.at(i);
			if(i < (missing_snapshots.size() -1))
			{
				tempstr2 += ",";
			}
		}
		sprintf(tempstr,"System Snapshot: %s Generated but is missing snapshots from: %s.",systemsnapshot_name.c_str(),tempstr2.c_str());
		diag = update_diagnostic(DATA_STORAGE,NOTICE,DROPPING_PACKETS,std::string(tempstr));
		//systemsnapshot_state = SnapshotState::READY;
	}
	if((systemsnapshot_state == SnapshotState::READY) or (systemsnapshot_state == SnapshotState::INCOMPLETE))
	{
		eros_systemsnapshot_state.source_device = mydevice.DeviceName;
		eros_systemsnapshot_state.snapshot_path = active_systemsnapshot_completepath;
	}
	return diag;
}
bool SnapshotNodeProcess::received_snapshot_fromdevice(std::string devicename)
{
	bool found = false;
	int index = -1;
	eros::diagnostic diag = root_diagnostic;
	for(std::size_t i = 0; i < filtered_snapshot_devices.size(); ++i )
	{
		
		if(filtered_snapshot_devices.at(i) == devicename)
		{
			index = i;
			found = true;
		}
	}
	if(found == true)
	{
		filtered_snapshot_devices.erase(filtered_snapshot_devices.begin() + index);
		double v = 1.0/(double)((uint32_t)all_snapshot_devices.size()+1);
		eros_systemsnapshot_state.percent_complete+= (uint8_t)(100.0*v);
		systemsnapshot_info.devices.push_back(devicename);
	}
	if(filtered_snapshot_devices.size() == 0)
	{
		systemsnapshot_state = SnapshotState::COMPLETE;
		eros_systemsnapshot_state.percent_complete = 100;
	}
	return found;
}
std::vector<eros::diagnostic> SnapshotNodeProcess::createnew_snapshot(uint8_t snapshot_mode,std::string command_text,std::string command_description)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	if((devicesnapshot_state == SnapshotState::RUNNING) or (devicesnapshot_state == SnapshotState::READY) or (systemsnapshot_state == SnapshotState::RUNNING))
	{
		diag = update_diagnostic(DATA_STORAGE,DROPPING_PACKETS,WARN,"Snapshot is still being generated.");
		diaglist.push_back(diag);
		return diaglist;
	}
	systemsnapshot_info.rostime_start = getROSTime();
	
	filtered_snapshot_devices = all_snapshot_devices;
	devicesnapshot_state = SnapshotState::RUNNING;
	eros_systemsnapshot_state.percent_complete = 0;
	systemsnapshot_info.devices.clear();
	systemsnapshot_info.generate_commandtext = command_text;
	systemsnapshot_info.generate_commanddescription = command_description;
	systemsnapshot_info.snapshot_mode = snapshot_mode;
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
		try
		{
			exec(tempstr,true);
		}
		catch(std::exception e)
		{
			std::string tempstr = "Command Exec failed with error: " + std::string(e.what());
			diag = update_diagnostic(DATA_STORAGE,WARN,DROPPING_PACKETS,tempstr);
		}
	}
	//Copy all folders into new snapshot 
	for(std::size_t i = 0; i < snapshot_config.folders.size(); ++i)
	{
		char tempstr1[1024];
		sprintf(tempstr1,"mkdir -p %s %s/%s",
				snapshot_config.folders.at(i).c_str(),
				snapshot_path.c_str(),
				snapshot_config.folders.at(i).c_str());
		try
		{
			printf("exec: %s\n",tempstr1);
			exec(tempstr1, true); 
		}
		catch(std::exception e)
		{
			std::string tempstr = "Folder Create failed with error: " + std::string(e.what());
			diag = update_diagnostic(DATA_STORAGE,WARN,DROPPING_PACKETS,tempstr);
		}
		char tempstr2[1024];
		sprintf(tempstr2, "rsync -a --exclude='.*' %s %s/%s",  
			snapshot_config.folders.at(i).c_str(),
			snapshot_path.c_str(),
			snapshot_config.folders.at(i).c_str());
		try
		{
			printf("exec: %s\n",tempstr2);
			exec(tempstr2, true); 
		}
		catch(std::exception e)
		{
			std::string tempstr = "Folder Copy failed with error: " + std::string(e.what());
			diag = update_diagnostic(DATA_STORAGE,WARN,DROPPING_PACKETS,tempstr);
		}
	}
	for(std::size_t i = 0; i < snapshot_config.files.size(); ++i)
	{
		char tempstr2[1024];
		sprintf(tempstr2, "cp %s %s",  
			snapshot_config.files.at(i).c_str(),
			snapshot_path.c_str());
		try
		{
			exec(tempstr2, true); 
		}
		catch(std::exception e)
		{
			std::string tempstr = "File Copy failed with error: " + std::string(e.what());
			diag = update_diagnostic(DATA_STORAGE,WARN,DROPPING_PACKETS,tempstr);
		}
	}
	//Zip it up
	if(1)
	{
		char tempstr[1024];
		sprintf(tempstr, "cd %s && zip -r %s.zip %s/*",  snapshot_config.destination_path.c_str(),snapshot_path.c_str(),snapshot_name.c_str());
		active_snapshot_completepath = snapshot_config.destination_path + snapshot_name + ".zip";
		exec(tempstr, true); 
	}
	//Make sure it's actually there
	{
		int file_found = count_files_indirectory(snapshot_config.destination_path,snapshot_name+".zip");
		if(file_found != 1)
		{
			char tempstr[1024];
			sprintf(tempstr,"Device Snapshot not created: %s",snapshot_name.c_str());
			diag = update_diagnostic(DATA_STORAGE,ERROR,DROPPING_PACKETS,std::string(tempstr));
			diaglist.push_back(diag);
			return diaglist;
		}
	}
	//Clean up folder
	if(1)
	{
		char tempstr[1024];
		sprintf(tempstr, "rm -r -f %s",  snapshot_path.c_str());
		exec(tempstr, true); 
	}
	char tempstr[512];
	sprintf(tempstr,"Device Snapshot Created at: %s",snapshot_path.c_str());
	diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,std::string(tempstr));
	diaglist.push_back(diag);
	devicesnapshot_state = SnapshotState::READY;
	if(getInstanceMode() == InstanceMode::MASTER)
	{
		run_systemsnapshot_timeout_timer = true;
		time_t rawtime2;
		struct tm * timeinfo2;
		char buffer2[80];
		time (&rawtime2);
		timeinfo2 = localtime(&rawtime2);

		strftime(buffer2,sizeof(buffer2),"SystemSnapshot_%Y_%m_%d_%H_%M_%S",timeinfo2);
		std::string str(buffer2);
		systemsnapshot_name = str;
		systemsnapshot_path = systemsnapshot_directory + systemsnapshot_name;
		if (mkdir(systemsnapshot_path.c_str(), 0777) == -1) 
		{
			char tempstr2[512];
			sprintf(tempstr2,"Unable to create System Snapshot folder: %s",systemsnapshot_path.c_str());
			diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr2));
			diaglist.push_back(diag);
			return diaglist;
		}
		//Move Device Snap to System Snap Directory
		{
			char tempstr[1024];
			sprintf(tempstr, "mv %s %s",  active_snapshot_completepath.c_str(),systemsnapshot_path.c_str());
			exec(tempstr, true); 
		}
		systemsnapshot_info.devices.push_back(mydevice.DeviceName);
		double v = 1.0/(double)((uint32_t)all_snapshot_devices.size()+1);
		eros_systemsnapshot_state.percent_complete+= (uint8_t)(100.0*v);

		devicesnapshot_state = SnapshotState::NOTRUNNING;
		systemsnapshot_state = SnapshotState::RUNNING;
	}
	diaglist.push_back(diag);
	return diaglist;
}
std::string SnapshotNodeProcess::map_snapshotmode_tostring(uint8_t t_mode)
{
	switch(t_mode)
	{
		case ROVERCOMMAND_SNAPSHOTMODE_UNDEFINED:
			return "UNKNOWN";
			break;
		case ROVERCOMMAND_SNAPSHOTMODE_MANUAL:
			return "MANUAL";
			break;
		case ROVERCOMMAND_SNAPSHOTMODE_AUTO:
			return "AUTO";
			break;
		default:
			return "UNKNOWN";
			break;
	}
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
		case SnapshotState::COMPLETE:
			return "COMPLETE";
			break;
		case SnapshotState::INCOMPLETE:
			return "INCOMPLETE";
			break;
		default:
			return "UNKNOWN";
			break;
	}
}
int SnapshotNodeProcess::count_files_indirectory(std::string directory,std::string filter)
{
	try
	{
	
		char tempstr[1024];
		sprintf(tempstr,"ls %s%s 2>/dev/null | wc -l",directory.c_str(),filter.c_str());
		std::string return_v = exec(tempstr,true);
		boost::trim_right(return_v);
		return std::atoi(return_v.c_str());
	}
	catch(const std::exception& e)
	{
		return 0;
	}
}
std::string SnapshotNodeProcess::generate_systemsnapshotinfo(SystemSnapshotInfo info)
{
	std::string tempstr;
	tempstr = "Snapshot Start: " + std::to_string(info.rostime_start) + "\r\n";
	tempstr += "Start DateTime: " + convert_time_tostr(info.rostime_start) + "\r\n";
	tempstr += "Snapshot Stop: " + std::to_string(info.rostime_stop) + "\r\n";
	tempstr += "Stop DateTime: " + convert_time_tostr(info.rostime_stop) + "\r\n";
	tempstr += "Snapshot State: " + map_state_tostring(info.state) + "\r\n";
	tempstr += "Mode: " + map_snapshotmode_tostring(info.snapshot_mode) + "\r\n";
	tempstr += "Command Text: " + info.generate_commandtext + "\r\n";
	tempstr += "Command Description: " + info.generate_commanddescription + "\r\n";
	tempstr += "Snapshot Contains Device Snapshot from the following Devices: \r\n";
	for(std::size_t i = 0; i < info.devices.size(); ++i)
	{
		tempstr += "\t[" + std::to_string(i) + "] Device: " + info.devices.at(i) + "\r\n";
	}
	return tempstr;
}