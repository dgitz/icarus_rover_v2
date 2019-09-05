#include "DatabaseNodeProcess.h"
eros::diagnostic DatabaseNodeProcess::set_config_filepaths(std::string t_filepath)
{
	eros::diagnostic diag = root_diagnostic;
	config_filepath = t_filepath;
	diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,"Config File Paths Set");
	return diag;
}
eros::diagnostic DatabaseNodeProcess::load_configfile(std::string path)
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
	if( NULL != l_pRootElement )
	{
		TiXmlElement *l_pConfigList = l_pRootElement->FirstChildElement( "ConfigList" );
		if(NULL != l_pConfigList)
		{
			all_items_loaded = true;
		}
	}
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
eros::diagnostic  DatabaseNodeProcess::finish_initialization()
{
    eros::diagnostic diag = root_diagnostic;
	//sample_map[(uint8_t)SampleEnum::UNKNOWN] = "UNKNOWN";
	//sample_map[(uint8_t)SampleEnum::ENUMTYPEA] = "ENUM TYPE A";
	//sample_map[(uint8_t)SampleEnum::ENUMTYPEB] = "ENUM TYPE B";
	diag = load_configfile(config_filepath);
	diag = update_diagnostic(diag);
    return diag;
}
eros::diagnostic DatabaseNodeProcess::update(double t_dt,double t_ros_time)
{
	if(initialized == true)
	{
		ready = true;

	}
	eros::diagnostic diag = update_baseprocess(t_dt,t_ros_time);
	if(diag.Level <= NOTICE)
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error.");
		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running");
		
	}
	return diag;
}
eros::diagnostic DatabaseNodeProcess::new_devicemsg(__attribute__((unused)) const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Updated Device");
	return diag;
}
std::vector<eros::diagnostic> DatabaseNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
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
std::vector<eros::diagnostic> DatabaseNodeProcess::check_programvariables()
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
uint8_t DatabaseNodeProcess::map_sampledatatype_ToInt(std::string data)
{
	std::map<std::string,uint8_t> reverse_map;
	std::map<std::string,uint8_t>::iterator it = reverse_map.begin();

	for (auto& x: sample_map)
	{
		reverse_map.insert (it, std::pair<std::string,uint8_t>(x.second,x.first));
	}
	it = reverse_map.find(data);
	if (it != reverse_map.end())
	{
		return it->second;
	}
	return (uint8_t)SampleEnum::UNKNOWN;
}
*/
/*
std::string DatabaseNodeProcess::map_sampledatatype_ToString(uint8_t data)
{
	std::map<uint8_t,std::string>::iterator it;
	it = sample_map.find(data);
	if (it != sample_map.end())
	{
		return it->second;
	}
	return "UNDEFINED";
}
*/
/*
bool DatabaseNodeProcess::process_jsonmsg(json msg)
{
	for (json::iterator it = msg.begin(); it != msg.end(); ++it)
	{
		if(it.key() == "Key1")
		{
			return true;
		}
	}
	return false;
}
*/