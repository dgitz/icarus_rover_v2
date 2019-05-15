#include "SampleNodeProcess.h"
eros::diagnostic SampleNodeProcess::set_config_filepaths(std::string t_filepath)
{
	eros::diagnostic diag = root_diagnostic;
	config_filepath = t_filepath;
	diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,"Config File Paths Set");
	return diag;
}
eros::diagnostic SampleNodeProcess::load_configfile(std::string path)
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
eros::diagnostic  SampleNodeProcess::finish_initialization()
{
    eros::diagnostic diag = root_diagnostic;
	diag = load_configfile(config_filepath);
	diag = update_diagnostic(diag);
    return diag;
}
eros::diagnostic SampleNodeProcess::update(double t_dt,double t_ros_time)
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
eros::diagnostic SampleNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Updated Device");
	return diag;
}
std::vector<eros::diagnostic> SampleNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
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
std::vector<eros::diagnostic> SampleNodeProcess::check_programvariables()
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
