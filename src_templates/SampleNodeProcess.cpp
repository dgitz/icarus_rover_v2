#include "SampleNodeProcess.h"
eros::diagnostic SampleNodeProcess::set_config_filepaths(std::string t_filepath)
{
	eros::diagnostic diag = diagnostic;
	config_filepath = t_filepath;
	return diag;
}
eros::diagnostic SampleNodeProcess::load_configfile(std::string path)
{
	eros::diagnostic diag = diagnostic;
	TiXmlDocument doc(path);
	bool configfile_loaded = doc.LoadFile();
	if(configfile_loaded == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = FATAL;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		char tempstr[512];
		sprintf(tempstr,"Unable to load: %s",path.c_str());
		diag.Description = std::string(tempstr);
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
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Not all items loaded.";
		return diag;
	}
	else
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = INFO;
		diag.Diagnostic_Message = INITIALIZING;
		diag.Description = "Config Loaded.";
		return diag;
	}


}
eros::diagnostic  SampleNodeProcess::finish_initialization()
{
    eros::diagnostic diag = diagnostic;
	diagnostic = load_configfile(config_filepath);
    return diagnostic;
}
eros::diagnostic SampleNodeProcess::update(double t_dt,double t_ros_time)
{
	if(initialized == true)
	{
		ready = true;

	}
	eros::diagnostic diag = diagnostic;
	diag = update_baseprocess(t_dt,t_ros_time);
	if(diag.Level <= NOTICE)
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Node Running.";

	}
	diagnostic = diag;
	return diag;
}
eros::diagnostic SampleNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = diagnostic;
	printf("Got Device: %s\n",device->DeviceName.c_str());
	return diag;
}
std::vector<eros::diagnostic> SampleNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = diagnostic;
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
	return diaglist;
}
std::vector<eros::diagnostic> SampleNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = diagnostic;
	bool status = true;

	if (status == true) {
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED.";
		diaglist.push_back(diag);
	} else {
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED.";
		diaglist.push_back(diag);
	}
	return diaglist;
}
