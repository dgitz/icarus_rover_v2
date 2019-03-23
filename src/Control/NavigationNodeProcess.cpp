#include "NavigationNodeProcess.h"
eros::diagnostic  NavigationNodeProcess::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	controlgroup_filepath = "";

	controlgroup_mode_map[ControlGroupMode::UNKNOWN] = "UNKNOWN";
	controlgroup_mode_map[ControlGroupMode::ARCADE] = "Arcade";

	controlgroup_inputtype_map[ControlGroupInputType::UNKNOWN] = "UNKNOWN";
	controlgroup_inputtype_map[ControlGroupInputType::THROTTLE] = "Throttle";
	controlgroup_inputtype_map[ControlGroupInputType::STEER] = "Steer";

	controlgroup_outputtype_map[ControlGroupOutputType::UNKNOWN] = "UNKNOWN";
	controlgroup_outputtype_map[ControlGroupOutputType::LEFTDRIVE] = "LeftDrive";
	controlgroup_outputtype_map[ControlGroupOutputType::RIGHTDRIVE] = "RightDrive";
	return diagnostic;
}
eros::diagnostic NavigationNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = diagnostic;
	diag = update_baseprocess(t_dt,t_ros_time);
	if(diag.Level <= NOTICE)
	{
		diag.Diagnostic_Type = NOERROR;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Node Running.";

	}
	diagnostic = diag;
	return diag;
}
eros::diagnostic NavigationNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = diagnostic;
	for(std::size_t i = 0; i < device->pins.size(); ++i)
	{
		all_pins.push_back(device->pins.at(i));
	}
	return diag;
}
std::vector<eros::diagnostic> NavigationNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
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
	else if(t_msg->Command == ROVERCOMMAND_DRIVECOMMAND)
	{
		json obj = json::parse(t_msg->CommandText);
		diag = update_controlgroups(obj);
		diaglist.push_back(diag);
	}
	return diaglist;
}
eros::diagnostic NavigationNodeProcess::update_controlgroups(json cmd)
{
	eros::diagnostic diag = diagnostic;
	double throttle = 0.0;
	double steer = 0.0;
	std::string controlgroup = "";
	for (json::iterator it = cmd.begin(); it != cmd.end(); ++it)
	{
		if(it.key() == "ControlGroup")
		{
			controlgroup = it.value();
		}
		if(it.key() == "ForwardVelocityPerc")
		{
			throttle = it.value();
		}
		if(it.key() == "RotateZAxisPerc")
		{
			steer = it.value();
		}
	}
	bool found = false;
	for(std::size_t i = 0; i < control_groups.size(); ++i)
	{
		if(control_groups.at(i).name == controlgroup)
		{
			found = true;
		}
	}
	if(found == false)
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = NOTICE;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[512];
		sprintf(tempstr,"ControlGroup: %s Not Updated.",controlgroup.c_str());
		diag.Description = std::string(tempstr);
	}

	diag.Diagnostic_Type = SOFTWARE;
	diag.Level = WARN;
	diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
	diag.Description = "Working on implementation.";
	return diag;
}
std::vector<eros::diagnostic> NavigationNodeProcess::check_programvariables()
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
eros::diagnostic NavigationNodeProcess::load_controlgroupfile()
{
	eros::diagnostic diag = diagnostic;
	TiXmlDocument doc(controlgroup_filepath);
	bool controlgroupfile_loaded = doc.LoadFile();
	if(controlgroupfile_loaded == false)
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = FATAL;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		char tempstr[512];
		sprintf(tempstr,"Unable to load: %s",controlgroup_filepath.c_str());
		diag.Description = std::string(tempstr);
		return diag;
	}
	TiXmlElement *l_pRootElement = doc.RootElement();

	if( NULL != l_pRootElement )
	{
		TiXmlElement *l_pControlGroupList = l_pRootElement->FirstChildElement( "ControlGroupList" );
		if(NULL != l_pControlGroupList)
		{
			TiXmlElement *l_pControlGroup = l_pControlGroupList->FirstChildElement( "ControlGroup" );
			if(NULL != l_pControlGroup)
			{
				while( l_pControlGroup )
				{
					ControlGroup cg;
					TiXmlElement *l_pControlGroupName = l_pControlGroup->FirstChildElement( "Name" );
					if ( NULL != l_pControlGroupName )
					{
						cg.name = l_pControlGroupName->GetText();
					}
					TiXmlElement *l_pControlGroupMode = l_pControlGroup->FirstChildElement( "Mode" );
					if ( NULL != l_pControlGroupMode )
					{
						cg.mode = map_ControlGroupMode_ToEnum(l_pControlGroupMode->GetText());
					}
					TiXmlElement *l_pControlGroupInput = l_pControlGroup->FirstChildElement( "Input" );
					if(NULL != l_pControlGroupInput)
					{
						while(l_pControlGroupInput)
						{
							ControlGroupInput input;
							TiXmlElement *l_pControlGroupInputName = l_pControlGroupInput->FirstChildElement( "Name" );
							if(NULL != l_pControlGroupInputName)
							{
								input.name = l_pControlGroupInputName->GetText();
							}
							TiXmlElement *l_pControlGroupInputType = l_pControlGroupInput->FirstChildElement( "Type" );
							if(NULL != l_pControlGroupInputType)
							{
								input.type = map_ControlGroupInputType_ToEnum(l_pControlGroupInputType->GetText());
							}
							cg.inputs.push_back(input);
							l_pControlGroupInput = l_pControlGroupInput->NextSiblingElement( "Input" );
						}
					}
					TiXmlElement *l_pControlGroupOutput = l_pControlGroup->FirstChildElement( "Output" );
					if(NULL != l_pControlGroupOutput)
					{
						while(l_pControlGroupOutput)
						{
							ControlGroupOutput output;
							TiXmlElement *l_pControlGroupOutputName = l_pControlGroupOutput->FirstChildElement( "Name" );
							if(NULL != l_pControlGroupOutputName)
							{
								output.name = l_pControlGroupOutputName->GetText();
							}
							TiXmlElement *l_pControlGroupOutputType = l_pControlGroupOutput->FirstChildElement( "Type" );
							if(NULL != l_pControlGroupOutputType)
							{
								output.type = map_ControlGroupOutputType_ToEnum(l_pControlGroupOutputType->GetText());
							}
							bool found = false;
							for(std::size_t k = 0; k < all_pins.size(); ++k)
							{
								if(all_pins.at(k).ConnectedDevice == output.name)
								{
									found = true;
									output.pin = all_pins.at(k);
								}
							}
							if(found == false)
							{
								diag.Diagnostic_Type = DATA_STORAGE;
								diag.Level = ERROR;
								diag.Diagnostic_Message = INITIALIZING_ERROR;
								char tempstr[512];
								sprintf(tempstr,"Could not find Pin:%s",output.name.c_str());
								diag.Description = std::string(tempstr);
								return diag;
							}

							cg.outputs.push_back(output);
							l_pControlGroupOutput = l_pControlGroupOutput->NextSiblingElement( "Output" );
						}
					}
					control_groups.push_back(cg);
					l_pControlGroup = l_pControlGroup->NextSiblingElement( "ControlGroup" );
				}


			}
		}
	}

	diag.Diagnostic_Type = NOERROR;
	diag.Level = NOTICE;
	diag.Diagnostic_Message = INITIALIZING;
	diag.Description = "Processed ControlGroup.xml";
	return diag;
}
void NavigationNodeProcess::print_controlgroups()
{

	for(std::size_t i = 0; i < control_groups.size(); ++i)
	{
		ControlGroup cg = control_groups.at(i);
		printf("[%d] ControlGroup: %s Type:%s\n",(uint16_t)i,cg.name.c_str(),map_ControlGroupMode_ToString(cg.mode).c_str());
		for(std::size_t j = 0; j < cg.inputs.size(); ++j)
		{
			printf("\t[%d] Input: %s Type:%s\n",(uint16_t)j,cg.inputs.at(j).name.c_str(),map_ControlGroupInputType_ToString(cg.inputs.at(j).type).c_str());
		}
		for(std::size_t j = 0; j < cg.outputs.size(); ++j)
		{
			printf("\t[%d] Output: %s Type:%s\n",(uint16_t)j,cg.outputs.at(j).name.c_str(),map_ControlGroupOutputType_ToString(cg.outputs.at(j).type).c_str());
			eros::pin pin = cg.outputs.at(j).pin;

			printf("\t\tPin Number: %d Value: %d Max Value: %d Min Value: %d Default Value: %d\n",
					pin.Number,pin.Value,pin.MaxValue,pin.MinValue,pin.DefaultValue);
		}
	}

}
std::string NavigationNodeProcess::map_ControlGroupMode_ToString(ControlGroupMode v)
{
	std::map<ControlGroupMode,std::string>::iterator it;
	it = controlgroup_mode_map.find(v);
	if (it != controlgroup_mode_map.end())
	{
		return it->second;
	}
	return "UNKNOWN";
}
NavigationNodeProcess::ControlGroupMode NavigationNodeProcess::map_ControlGroupMode_ToEnum(std::string v)
{
	std::map<std::string,ControlGroupMode> reverse_map;
	std::map<std::string,ControlGroupMode>::iterator it = reverse_map.begin();

	for (auto& x: controlgroup_mode_map)
	{
		reverse_map.insert (it, std::pair<std::string,ControlGroupMode>(x.second,x.first));
	}
	it = reverse_map.find(v);
	if (it != reverse_map.end())
	{
		return it->second;
	}
	return ControlGroupMode::UNKNOWN;
}
std::string NavigationNodeProcess::map_ControlGroupInputType_ToString(ControlGroupInputType v)
{

	std::map<ControlGroupInputType,std::string>::iterator it;
	it = controlgroup_inputtype_map.find(v);
	if (it != controlgroup_inputtype_map.end())
	{
		return it->second;
	}

	return "UNKNOWN";
}
NavigationNodeProcess::ControlGroupInputType NavigationNodeProcess::map_ControlGroupInputType_ToEnum(std::string v)
{

	std::map<std::string,ControlGroupInputType> reverse_map;
	std::map<std::string,ControlGroupInputType>::iterator it = reverse_map.begin();

	for (auto& x: controlgroup_inputtype_map)
	{
		reverse_map.insert (it, std::pair<std::string,ControlGroupInputType>(x.second,x.first));
	}
	it = reverse_map.find(v);
	if (it != reverse_map.end())
	{
		return it->second;
	}

	return ControlGroupInputType::UNKNOWN;
}
std::string NavigationNodeProcess::map_ControlGroupOutputType_ToString(ControlGroupOutputType v)
{

	std::map<ControlGroupOutputType,std::string>::iterator it;
	it = controlgroup_outputtype_map.find(v);
	if (it != controlgroup_outputtype_map.end())
	{
		return it->second;
	}

	return "UNKNOWN";
}
NavigationNodeProcess::ControlGroupOutputType NavigationNodeProcess::map_ControlGroupOutputType_ToEnum(std::string v)
{

	std::map<std::string,ControlGroupOutputType> reverse_map;
	std::map<std::string,ControlGroupOutputType>::iterator it = reverse_map.begin();

	for (auto& x: controlgroup_outputtype_map)
	{
		reverse_map.insert (it, std::pair<std::string,ControlGroupOutputType>(x.second,x.first));
	}
	it = reverse_map.find(v);
	if (it != reverse_map.end())
	{
		return it->second;
	}

	return ControlGroupOutputType::UNKNOWN;
}
NavigationNodeProcess::DrivePerc NavigationNodeProcess::arcade_mix(double throttle_perc,double steer_perc)
{
	DrivePerc d;
	double v =(100.0-fabs(steer_perc)) * (throttle_perc/100.0) + throttle_perc;
	double w= (100.0-fabs(throttle_perc)) * (steer_perc/100.0) + steer_perc;
	d.left = (v+w)/2.0;
	d.right = (v-w)/2.0;
	return d;
}
