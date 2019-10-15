#include "ControlGroupNodeProcess.h"
eros::diagnostic ControlGroupNodeProcess::set_config_filepaths(std::string t_filepath)
{
	eros::diagnostic diag = root_diagnostic;
	config_filepath = t_filepath;
	diag = update_diagnostic(DATA_STORAGE, INFO, INITIALIZING, "Config File Paths Set");
	return diag;
}
ControlGroup::Mode ControlGroupNodeProcess::map_controlgroupmode_toenum(std::string v)
{
	if (v == "PID")
	{
		return ControlGroup::Mode::PID;
	}
	else if(v == "Direct")
	{
		return ControlGroup::Mode::DIRECT;
	}
	else
	{
		return ControlGroup::Mode::UNKNOWN;
	}
}
ControlGroup::SignalClass ControlGroupNodeProcess::map_signalclass_toenum(std::string v)
{
	if ((v == "Signal") || (v == "SIGNAL"))
	{
		return ControlGroup::SignalClass::SIGNAL;
	}
	else if ((v == "Pin") || (v == "PIN"))
	{
		return ControlGroup::SignalClass::PIN;
	}
	else
	{
		return ControlGroup::SignalClass::UNKNOWN;
	}
}
eros::diagnostic ControlGroupNodeProcess::new_inputsignalmsg(const eros::signal::ConstPtr &t_msg)
{
	eros::signal sig = convert_fromptr(t_msg);
	eros::diagnostic diag = root_diagnostic;
	for (std::size_t i = 0; i < controlgroups.size(); ++i)
	{
		std::vector<std::string> input_names = controlgroups.at(i).get_inputsignalnames();
		for (std::size_t j = 0; j < input_names.size(); ++j)
		{
			if (input_names.at(j) == sig.name)
			{
				diag = controlgroups.at(i).new_input(sig);
				if (diag.Level > NOTICE)
				{
					return diag;
				}
			}
		}
	}
	return diag;
}
eros::diagnostic ControlGroupNodeProcess::set_PIDGains(std::string controlgroup_name,double P,double I,double D)
{
	eros::diagnostic diag = root_diagnostic;
	bool found = false;
	for(std::size_t i = 0; i < controlgroups.size(); ++i)
	{
		if(controlgroups.at(i).get_name() == controlgroup_name)
		{
			found = true;
			controlgroups.at(i).set_PIDGains(P,I,D);
		}
	}
	if(found == false)
	{
		diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, "Control Group:" + controlgroup_name + " Not Found.");
	}
	return diag;
}
eros::diagnostic ControlGroupNodeProcess::load_configfile(std::string path)
{
	eros::diagnostic diag = root_diagnostic;
	TiXmlDocument doc(path);
	bool configfile_loaded = doc.LoadFile();
	if (configfile_loaded == false)
	{
		char tempstr[512];
		sprintf(tempstr, "Unable to load: %s", path.c_str());
		diag = update_diagnostic(DATA_STORAGE, FATAL, INITIALIZING_ERROR, std::string(tempstr));
		return diag;
	}
	TiXmlElement *l_pRootElement = doc.RootElement();
	bool all_items_loaded = false;
	if (NULL != l_pRootElement)
	{
		TiXmlElement *l_pControlGroupList = l_pRootElement->FirstChildElement("ControlGroupList");
		if (NULL != l_pControlGroupList)
		{
			TiXmlElement *l_pControlGroup = l_pControlGroupList->FirstChildElement("ControlGroup");
			while (l_pControlGroup)
			{
				ControlGroup controlgroup;
				
				double gain_P, gain_I, gain_D = 0.0;
				std::string name;
				std::string mode;
				TiXmlElement *l_pCGName = l_pControlGroup->FirstChildElement("Name");
				if (l_pCGName != NULL)
				{
					name = l_pCGName->GetText();
				}
				else
				{
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, "ControlGroup Name Not Found.");
				}
				TiXmlElement *l_pCGMode = l_pControlGroup->FirstChildElement("Mode");
				if (l_pCGMode != NULL)
				{
					mode = l_pCGMode->GetText();
				}
				
				ControlGroup::Mode cg_mode = map_controlgroupmode_toenum(mode);
				diag = controlgroup.init(diag, cg_mode, name);
				if (diag.Level > NOTICE)
				{
					return diag;
				}
				TiXmlElement *l_pOptions = l_pControlGroup->FirstChildElement("Options");
				if (l_pOptions != NULL)
				{
					TiXmlElement *l_pPrintTuningInfo = l_pOptions->FirstChildElement("PrintTuningInfo");
					if (l_pPrintTuningInfo != NULL)
					{
						int print_tuning_info = std::atoi(l_pPrintTuningInfo->GetText());
						controlgroup.set_printtuninginfo((bool)print_tuning_info);
					}
					TiXmlElement *l_pPublishTuningInfo = l_pOptions->FirstChildElement("PublishTuningInfo");
					if (l_pPublishTuningInfo != NULL)
					{
						int publish_tuning_info = std::atoi(l_pPublishTuningInfo->GetText());
						controlgroup.set_publishtuninginfo((bool)publish_tuning_info);
					}
				}
				TiXmlElement *l_pLimits = l_pControlGroup->FirstChildElement("Limits");
				if(l_pLimits != NULL)
				{
					TiXmlElement *l_pOutputLimits = l_pLimits->FirstChildElement("Output");
					if(l_pOutputLimits != NULL)
					{
						double output_max = 0.0;
						double output_min = 0.0;
						double output_default = 0.0;
						double output_delta = 0.0;
						TiXmlElement *l_pOutputMax = l_pOutputLimits->FirstChildElement("Max");
						if(l_pOutputMax != NULL)
						{
							output_max = std::atof(l_pOutputMax->GetText());
						}
						TiXmlElement *l_pOutputMin = l_pOutputLimits->FirstChildElement("Min");
						if(l_pOutputMin != NULL)
						{
							output_min = std::atof(l_pOutputMin->GetText());
						}
						TiXmlElement *l_pOutputDefault = l_pOutputLimits->FirstChildElement("Default");
						if(l_pOutputDefault != NULL)
						{
							output_default = std::atof(l_pOutputDefault->GetText());
						}
						TiXmlElement *l_pOutputDelta = l_pOutputLimits->FirstChildElement("Delta");
						if(l_pOutputDelta != NULL)
						{
							output_delta = std::atof(l_pOutputDelta->GetText());
						}
						controlgroup.set_outputlimits(output_min,output_default,output_max);
						controlgroup.set_max_deltaoutput(output_delta);
					}
					TiXmlElement *l_pInputLimits = l_pLimits->FirstChildElement("Input");
					if(l_pInputLimits != NULL)
					{
						double input_min = 0.0;
						double input_default = 0.0;
						double input_max = 0.;
						TiXmlElement *l_InputMax = l_pInputLimits->FirstChildElement("Max");
						if(l_InputMax != NULL)
						{
							input_max = std::atof(l_InputMax->GetText());
						}
						TiXmlElement *l_pInputMin = l_pInputLimits->FirstChildElement("Min");
						if(l_pInputMin != NULL)
						{
							input_min = std::atof(l_pInputMin->GetText());
						}
						TiXmlElement *l_pInputDefault = l_pInputLimits->FirstChildElement("Default");
						if(l_pInputDefault != NULL)
						{
							input_default = std::atof(l_pInputDefault->GetText());
						}
						controlgroup.set_inputlimits(input_min,input_default,input_max);
					}
				}
				if (cg_mode == ControlGroup::Mode::PID)
				{

					bool all_gains_found = true;
					TiXmlElement *l_pGains = l_pControlGroup->FirstChildElement("Gains");
					if (l_pGains != NULL)
					{
						TiXmlElement *l_pP = l_pGains->FirstChildElement("P");
						if (l_pP != NULL)
						{
							gain_P = std::atof(l_pP->GetText());
						}
						else
						{
							all_gains_found = false;
						}
						TiXmlElement *l_pI = l_pGains->FirstChildElement("I");
						if (l_pI != NULL)
						{
							gain_I = std::atof(l_pI->GetText());
						}
						else
						{
							all_gains_found = false;
						}
						TiXmlElement *l_pD = l_pGains->FirstChildElement("D");
						if (l_pD != NULL)
						{
							gain_D = std::atof(l_pD->GetText());
						}
						else
						{
							all_gains_found = false;
						}
					}
					else
					{
						diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, "Gains Section Not Found.");
						return diag;
					}
					if (all_gains_found == false)
					{
						diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, "Not All Gains Found.");
						return diag;
					}
				}
				//Read Inputs, then Outputs
				TiXmlElement *l_pInput = l_pControlGroup->FirstChildElement("Input");
				while (l_pInput)
				{
					std::string signal_class;
					std::string signal_name;
					std::string signal_units;
					TiXmlElement *l_pSignalClass = l_pInput->FirstChildElement("Class");
					if (l_pSignalClass != NULL)
					{
						signal_class = l_pSignalClass->GetText();
					}
					TiXmlElement *l_pSignalName = l_pInput->FirstChildElement("Name");
					if (l_pSignalName != NULL)
					{
						signal_name = l_pSignalName->GetText();
					}
					ControlGroup::SignalClass signal_class_enum = map_signalclass_toenum(signal_class);
					if (signal_class_enum == ControlGroup::SignalClass::SIGNAL)
					{
						TiXmlElement *l_pSignalUnits = l_pInput->FirstChildElement("Units");
						if (l_pSignalUnits != NULL)
						{
							signal_units = l_pSignalUnits->GetText();
						}
						double conversion_factor;
						diag = controlgroup.initialize_signal(ControlGroup::SignalDirection::INPUT, signal_name, convert_signaltype(signal_units, &conversion_factor));
						if (diag.Level > NOTICE)
						{
							return diag;
						}
					}
					l_pInput = l_pInput->NextSiblingElement("Input");
				}
				TiXmlElement *l_pOutput = l_pControlGroup->FirstChildElement("Output");
				while (l_pOutput)
				{
					std::string signal_class;
					std::string connected_device;
					TiXmlElement *l_pSignalClass = l_pOutput->FirstChildElement("Class");
					if (l_pSignalClass != NULL)
					{
						signal_class = l_pSignalClass->GetText();
					}
					TiXmlElement *l_pConnectedDevice = l_pOutput->FirstChildElement("ConnectedDevice");
					if (l_pConnectedDevice != NULL)
					{
						connected_device = l_pConnectedDevice->GetText();
					}
					ControlGroup::SignalClass signal_class_enum = map_signalclass_toenum(signal_class);
					if (signal_class_enum == ControlGroup::SignalClass::PIN)
					{
						diag = controlgroup.initialize_pin(ControlGroup::SignalDirection::OUTPUT,
							connected_device);
						if (diag.Level > NOTICE)
						{
							return diag;
						}
					}
					l_pOutput = l_pOutput->NextSiblingElement("Output");
				}
				//Output Signals
				if (cg_mode == ControlGroup::Mode::PID)
				{
					controlgroup.set_PIDGains(gain_P, gain_I, gain_D);
				}
				diag = controlgroup.finish_initialization();
				if (diag.Level > NOTICE)
				{
					return diag;
				}
				controlgroups.push_back(controlgroup);
				l_pControlGroup = l_pControlGroup->NextSiblingElement("ControlGroup");
			}
			all_items_loaded = true;
		}
	}
	if (all_items_loaded == false)
	{
		diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, "Not all items loaded.");
		return diag;
	}
	else
	{
		diag = update_diagnostic(DATA_STORAGE, INFO, INITIALIZING, "Config Loaded.");
		return diag;
	}
}
eros::diagnostic ControlGroupNodeProcess::set_pinproperties(eros::pin pin)
{
	eros::diagnostic diag = root_diagnostic;
	for(std::size_t i = 0; i < controlgroups.size(); ++i)
	{
		 std::vector<eros::pin> outputs = controlgroups.at(i).get_outputpins();
		 for(std::size_t j = 0; j < outputs.size(); ++j)
		 {
			 if(outputs.at(j).ConnectedDevice == pin.ConnectedDevice)
			 {
				 diag = controlgroups.at(i).set_pinproperties(pin);
				 return diag;
			 }
		 }
	}
	diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Pin: " + pin.ConnectedDevice + " Not Found in Control Groups.");
	return diag;
}
eros::diagnostic ControlGroupNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	diag = load_configfile(config_filepath);
	diag = update_diagnostic(diag);
	return diag;
}
eros::diagnostic ControlGroupNodeProcess::update(double t_dt, double t_ros_time)
{
	if (initialized == true)
	{
		bool all_controlgroups_ready = true;
		if(controlgroups.size() == 0)
		{
			ready = true;
		}
		for(std::size_t i = 0; i < controlgroups.size(); ++i)
		{
			all_controlgroups_ready = all_controlgroups_ready && controlgroups.at(i).is_ready();
		}
		
		ready = all_controlgroups_ready;
		
	}
	eros::diagnostic diag = update_baseprocess(t_dt, t_ros_time);
	for(std::size_t i = 0; i < controlgroups.size(); ++i)
	{
		diag = controlgroups.at(i).update(t_dt);
		diag = update_diagnostic(diag);
		//print_diagnostic(diag);
	}
	bool controlgroups_ok = true;
	for(std::size_t i = 0; i < controlgroups.size(); ++i)
	{
		controlgroups_ok = controlgroups_ok && controlgroups.at(i).is_ready();
	}
	if(controlgroups_ok == true)
	{
		diag = update_diagnostic(ACTUATORS, INFO, NOERROR, "All Control Groups Ready.");
	}
	if (diag.Level <= NOTICE)
	{
		diag = update_diagnostic(DATA_STORAGE, INFO, NOERROR, "No Error.");
		diag = update_diagnostic(SOFTWARE, INFO, NOERROR, "Node Running");
	}
	
	return diag;
}
eros::diagnostic ControlGroupNodeProcess::new_devicemsg(const eros::device::ConstPtr &device)
{
	eros::diagnostic diag = update_diagnostic(SOFTWARE, INFO, NOERROR, "Updated Device");
	return diag;
}
std::vector<eros::diagnostic> ControlGroupNodeProcess::new_commandmsg(const eros::command::ConstPtr &t_msg)
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
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		diag = update_diagnostic(diaglist.at(i));
	}
	return diaglist;
}
std::vector<eros::diagnostic> ControlGroupNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true)
	{
		diag = update_diagnostic(SOFTWARE, INFO, DIAGNOSTIC_PASSED, "Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	}
	else
	{
		diag = update_diagnostic(SOFTWARE, WARN, DIAGNOSTIC_FAILED, "Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
std::vector<eros::view_controlgroup> ControlGroupNodeProcess::get_controlgroupviews()
{
	std::vector<eros::view_controlgroup> list;
	for(std::size_t i = 0; i < controlgroups.size(); ++i)
	{
		if(controlgroups.at(i).get_publishtuninginfo() == true)
		{
			eros::view_controlgroup cgview = controlgroups.at(i).get_controlgroupview();
			list.push_back(cgview);
		}
	}
	return list;
}