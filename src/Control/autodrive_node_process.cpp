#include "autodrive_node_process.h"
AutoDriveNodeProcess::AutoDriveNodeProcess()
{
	controlgroups.clear();
	initialized = false;
}
AutoDriveNodeProcess::~AutoDriveNodeProcess()
{

}
icarus_rover_v2::diagnostic AutoDriveNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
    diagnostic = indiag;
	if(read_ControlGroupFile() == false)
	{
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
		diagnostic.Description = "Unable to read ControlGroup file.";
		initialized = false;
		return diagnostic;
	}
	else
	{
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Description = "Process initialized.";
		return diagnostic;
	}
}
icarus_rover_v2::diagnostic AutoDriveNodeProcess::update(double dt)
{
	bool all_pininfo_received = true;
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		if(controlgroups.at(i).output.pin_info_received == false) { all_pininfo_received = false; }
	}
	if(controlgroups.size() == 0) { all_pininfo_received = false; }
	if(all_pininfo_received == true)
	{
		initialized = true;
	}
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		if(armed_state != ARMEDSTATUS_ARMED)
		{
			controlgroups.at(i).output.pin.Value = controlgroups.at(i).output.pin.DefaultValue;
			controlgroups.at(i).cum_error = 0.0;
		}
		else
		{
			if(controlgroups.at(i).gain.type == "PID")
			{
				double error = controlgroups.at(i).command.input-controlgroups.at(i).sensor.input;
				
				controlgroups.at(i).cum_error += error;
				double P_term = controlgroups.at(i).gain.P*error;
				double I_term = controlgroups.at(i).gain.I*controlgroups.at(i).cum_error;
				double D_term = controlgroups.at(i).gain.D*(error-controlgroups.at(i).current_error);
				controlgroups.at(i).current_error = error;
				controlgroups.at(i).output.pin.Value = (int32_t)P_term+(int32_t)I_term+(int32_t)D_term+controlgroups.at(i).output.pin.DefaultValue;
			}
			if(controlgroups.at(i).output.pin.Value > controlgroups.at(i).output.pin.MaxValue)
			{
				controlgroups.at(i).output.pin.Value = controlgroups.at(i).output.pin.MaxValue;
			}
			else if(controlgroups.at(i).output.pin.Value < controlgroups.at(i).output.pin.MinValue)
			{
				controlgroups.at(i).output.pin.Value = controlgroups.at(i).output.pin.MinValue;
			}
		}
	}
	diagnostic.Diagnostic_Type = SOFTWARE;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Process Updated.";
	return diagnostic;
}
icarus_rover_v2::diagnostic AutoDriveNodeProcess::new_commandmsg(std::string cg_name, double v)
{
	bool found = false;
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		if(controlgroups.at(i).name == cg_name)
		{
			controlgroups.at(i).command.input = v;
			found = true;
		}
	}
	if(found == true)
	{
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Description = "Processed Command message";
		return diagnostic;
	}
	else
	{
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Level = WARN;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		char tempstr[128];
		sprintf(tempstr,"Control Group: %s not found for Command message.",cg_name.c_str());
		diagnostic.Description = std::string(tempstr);
		return diagnostic;
	}
}
icarus_rover_v2::diagnostic AutoDriveNodeProcess::new_sensormsg(std::string cg_name, double v)
{
	bool found = false;
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		if(controlgroups.at(i).name == cg_name)
		{
			controlgroups.at(i).sensor.input = v;
			found = true;
		}
	}
	if(found == true)
	{
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Description = "Processed Sensor message";
		return diagnostic;
	}
	else
	{
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Level = WARN;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		char tempstr[128];
		sprintf(tempstr,"Control Group: %s not found for Sensor message.",cg_name.c_str());
		diagnostic.Description = std::string(tempstr);
		return diagnostic;
	}
	
}
std::vector<icarus_rover_v2::pin> AutoDriveNodeProcess::get_controlgroup_pins(std::string topic)
{
	std::vector<icarus_rover_v2::pin> pins;
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		if(topic == controlgroups.at(i).output.topic)
		{
			pins.push_back(controlgroups.at(i).output.pin);
		}
	}
	return pins;
}
icarus_rover_v2::diagnostic AutoDriveNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	for(std::size_t i = 0; i < newdevice.pins.size(); i++)
	{
		for(std::size_t j = 0; j < controlgroups.size(); j++)
		{
			if((controlgroups.at(i).output.pin_info_received == false) && (controlgroups.at(j).output.name == newdevice.pins.at(i).Name))
			{
				controlgroups.at(j).output.topic = "/" + newdevice.DeviceName + "/" + newdevice.pins.at(i).Function;
				controlgroups.at(j).output.pin = newdevice.pins.at(i);
				controlgroups.at(j).output.pin_info_received = true;
			}
		}
	}
	diagnostic.Diagnostic_Type = SOFTWARE;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Device Message Processed.";
	return diagnostic;
}
bool AutoDriveNodeProcess::read_ControlGroupFile()
{
	TiXmlDocument controlgroup_doc("/home/robot/config/ControlGroup.xml");
	bool controlgroupfile_loaded = controlgroup_doc.LoadFile();
    if(controlgroupfile_loaded == false) { 	return false;  }
	TiXmlElement *l_pRootElement = controlgroup_doc.RootElement();

	if( NULL != l_pRootElement )
	{
	    TiXmlElement *l_pControlGroupList = l_pRootElement->FirstChildElement( "ControlGroupList" );
		if ( NULL != l_pControlGroupList )
	    {
	        TiXmlElement *l_pControlGroup = l_pControlGroupList->FirstChildElement( "ControlGroup" );

	        while( l_pControlGroup )
	        {
				ControlGroup new_cg;
				
	        	TiXmlElement *l_pCGName = l_pControlGroup->FirstChildElement( "Name" );
				if ( NULL != l_pCGName )
				{
					new_cg.name = l_pCGName->GetText();
				}
				
				TiXmlElement *l_pCGCommand = l_pControlGroup->FirstChildElement("Command");
				if ( NULL != l_pCGCommand )
				{
					TiXmlElement *l_pCGCommandType = l_pCGCommand->FirstChildElement("Type");
					if ( NULL != l_pCGCommandType )
					{
						new_cg.command.type = l_pCGCommandType->GetText();
					}
					
					TiXmlElement *l_pCGCommandTopic = l_pCGCommand->FirstChildElement("Topic");
					if ( NULL != l_pCGCommandTopic )
					{
						new_cg.command.topic = l_pCGCommandTopic->GetText();
					}
					
					TiXmlElement *l_pCGCommandName = l_pCGCommand->FirstChildElement("Name");
					if ( NULL != l_pCGCommandName )
					{
						new_cg.command.name = l_pCGCommandName->GetText();
					}
					
					TiXmlElement *l_pCGCommandIndex = l_pCGCommand->FirstChildElement("Index");
					if ( NULL != l_pCGCommandIndex )
					{
						new_cg.command.index = atoi(l_pCGCommandIndex->GetText());
					}
					
					TiXmlElement *l_pCGCommandMinValue = l_pCGCommand->FirstChildElement("MinValue");
					if ( NULL != l_pCGCommandMinValue )
					{
						new_cg.command.min_value = atof(l_pCGCommandMinValue->GetText());
					}
					
					TiXmlElement *l_pCGCommandMaxValue = l_pCGCommand->FirstChildElement("MaxValue");
					if ( NULL != l_pCGCommandMaxValue )
					{
						new_cg.command.max_value = atof(l_pCGCommandMaxValue->GetText());
					}
				}
				
				TiXmlElement *l_pCGSensor = l_pControlGroup->FirstChildElement("Sensor");
				if ( NULL != l_pCGSensor )
				{
					TiXmlElement *l_pCGSensorType = l_pCGSensor->FirstChildElement("Type");
					if ( NULL != l_pCGSensorType )
					{
						new_cg.sensor.type = l_pCGSensorType->GetText();
					}
					
					TiXmlElement *l_pCGSensorName = l_pCGSensor->FirstChildElement("Name");
					if ( NULL != l_pCGSensorName )
					{
						int sensor = get_sensor(l_pCGSensorName->GetText());
						if(sensor == SENSOR_UNDEFINED) { return false; }
						new_cg.sensor.name = sensor;
						
					}
				}
				
				TiXmlElement *l_pCGOutput = l_pControlGroup->FirstChildElement("Output");
				if ( NULL != l_pCGOutput )
				{
					
					TiXmlElement *l_pCGOutputName = l_pCGOutput->FirstChildElement("Name");
					if ( NULL != l_pCGOutputName )
					{
						new_cg.output.name = l_pCGOutputName->GetText();
					}
					
					
				}
				
				TiXmlElement *l_pCGGain = l_pControlGroup->FirstChildElement("Gain");
				if ( NULL != l_pCGGain )
				{
					TiXmlElement *l_pCGGainType = l_pCGGain->FirstChildElement("Type");
					if ( NULL != l_pCGGainType )
					{
						new_cg.gain.type = l_pCGGainType->GetText();
					}
					
					TiXmlElement *l_pCGGainP = l_pCGGain->FirstChildElement("Proportional");
					if ( NULL != l_pCGGainP )
					{
						new_cg.gain.P = atof(l_pCGGainP->GetText());
					}
					
					TiXmlElement *l_pCGGainI = l_pCGGain->FirstChildElement("Integral");
					if ( NULL != l_pCGGainI )
					{
						new_cg.gain.I = atof(l_pCGGainI->GetText());
					}
					
					TiXmlElement *l_pCGGainD = l_pCGGain->FirstChildElement("Derivative");
					if ( NULL != l_pCGGainD )
					{
						new_cg.gain.D = atof(l_pCGGainD->GetText());
					}
				}
		
				new_cg.sensor.input = 0.0;
				new_cg.current_error = 0.0;
				new_cg.cum_error = 0.0;
				new_cg.output.pin_info_received = false;
				controlgroups.push_back(new_cg);
				l_pControlGroup = l_pControlGroup->NextSiblingElement( "ControlGroup" );
			}
		}
	}
	return true;
}
icarus_rover_v2::diagnostic AutoDriveNodeProcess::new_armedstatemsg(uint8_t msg)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	armed_state = msg;
	diag.Level = INFO;
	diag.Diagnostic_Type = SOFTWARE;
	diag.Diagnostic_Message = NOERROR;
	char tempstr[512];
	sprintf(tempstr,"Rover Armed State: %d Processed.",msg);
	diag.Description = std::string(tempstr);
	return diag;
}
icarus_rover_v2::diagnostic AutoDriveNodeProcess::new_tunecontrolgroupmsg(icarus_rover_v2::controlgroup msg)
{
	bool found = false;
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		if((msg.name == controlgroups.at(i).name) and (msg.type == controlgroups.at(i).gain.type))
		{
			controlgroups.at(i).gain.P = msg.value1;
			controlgroups.at(i).gain.I = msg.value2;
			controlgroups.at(i).gain.D = msg.value3;
			controlgroups.at(i).output.pin.MaxValue = msg.maxvalue;
			controlgroups.at(i).output.pin.MinValue = msg.minvalue;
			found = true;
		}
	}
	if(found == true)
	{
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Description = "Processed Control Group Tune message";
		return diagnostic;
	}
	else
	{
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Level = WARN;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		char tempstr[128];
		sprintf(tempstr,"Contrl Group: %s type: %s not found.",msg.name.c_str(),msg.type.c_str());
		diagnostic.Description = std::string(tempstr);
		return diagnostic;
	}
}
uint8_t AutoDriveNodeProcess::get_sensor(std::string v)
{
	if(v == "/pose.yawrate")
	{
		return SENSOR_POSEYAWRATE;
	}
	else
	{
		return SENSOR_UNDEFINED;
	}
}
std::string AutoDriveNodeProcess::get_sensorname(uint8_t v)
{
	switch(v)
	{
		case SENSOR_UNDEFINED: return "UNDEFINED"; break;
		case SENSOR_POSEYAWRATE: return "/pose.yawrate"; break;
		default: return "UNDEFINED"; break;
	}
}
