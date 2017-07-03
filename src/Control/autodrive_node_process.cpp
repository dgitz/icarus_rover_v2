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
	if(read_ControlGroupFile()) == false)
	{
		initialized = false;
	}
	else
	{
		initialized = true;
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic AutoDriveNodeProcess::update(double dt)
{
	return diagnostic;
}
bool AutoDriveNodeProcess::read_ControlGroupFile()
{
	TiXmlDocument controlgroup_doc("/home/robot/config/ControlGroup.xml");
	bool controlgroupfile_loaded = controlgroup_doc.LoadFile();
    if(controlgroupfile_loaded == false) { return false; }
	TiXmlElement *l_pRootElement = doc.RootElement();

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
						new_cg.Command.type = l_pCGCommandType->GetText();
					}
					
					TiXmlElement *l_pCGCommandTopic = l_pCGCommand->FirstChildElement("Topic");
					if ( NULL != l_pCGCommandTopic )
					{
						new_cg.Command.topic = l_pCGCommandTopic->GetText();
					}
					
					TiXmlElement *l_pCGCommandName = l_pCGCommand->FirstChildElement("Name");
					if ( NULL != l_pCGCommandName )
					{
						new_cg.Command.name = l_pCGCommandName->GetText();
					}
					
					TiXmlElement *l_pCGCommandIndex = l_pCGCommand->FirstChildElement("Index");
					if ( NULL != l_pCGCommandIndex )
					{
						new_cg.Command.index = atoi(l_pCGCommandIndex->GetText());
					}
					
					TiXmlElement *l_pCGCommandMinValue = l_pCGCommand->FirstChildElement("MinValue");
					if ( NULL != l_pCGCommandMinValue )
					{
						new_cg.Command.minvalue = atof(l_pCGCommandMinValue->GetText());
					}
					
					TiXmlElement *l_pCGCommandMaxValue = l_pCGCommand->FirstChildElement("MaxValue");
					if ( NULL != l_pCGCommandMaxValue )
					{
						new_cg.Command.maxvalue = atof(l_pCGCommandMaxValue->GetText());
					}
				}
				
				TiXmlElement *l_pCGSensor = l_pControlGroup->FirstChildElement("Sensor");
				if ( NULL != l_pCGSensor )
				{
					TiXmlElement *l_pCGSensorType = l_pCGSensor->FirstChildElement("Type");
					if ( NULL != l_pCGSensorType )
					{
						new_cg.Sensor.type = l_pCGSensorType->GetText();
					}
					
					TiXmlElement *l_pCGSensorTopic = l_pCGSensor->FirstChildElement("Topic");
					if ( NULL != l_pCGSensorTopic )
					{
						new_cg.Sensor.topic = l_pCGSensorTopic->GetText();
					}
					
					TiXmlElement *l_pCGSensorName = l_pCGSensor->FirstChildElement("Name");
					if ( NULL != l_pCGSensorName )
					{
						int sensor = get_sensor(l_pCGSensorName->GetText());
						if(sensor == SENSOR_UNDEFINED) { return false; }
						new_cg.Sensor.name = sensor;
						
					}
				}
				
				TiXmlElement *l_pCGOutput = l_pControlGroup->FirstChildElement("Output");
				if ( NULL != l_pCGOutput )
				{
					TiXmlElement *l_pCGOutputType = l_pCGOutput->FirstChildElement("Type");
					if ( NULL != l_pCGOutputType )
					{
						new_cg.Output.type = l_pCGOutputType->GetText();
					}
					
					TiXmlElement *l_pCGOutputTopic = l_pCGOutput->FirstChildElement("Topic");
					if ( NULL != l_pCGOutputTopic )
					{
						new_cg.Output.topic = l_pCGOutputTopic->GetText();
					}
					
					TiXmlElement *l_pCGOutputParent = l_pCGOutput->FirstChildElement("Parent");
					if ( NULL != l_pCGOutputParent )
					{
						new_cg.Output.parent = l_pCGOutputParent->GetText();
					}
					
					TiXmlElement *l_pCGOutputPin = l_pCGOutput->FirstChildElement("PinNumber");
					if ( NULL != l_pCGOutputPin )
					{
						new_cg.Output.pinnumber = atoi(l_pCGOutputPin->GetText());
					}
					
					TiXmlElement *l_pCGOutputFunction = l_pCGOutput->FirstChildElement("Function");
					if ( NULL != l_pCGOutputFunction )
					{
						new_cg.Output.function = l_pCGOutputFunction->GetText();
					}
					
					TiXmlElement *l_pCGOutputMaxValue = l_pCGOutput->FirstChildElement("MaxValue");
					if ( NULL != l_pCGOutputMaxValue )
					{
						new_cg.Output.maxvalue = atoi(l_pCGOutputMaxValue->GetText());
					}
					
					TiXmlElement *l_pCGOutputMinValue = l_pCGOutput->FirstChildElement("MinValue");
					if ( NULL != l_pCGOutputMinValue )
					{
						new_cg.Output.minvalue = atoi(l_pCGOutputMinValue->GetText());
					}
					
					TiXmlElement *l_pCGOutputNeutralValue = l_pCGOutput->FirstChildElement("NeutralValue");
					if ( NULL != l_pCGOutputNeutralValue )
					{
						new_cg.Output.neutralvalue = atoi(l_pCGOutputNeutralValue->GetText());
					}
					
					TiXmlElement *l_pCGOutputDeadband = l_pCGOutput->FirstChildElement("Deadband");
					if ( NULL != l_pCGOutputDeadband )
					{
						new_cg.Output.deadband = atof(l_pCGOutputDeadband->GetText());
					}
				}
				
				TiXmlElement *l_pCGGain = l_pControlGroup->FirstChildElement("Gain");
				if ( NULL != l_pCGGain )
				{
					TiXmlElement *l_pCGGainType = l_pCGGain->FirstChildElement("Type");
					if ( NULL != l_pCGGainType )
					{
						new_cg.Gain.type = l_pCGGainType->GetText();
					}
					
					TiXmlElement *l_pCGGainP = l_pCGGain->FirstChildElement("Proportional");
					if ( NULL != l_pCGGainP )
					{
						new_cg.Gain.P = atof(l_pCGGainP->GetText());
					}
					
					TiXmlElement *l_pCGGainI = l_pCGGain->FirstChildElement("Integral");
					if ( NULL != l_pCGGainI )
					{
						new_cg.Gain.I = atof(l_pCGGainI->GetText());
					}
					
					TiXmlElement *l_pCGGainD = l_pCGGain->FirstChildElement("Derivative");
					if ( NULL != l_pCGGainD )
					{
						new_cg.Gain.D = atof(l_pCGGainD->GetText());
					}
				}

				controlgroups.push_back(new_cg);
				l_pControlGroup = l_pControlGroup->NextSiblingElement( "ControlGroup" );
			}
		}
	}
	return false;
}
int AutoDriveNodeProcess::get_sensor(std::string v)
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