#include "TopicRemapperNodeProcess.h"
eros::diagnostic  TopicRemapperNodeProcess::finish_initialization()
{
    eros::diagnostic diag = root_diagnostic;
	diag = update_diagnostic(REMOTE_CONTROL,NOTICE,NOERROR,"No Message Processed Yet.");
    return diag;
}
eros::diagnostic TopicRemapperNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if(initialized == true)
	{
		ready = true;

	}
	
	diag = update_baseprocess(t_dt,t_ros_time);
	if((is_initialized() == true) and (is_ready() == true))
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error.");
	}
	if(diag.Level <= NOTICE)
	{

		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running.");

	}
	return diag;
}
eros::diagnostic TopicRemapperNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = root_diagnostic;
	return diag;
}
std::vector<eros::diagnostic> TopicRemapperNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
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
	return diaglist;
}
std::vector<eros::diagnostic> TopicRemapperNodeProcess::check_programvariables()
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
int TopicRemapperNodeProcess::parse_topicmapfile(TiXmlDocument doc)
{
	TiXmlElement *l_pRootElement = doc.RootElement();

	if( NULL != l_pRootElement )
	{
		// set of &lt;person&gt; tags
		TiXmlElement *l_pTopicMapList = l_pRootElement->FirstChildElement( "TopicMapList" );

		if ( NULL != l_pTopicMapList )
		{
			TiXmlElement *l_pTopicMap = l_pTopicMapList->FirstChildElement( "TopicMap" );

			while( l_pTopicMap )
			{
				TopicMap newtopicmap;
				OutputMode outputmode;
				TiXmlElement *l_pOutputMode = l_pTopicMap->FirstChildElement("OutputMode");
				if(NULL != l_pOutputMode)
				{
					TiXmlElement *l_pMode = l_pOutputMode->FirstChildElement("Mode");
					if(NULL != l_pMode)
					{
						outputmode.mode = l_pMode->GetText();
					}

					TiXmlElement *l_pType = l_pOutputMode->FirstChildElement("Type");
					if(NULL != l_pType)
					{
						outputmode.type = l_pType->GetText();
					}

					TiXmlElement *l_pTopic = l_pOutputMode->FirstChildElement("Topic");
					if(NULL != l_pTopic)
					{
						outputmode.topic = l_pTopic->GetText();
					}

					TiXmlElement *l_pName = l_pOutputMode->FirstChildElement("Name");
					if(NULL != l_pName)
					{
						outputmode.name = l_pName->GetText();
					}

					TiXmlElement *l_pIndex = l_pOutputMode->FirstChildElement("Index");
					if(NULL != l_pIndex)
					{
						outputmode.index = std::atoi(l_pIndex->GetText());
					}

					TiXmlElement *l_pRequiredValue = l_pOutputMode->FirstChildElement("RequiredValue");
					if(NULL != l_pRequiredValue)
					{
						outputmode.required_value = std::atoi(l_pRequiredValue->GetText());
					}
				}
				else
				{
					outputmode.mode = "Direct";
				}
				newtopicmap.outputmode = outputmode;

                InputChannel in;

				//Input
				TiXmlElement *l_pInput = l_pTopicMap->FirstChildElement( "InputChannel" );
				if(NULL != l_pInput)
				{
					TiXmlElement *l_pInputType = l_pInput->FirstChildElement( "Type" );
					if(NULL != l_pInputType)
					{
						std::string input_type = l_pInputType->GetText();
						if(input_type == "sensor_msgs/Joy")
						{
							in.type = input_type;
							TiXmlElement *l_pTopic = l_pInput->FirstChildElement( "Topic" );
							if(NULL != l_pTopic)
							{
								in.topic = l_pTopic->GetText();
							}
                            else { return -1; }
							TiXmlElement *l_pName = l_pInput->FirstChildElement( "Name" );
							if(NULL != l_pName)
							{
								in.name = l_pName->GetText();
							}
                            else { return -1; }

                            TiXmlElement *l_pIndex = l_pInput->FirstChildElement("Index");
                            if(NULL != l_pIndex)
                            {
                                in.index = std::atoi(l_pIndex->GetText());
                            }

                            TiXmlElement *l_pMinValue = l_pInput->FirstChildElement("MinValue");
                            if(NULL != l_pMinValue)
                            {
                                in.minvalue = std::atof(l_pMinValue->GetText());
                            }
                            else
                            {
                            	in.minvalue = 0.0;
                            }

                            TiXmlElement *l_pMaxValue = l_pInput->FirstChildElement("MaxValue");
                            if(NULL != l_pMaxValue)
                            {
                                in.maxvalue = std::atof(l_pMaxValue->GetText());
                            }
                            else
                            {
                            	in.maxvalue = 0.0;
                            }
						}
						else
						{
							char tempstr[128];
							sprintf(tempstr,"Input Topic: %s not supported.  Exiting.",input_type.c_str());
							return 0;
						}
					}
				}

				//Outputs
				std::vector<OutputChannel> outputs;
				//std::vector<ros::Publisher> pubs;
				TiXmlElement *l_Outputs = l_pTopicMap->FirstChildElement("Outputs");
				if ( NULL != l_Outputs )
				{
					TiXmlElement *l_pOutputChannel = l_Outputs->FirstChildElement( "OutputChannel" );
					while( l_pOutputChannel )
					{
						OutputChannel out;
						TiXmlElement *l_pOutputType = l_pOutputChannel->FirstChildElement( "Type" );
						if(NULL != l_pOutputType)
						{
							std::string output_type = l_pOutputType->GetText();
							if((output_type == "eros/pin") || (output_type == "std_msgs/Float32") || (output_type == "sensor_msgs/JointState"))
							{
								out.type = output_type;
								TiXmlElement *l_pTopic = l_pOutputChannel->FirstChildElement( "Topic" );
								if(NULL != l_pTopic)
								{
									out.topic = l_pTopic->GetText();
								}
								else { return -1; }

								TiXmlElement *l_pParentDevice = l_pOutputChannel->FirstChildElement( "ParentDevice" );
								if(NULL != l_pParentDevice)
								{
									out.parentdevice = l_pParentDevice->GetText();
								}
								else { return -1; }

								TiXmlElement *l_pPinNumber = l_pOutputChannel->FirstChildElement( "PinNumber" );
								if(NULL != l_pPinNumber)
								{
									out.pinnumber = std::atoi(l_pPinNumber->GetText());
								}

								TiXmlElement *l_pFunction = l_pOutputChannel->FirstChildElement( "Function" );
								if(NULL != l_pFunction)
								{
									out.function = l_pFunction->GetText();
								}
								else
								{
									if(output_type == "eros/pin")
									{
										printf("Output: %s Requires the OutputChannel Tag: Function\n",out.topic.c_str());
										return -1;
									}
									else
									{
										out.function = "PINMODE_UNDEFINED";;
									}
								}

								TiXmlElement *l_pMaxValue = l_pOutputChannel->FirstChildElement( "MaxValue" );
								if(NULL != l_pMaxValue)
								{
									out.maxvalue = std::atof(l_pMaxValue->GetText());
								}
								else { out.maxvalue = 0.0; }

								TiXmlElement *l_pMinValue = l_pOutputChannel->FirstChildElement( "MinValue" );
								if(NULL != l_pMinValue)
								{
									out.minvalue = std::atof(l_pMinValue->GetText());
								}
								else { out.minvalue = 0.0; }

								TiXmlElement *l_pNeutralValue = l_pOutputChannel->FirstChildElement( "NeutralValue" );
								if(NULL != l_pNeutralValue)
								{
									out.neutralvalue = std::atof(l_pNeutralValue->GetText());
								}
								else { out.neutralvalue = 0.0; }
								out.value = out.neutralvalue;

								TiXmlElement *l_pDeadband = l_pOutputChannel->FirstChildElement( "Deadband" );
								if(NULL != l_pDeadband)
								{
									out.deadband = std::atof(l_pDeadband->GetText());
								}
								else { out.deadband = 0.0; }
							}
							else
							{
								char tempstr[128];
								sprintf(tempstr,"Output Topic: %s not supported.  Exiting.",output_type.c_str());
								printf("%s\n",tempstr);
							}
						}
						//ros::Publisher pub;
						//pubs.push_back(pub);
						outputs.push_back(out);
						l_pOutputChannel = l_pOutputChannel->NextSiblingElement( "OutputChannel" );
					}

				}
                newtopicmap.in = in;
                newtopicmap.outs = outputs;
                //newtopicmap.pubs = pubs;
				TopicMaps.push_back(newtopicmap);
				l_pTopicMap = l_pTopicMap->NextSiblingElement( "TopicMap" );
			}
		}
	}
	return TopicMaps.size();
}
eros::diagnostic TopicRemapperNodeProcess::load(std::string topicmapfilepath)
{
    eros::diagnostic diag = root_diagnostic;
    TiXmlDocument topicmap_doc(topicmapfilepath);
    bool topicmapfile_loaded = topicmap_doc.LoadFile();
	if(topicmapfile_loaded == true)
	{
		if(parse_topicmapfile(topicmap_doc) <= 0)
		{
            char tempstr[512];
            sprintf(tempstr,"Didn't read any TopicMaps in: %s",topicmapfilepath.c_str());
			diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
            return diag;
		}
        else
        {
            char tempstr[512];
            sprintf(tempstr,"Loaded: %s with %d Topic Maps",topicmapfilepath.c_str(),(int)TopicMaps.size());
			diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,std::string(tempstr));
            return diag;
        }
    }
    else
    {
        char tempstr[512];
        sprintf(tempstr,"Unable to load: %s",topicmapfilepath.c_str());
		diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
        return diag;
    }
}
std::string TopicRemapperNodeProcess::print_topicmaps()
{
    std::ostringstream ss;
    ss << "------ Topic Map -----" << std::endl;
    for(std::size_t i = 0; i < TopicMaps.size();i++)
    {
        if(TopicMaps.at(i).outputmode.mode == "Direct")
        {
            ss << "i: " << i << " Output Mode: " << TopicMaps.at(i).outputmode.mode << std::endl;
        }
        else
        {
            ss << "i: " << i << " Output Mode: " << TopicMaps.at(i).outputmode.mode << " Type: " << TopicMaps.at(i).outputmode.type <<
                " Topic: " << TopicMaps.at(i).outputmode.topic << " Name: " << TopicMaps.at(i).outputmode.name << " Index: " << TopicMaps.at(i).outputmode.index <<
                " Required Value: " << TopicMaps.at(i).outputmode.required_value << std::endl;
        }

        {
            ss << "i: " << i << " Input Channel Type: " << TopicMaps.at(i).in.type << " Topic: " << TopicMaps.at(i).in.topic << " Name: " <<
                TopicMaps.at(i).in.name << " Index: " << TopicMaps.at(i).in.index << " Min Value: " << TopicMaps.at(i).in.minvalue <<
                " Max Value: " << TopicMaps.at(i).in.maxvalue << std::endl;
        }
        {
            for(std::size_t j = 0; j < TopicMaps.at(i).outs.size();j++)
            {
               ss << "i: " << i << " Output Channel[" << j << "]: Type: " << TopicMaps.at(i).outs.at(j).type << " Topic: " << TopicMaps.at(i).outs.at(j).topic <<
                " ParentDevice: " << TopicMaps.at(i).outs.at(j).parentdevice << " Pin: " << TopicMaps.at(i).outs.at(j).pinnumber << " Function: " <<
                TopicMaps.at(i).outs.at(j).function << " Max Value: " << TopicMaps.at(i).outs.at(j).maxvalue << " Neutral Value: " << TopicMaps.at(i).outs.at(j).neutralvalue <<
                " Min Value: " << TopicMaps.at(i).outs.at(j).minvalue << " Deadband: " << TopicMaps.at(i).outs.at(j).deadband << std::endl;
            }
        }
    }
    return ss.str();
}
/*
void TopicRemapperNodeProcess::set_topicmap_sub(std::size_t i,ros::Subscriber sub)
{
    TopicMaps.at(i).sub = sub;
}
*/
double TopicRemapperNodeProcess::scale_value(double x,double neutral,double x1,double x2,double y1,double y2, double deadband)
{
    double out = 0.0;
    if(x < (-1.0*deadband))
    {
        double m = (y1-neutral)/(x1-(-1.0*deadband));
        out = m*(x-x1)+y1;
    }
    else if(x > deadband)
    {
    	double m = (y2-neutral)/(x2-(deadband));
    	out = m*(x-x2)+y2;
    }
    else
    {
        out = neutral;
    }
    if(y2 > y1)
    {
    	if(out > y2) { out = y2; }
    	if(out < y1) { out = y1; }
    }
    else
    {
    	if(out > y1) { out = y1; }
    	if(out < y2) { out = y2; }
    }
    return out;
}
eros::diagnostic TopicRemapperNodeProcess::new_joymsg(sensor_msgs::Joy msg,std::string topic)
{
    eros::diagnostic diag = root_diagnostic;
    for(std::size_t i = 0; i < TopicMaps.size();i++)
	{
        TopicMap map = TopicMaps.at(i);
		if(map.in.topic == topic)
		{
            if(map.in.name == "axis")
            {
                for(std::size_t j = 0; j < TopicMaps.at(i).outs.size();j++)
                {
                	bool update_output = false;
                	if(TopicMaps.at(i).outputmode.mode == "Direct")
                	{
                		update_output = true;
                	}
                	else
                	{
                		if(TopicMaps.at(i).outputmode.mode == "Switch")
                		{
                			if(TopicMaps.at(i).outputmode.topic == map.in.topic)
                			{
                				if(TopicMaps.at(i).outputmode.name == "button")
                				{
                					if(msg.buttons[TopicMaps.at(i).outputmode.index] == TopicMaps.at(i).outputmode.required_value)
                					{
                						update_output = true;
                					}
                				}
                			}
                		}
                	}
                	OutputChannel ch = TopicMaps.at(i).outs.at(j);
                	if(update_output == true)
                	{
                		double in_value = msg.axes[map.in.index];

						if(ch.type == "eros/pin")
						{
							double out = scale_value(in_value,ch.neutralvalue,map.in.minvalue,map.in.maxvalue,ch.minvalue,ch.maxvalue,ch.deadband);
							eros::pin newpin;
							newpin.stamp = msg.header.stamp;
							newpin.ParentDevice = ch.parentdevice;
							newpin.DefaultValue = (int)ch.neutralvalue;
							newpin.Function = ch.function;
							newpin.Number = ch.pinnumber;
							newpin.Value = (int)out;
							//p_pwmoutputs.pins.push_back(newpin);
							map.outs.at(j).value = newpin.Value;
							//map.pubs.at(j).publish(newpin);
						}
						else if(ch.type == "std_msgs/Float32")
						{
							double out = scale_value(in_value,ch.neutralvalue,map.in.minvalue,map.in.maxvalue,ch.minvalue,ch.maxvalue,ch.deadband);
							std_msgs::Float32 value;
							value.data = out;
							map.outs.at(j).value = out;
							//map.pubs.at(j).publish(value);
						}
						else if(ch.type == "sensor_msgs/JointState")
						{
							std::string member = TopicMaps.at(i).outs.at(j).topic;
							double out = scale_value(in_value,ch.neutralvalue,map.in.minvalue,map.in.maxvalue,ch.minvalue,ch.maxvalue,ch.deadband);
							sensor_msgs::JointState joint;
							//joint.header.stamp = ros::Time::now();
							joint.name.resize(1);

							std::size_t found1 = member.substr(1).find("/");
							std::size_t found2 = member.substr(found1+2).find("/")+found1;
							std::string variable = member.substr(found1+2,found2-found1);
							std::string name = member.substr(found2+3);
							joint.name[0] = name;
							if(variable == "position")
							{
								joint.position.resize(1);
								joint.position[0] = out;

							}
							else if(variable == "velocity")
							{
								joint.velocity.resize(1);
								joint.velocity[0] = out;
							}
							else if(variable == "effort")
							{
								joint.effort.resize(1);
								joint.effort[0] = out;
							}
							else
							{
								char tempstr[512];
								sprintf(tempstr,"OutputChannel JointState Not Supported: %s\n",member.c_str());
								diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
								return diag;
								//logger->log_warn(std::string(tempstr));
							}
							map.outs.at(j).value = out;
							//map.pubs.at(j).publish(joint);
						}

                	}
                	else
                	{

                		if(ch.type == "eros/pin")
                		{
                			eros::pin newpin;
                			newpin.stamp = msg.header.stamp;
                			newpin.ParentDevice = ch.parentdevice;
                			newpin.DefaultValue = (int)ch.neutralvalue;
                			newpin.Function = ch.function;
                			newpin.Number = ch.pinnumber;
                			newpin.Value = map.outs.at(j).value;
                			//map.pubs.at(j).publish(newpin);
                		}

                		else if(ch.type == "std_msgs/Float32")
                		{
                			std_msgs::Float32 value;
                			value.data = map.outs.at(j).value;
                			//map.pubs.at(j).publish(value);
                		}

                		else if(ch.type == "sensor_msgs/JointState")
                		{
                			std::string member = TopicMaps.at(i).outs.at(j).topic;
                			sensor_msgs::JointState joint;
                			//joint.header.stamp = ros::Time::now();
                			joint.name.resize(1);

                			std::size_t found1 = member.substr(1).find("/");
                			std::size_t found2 = member.substr(found1+2).find("/")+found1;
                			std::string variable = member.substr(found1+2,found2-found1);
                			std::string name = member.substr(found2+3);
                			joint.name[0] = name;
                			if(variable == "position")
                			{
                				joint.position.resize(1);
                				joint.position[0] = map.outs.at(j).value;

                			}
                			else if(variable == "velocity")
                			{
                				joint.velocity.resize(1);
                				joint.velocity[0] = map.outs.at(j).value;
                			}
                			else if(variable == "effort")
                			{
                				joint.effort.resize(1);
                				joint.effort[0] = map.outs.at(j).value;
                			}
                			else
                			{
                				char tempstr[512];
                				sprintf(tempstr,"OutputChannel JointState Not Supported: %s\n",member.c_str());
                				diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
                				//logger->log_warn(std::string(tempstr));
                			}
                			//map.pubs.at(j).publish(joint);
                		}

                	}

                }
            }
            if(map.in.name == "button")
            {

            	for(std::size_t j = 0; j < TopicMaps.at(i).outs.size();j++)
				{
            		OutputChannel ch = TopicMaps.at(i).outs.at(j);
            		eros::pin newpin;
					newpin.stamp = msg.header.stamp;
					newpin.ParentDevice = ch.parentdevice;
					newpin.Function = ch.function;
					newpin.Number = ch.pinnumber;
					newpin.Value = msg.buttons[map.in.index];
					//p_digitaloutputs.pins.push_back(newpin);
					//map.pubs.at(j).publish(newpin);
				}
            }
		}
		TopicMaps.at(i) = map;
	}
    char tempstr[512];
    sprintf(tempstr,"Processed joystick message from topic: %s",topic.c_str());
	diag = update_diagnostic(REMOTE_CONTROL,INFO,NOERROR,std::string(tempstr));
    return diag;
}
std::vector<eros::pin> TopicRemapperNodeProcess::get_outputs_pins()
{
    std::vector<eros::pin> outs;
    for(std::size_t i = 0; i < TopicMaps.size(); i++)
    {
        for(std::size_t j = 0; j < TopicMaps.at(i).outs.size(); j++)
        {
            if(TopicMaps.at(i).outs.at(j).type == "eros/pin")
            {
                eros::pin out;
                OutputChannel ch = TopicMaps.at(i).outs.at(j);
                out.ParentDevice = ch.parentdevice;
                out.Function = ch.function;
                out.Number = ch.pinnumber;
                out.Value = ch.value;
                outs.push_back(out);
            }
        }
    }
    return outs;

}
std::vector<std_msgs::Float32> TopicRemapperNodeProcess::get_outputs_float32()
{
    std::vector<std_msgs::Float32> outs;
    for(std::size_t i = 0; i < TopicMaps.size(); i++)
    {
        for(std::size_t j = 0; j < TopicMaps.at(i).outs.size(); j++)
        {
            if(TopicMaps.at(i).outs.at(j).type == "std_msgs/Float32")
            {
                std_msgs::Float32 out;
                OutputChannel ch = TopicMaps.at(i).outs.at(j);
                out.data = ch.value;
                outs.push_back(out);
            }
        }
    }
    return outs;
}
