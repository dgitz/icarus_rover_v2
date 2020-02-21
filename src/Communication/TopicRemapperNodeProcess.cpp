#include "TopicRemapperNodeProcess.h"
eros::diagnostic TopicRemapperNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	reset();
	diag = update_diagnostic(REMOTE_CONTROL, NOTICE, NOERROR, "No Message Processed Yet.");
	return diag;
}
eros::diagnostic TopicRemapperNodeProcess::update(double t_dt, double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if (task_state == TASKSTATE_PAUSE)
	{
	}
	else if (task_state == TASKSTATE_RESET)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if (v == false)
		{
			diag = update_diagnostic(SOFTWARE, ERROR, DIAGNOSTIC_FAILED,
									 "Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
	}
	else if (task_state == TASKSTATE_INITIALIZED)
	{
		request_statechange(TASKSTATE_RUNNING);
	}
	else if (task_state == TASKSTATE_RUNNING)
	{
	}
	else if (task_state != TASKSTATE_RUNNING)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if (v == false)
		{
			diag = update_diagnostic(SOFTWARE, ERROR, DIAGNOSTIC_FAILED,
									 "Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
	}
	diag = update_baseprocess(t_dt, t_ros_time);
	if (diag.Level <= NOTICE)
	{
		diag = update_diagnostic(DATA_STORAGE, INFO, NOERROR, "No Error.");
		diag = update_diagnostic(SOFTWARE, INFO, NOERROR, "Node Running");
	}
	return diag;
}
eros::diagnostic TopicRemapperNodeProcess::new_devicemsg(__attribute__((unused)) const eros::device::ConstPtr &device)
{
	eros::diagnostic diag = root_diagnostic;
	return diag;
}
std::vector<eros::diagnostic> TopicRemapperNodeProcess::new_commandmsg(const eros::command::ConstPtr &t_msg)
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
	else if (t_msg->Command == ROVERCOMMAND_TASKCONTROL)
	{
		if (node_name.find(t_msg->CommandText) != std::string::npos)
		{
			uint8_t prev_taskstate = get_taskstate();
			bool v = request_statechange(t_msg->Option2);
			if (v == false)
			{
				diag = update_diagnostic(SOFTWARE, ERROR, DIAGNOSTIC_FAILED,
										 "Unallowed State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}
			else
			{
				if (task_state == TASKSTATE_RESET)
				{
					reset();
				}
				diag = update_diagnostic(SOFTWARE, NOTICE, DIAGNOSTIC_PASSED,
										 "Commanded State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}
		}
	}
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		diag = update_diagnostic(diaglist.at(i));
	}
	return diaglist;
}
std::vector<eros::diagnostic> TopicRemapperNodeProcess::check_programvariables()
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
eros::diagnostic TopicRemapperNodeProcess::parse_topicmapfile(eros::diagnostic diag, TiXmlDocument doc)
{
	diag.Diagnostic_Type = DATA_STORAGE;
	TiXmlElement *l_pRootElement = doc.RootElement();

	if (NULL != l_pRootElement)
	{
		// set of &lt;person&gt; tags
		TiXmlElement *l_pTopicMapList = l_pRootElement->FirstChildElement("TopicMapList");

		if (NULL != l_pTopicMapList)
		{
			TiXmlElement *l_pTopicMap = l_pTopicMapList->FirstChildElement("TopicMap");

			while (l_pTopicMap)
			{
				TopicMap newtopicmap;
				OutputMode outputmode;
				TiXmlElement *l_pOutputMode = l_pTopicMap->FirstChildElement("OutputMode");
				if (NULL != l_pOutputMode)
				{
					TiXmlElement *l_pMode = l_pOutputMode->FirstChildElement("Mode");
					if (NULL != l_pMode)
					{
						outputmode.mode = map_outputmode_enum(l_pMode->GetText());
						if (outputmode.mode == TopicRemapperNodeProcess::OutputModeEnum::UNKNOWN)
						{
							diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,
							"Output Mode: " + std::string(l_pMode->GetText()) + " Is Not Supported.");
							return diag;
						}
					}

					TiXmlElement *l_pType = l_pOutputMode->FirstChildElement("Type");
					if (NULL != l_pType)
					{
						outputmode.type = l_pType->GetText();
					}

					TiXmlElement *l_pTopic = l_pOutputMode->FirstChildElement("Topic");
					if (NULL != l_pTopic)
					{
						outputmode.topic = l_pTopic->GetText();
					}

					TiXmlElement *l_pName = l_pOutputMode->FirstChildElement("Name");
					if (NULL != l_pName)
					{
						outputmode.name = l_pName->GetText();
					}

					TiXmlElement *l_pIndex = l_pOutputMode->FirstChildElement("Index");
					if (NULL != l_pIndex)
					{
						outputmode.index = std::atoi(l_pIndex->GetText());
					}

					TiXmlElement *l_pRequiredValue = l_pOutputMode->FirstChildElement("RequiredValue");
					if (NULL != l_pRequiredValue)
					{
						outputmode.required_value = std::atoi(l_pRequiredValue->GetText());
					}
				}
				newtopicmap.outputmode = outputmode;

				std::vector<InputChannel> inputs;
				TiXmlElement *l_Inputs = l_pTopicMap->FirstChildElement("Inputs");
				if (NULL != l_Inputs)
				{
					TiXmlElement *l_pInput = l_Inputs->FirstChildElement("InputChannel");
					while (l_pInput)
					{
						if (NULL != l_pInput)
						{
							InputChannel in;
							TiXmlElement *l_pInputType = l_pInput->FirstChildElement("Type");
							if (NULL != l_pInputType)
							{
								std::string input_type = l_pInputType->GetText();
								if (input_type == "sensor_msgs/Joy")
								{
									in.type = input_type;
									TiXmlElement *l_pTopic = l_pInput->FirstChildElement("Topic");
									if (NULL != l_pTopic)
									{
										in.topic = l_pTopic->GetText();
									}
									else
									{
										diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Input Channel Topic Not Found.");
										return diag;
									}
									TiXmlElement *l_pName = l_pInput->FirstChildElement("Name");
									if (NULL != l_pName)
									{
										in.name = l_pName->GetText();
									}
									else
									{
										diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Input Channel Name Not Found.");
										return diag;
									}

									TiXmlElement *l_pIndex = l_pInput->FirstChildElement("Index");
									if (NULL != l_pIndex)
									{
										in.index = std::atoi(l_pIndex->GetText());
									}

									TiXmlElement *l_pMinValue = l_pInput->FirstChildElement("MinValue");
									if (NULL != l_pMinValue)
									{
										in.minvalue = std::atof(l_pMinValue->GetText());
									}
									else
									{
										in.minvalue = 0.0;
									}

									TiXmlElement *l_pMaxValue = l_pInput->FirstChildElement("MaxValue");
									if (NULL != l_pMaxValue)
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
									diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,
										"Input Channel Type: " + input_type + " Not Supported.");
									return diag;
								}
							}
							inputs.push_back(in);
							l_pInput = l_pInput->NextSiblingElement("InputChannel");
						}
					}
				}
				if ((newtopicmap.outputmode.mode == TopicRemapperNodeProcess::OutputModeEnum::DIRECT) ||
					(newtopicmap.outputmode.mode == TopicRemapperNodeProcess::OutputModeEnum::SWITCH))
				{
					if (inputs.size() != 1)
					{
						diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,
							"Output Mode: " + map_outputmode_string(newtopicmap.outputmode.mode) + " Requires Exactly 1 Input Channel.");
						return diag;
					}
				}
				if (newtopicmap.outputmode.mode == TopicRemapperNodeProcess::OutputModeEnum::ARCADEMIX)
				{
					if (inputs.size() != 2)
					{
						diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,
							"Output Mode: " + map_outputmode_string(newtopicmap.outputmode.mode) + " Requires Exactly 2 Input Channels.");
						return diag;
					}
				}
				//Outputs
				std::vector<OutputChannel> outputs;
				//std::vector<ros::Publisher> pubs;
				TiXmlElement *l_Outputs = l_pTopicMap->FirstChildElement("Outputs");
				if (NULL != l_Outputs)
				{
					TiXmlElement *l_pOutputChannel = l_Outputs->FirstChildElement("OutputChannel");
					while (l_pOutputChannel)
					{
						OutputChannel out;
						TiXmlElement *l_pOutputType = l_pOutputChannel->FirstChildElement("Type");
						if (NULL != l_pOutputType)
						{
							std::string output_type = l_pOutputType->GetText();
							if ((output_type == "eros/pin") ||
								(output_type == "std_msgs/Float32") ||
								(output_type == "std_msgs/Bool") ||
								(output_type == "sensor_msgs/JointState"))
							{
								out.type = output_type;
								TiXmlElement *l_pTopic = l_pOutputChannel->FirstChildElement("Topic");
								if (NULL != l_pTopic)
								{
									out.topic = l_pTopic->GetText();
								}
								else
								{
									diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Output Channel Topic Not Found.");
									return diag;
								}
								if(output_type == "eros/pin")
								{
									TiXmlElement *l_pParentDevice = l_pOutputChannel->FirstChildElement("ParentDevice");
									if (NULL != l_pParentDevice)
									{
										out.parentdevice = l_pParentDevice->GetText();
									}
									else
									{
										diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,
											"Output Channel Parent Device Not Found.");
										return diag;
									}

									TiXmlElement *l_pPinName = l_pOutputChannel->FirstChildElement("PinName");
									if (NULL != l_pPinName)
									{
										out.pinname = l_pPinName->GetText();
									}

									TiXmlElement *l_pFunction = l_pOutputChannel->FirstChildElement("Function");
									if (NULL != l_pFunction)
									{
										out.function = l_pFunction->GetText();
									}
									else
									{
										diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,
											"Output Channel Function Not Found.");
											return diag;
									}
								}

								TiXmlElement *l_pMaxValue = l_pOutputChannel->FirstChildElement("MaxValue");
								if (NULL != l_pMaxValue)
								{
									out.maxvalue = std::atof(l_pMaxValue->GetText());
								}
								else
								{
									out.maxvalue = 0.0;
								}

								TiXmlElement *l_pMinValue = l_pOutputChannel->FirstChildElement("MinValue");
								if (NULL != l_pMinValue)
								{
									out.minvalue = std::atof(l_pMinValue->GetText());
								}
								else
								{
									out.minvalue = 0.0;
								}

								TiXmlElement *l_pNeutralValue = l_pOutputChannel->FirstChildElement("NeutralValue");
								if (NULL != l_pNeutralValue)
								{
									out.neutralvalue = std::atof(l_pNeutralValue->GetText());
								}
								else
								{
									out.neutralvalue = 0.0;
								}
								out.value = out.neutralvalue;

								TiXmlElement *l_pDeadband = l_pOutputChannel->FirstChildElement("Deadband");
								if (NULL != l_pDeadband)
								{
									out.deadband = std::atof(l_pDeadband->GetText());
								}
								else
								{
									out.deadband = 0.0;
								}
							}
							else
							{
								diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,
									"Output Channel Topic Type: " + output_type + " Not Supported.");
								return diag;
							}
						}
						//ros::Publisher pub;
						//pubs.push_back(pub);
						outputs.push_back(out);
						l_pOutputChannel = l_pOutputChannel->NextSiblingElement("OutputChannel");
					}
				}
				newtopicmap.ins = inputs;
				newtopicmap.outs = outputs;
				//newtopicmap.pubs = pubs;
				TopicMaps.push_back(newtopicmap);
				l_pTopicMap = l_pTopicMap->NextSiblingElement("TopicMap");
			}
		}
	}
	diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,"Topic Maps Initialized.");
	return diag;
}
eros::diagnostic TopicRemapperNodeProcess::load(std::string topicmapfilepath)
{
	eros::diagnostic diag = root_diagnostic;
	TiXmlDocument topicmap_doc(topicmapfilepath);
	bool topicmapfile_loaded = topicmap_doc.LoadFile();
	if (topicmapfile_loaded == true)
	{
		diag = parse_topicmapfile(diag, topicmap_doc);
		if (diag.Level > NOTICE)
		{
			diag = update_diagnostic(diag);
			return diag;
		}
		else
		{
			diag = update_diagnostic(DATA_STORAGE, INFO, INITIALIZING, "Loaded: " + topicmapfilepath + " with " + 
				std::to_string(TopicMaps.size()) + " Topic Maps");
			return diag;
		}
	}
	else
	{
		diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, "Unable to find: " + topicmapfilepath);
		return diag;
	}
}
std::string TopicRemapperNodeProcess::print_topicmaps()
{
	std::ostringstream ss;
	ss << "------ Topic Map -----" << std::endl;
	for (std::size_t i = 0; i < TopicMaps.size(); i++)
	{
		if (TopicMaps.at(i).outputmode.mode == TopicRemapperNodeProcess::OutputModeEnum::DIRECT)
		{
			ss << "[" << i+1 << "/" << TopicMaps.size() << "]: Output Mode: " << map_outputmode_string(TopicMaps.at(i).outputmode.mode) << std::endl;
		}
		else
		{
			ss << "[" << i+1 << "/" << TopicMaps.size() << "]: Output Mode: " << map_outputmode_string(TopicMaps.at(i).outputmode.mode) << " Type: " << TopicMaps.at(i).outputmode.type << " Topic: " << TopicMaps.at(i).outputmode.topic << " Name: " << TopicMaps.at(i).outputmode.name << " Index: " << TopicMaps.at(i).outputmode.index << " Required Value: " << TopicMaps.at(i).outputmode.required_value << std::endl;
		}
		for(std::size_t j = 0; j < TopicMaps.at(i).ins.size(); ++j)
		{
			ss << "\tInput Channel[" << j << "] Type: " << TopicMaps.at(i).ins.at(0).type << " Topic: " << TopicMaps.at(i).ins.at(0).topic << " Name: " << TopicMaps.at(i).ins.at(0).name << " Index: " << TopicMaps.at(i).ins.at(0).index << " Min Value: " << TopicMaps.at(i).ins.at(0).minvalue << " Max Value: " << TopicMaps.at(i).ins.at(0).maxvalue << std::endl;
		}
		for (std::size_t j = 0; j < TopicMaps.at(i).outs.size(); j++)
		{
			ss << "\tOutput Channel[" << j << "]: Type: " << TopicMaps.at(i).outs.at(j).type << " Topic: " << TopicMaps.at(i).outs.at(j).topic << " ParentDevice: " << TopicMaps.at(i).outs.at(j).parentdevice << " Pin: " << TopicMaps.at(i).outs.at(j).pinname << " Function: " << TopicMaps.at(i).outs.at(j).function << " Max Value: " << TopicMaps.at(i).outs.at(j).maxvalue << " Neutral Value: " << TopicMaps.at(i).outs.at(j).neutralvalue << " Min Value: " << TopicMaps.at(i).outs.at(j).minvalue << " Deadband: " << TopicMaps.at(i).outs.at(j).deadband << std::endl;
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
double TopicRemapperNodeProcess::scale_value(double x, double neutral, double x1, double x2, double y1, double y2, double deadband)
{
	double out = 0.0;
	if (x < (-1.0 * deadband))
	{
		double m = (y1 - neutral) / (x1 - (-1.0 * deadband));
		out = m * (x - x1) + y1;
	}
	else if (x > deadband)
	{
		double m = (y2 - neutral) / (x2 - (deadband));
		out = m * (x - x2) + y2;
	}
	else
	{
		out = neutral;
	}
	if (y2 > y1)
	{
		if (out > y2)
		{
			out = y2;
		}
		if (out < y1)
		{
			out = y1;
		}
	}
	else
	{
		if (out > y1)
		{
			out = y1;
		}
		if (out < y2)
		{
			out = y2;
		}
	}
	return out;
}
eros::diagnostic TopicRemapperNodeProcess::new_joymsg(sensor_msgs::Joy msg, std::string topic)
{
	eros::diagnostic diag = root_diagnostic;
	for (std::size_t i = 0; i < TopicMaps.size(); i++)
	{
		TopicMap map = TopicMaps.at(i);
		if (map.outputmode.mode == TopicRemapperNodeProcess::OutputModeEnum::ARCADEMIX)
		{
			double x_axis,y_axis = 0.0;
			bool update_output = false;
			if((map.ins.at(0).topic == topic) && (map.ins.at(1).topic == topic))
			{
				if((map.ins.at(0).name == "axis") && (map.ins.at(1).name == "axis"))
				{
					update_output = true;
					x_axis = -msg.axes[map.ins.at(0).index];
					y_axis = msg.axes[map.ins.at(1).index];
				}
			}
			if(update_output == true)
			{
				OutputChannel ch1 = map.outs.at(0);
				OutputChannel ch2 = map.outs.at(1);
				if ((ch1.type == "eros/pin") && (ch2.type == "eros/pin"))
				{
					DrivePerc perc = arcade_mix(x_axis*100.0,y_axis*100.0);
					double l_out = scale_value(perc.left, ch1.neutralvalue, -100.0, 100.0, ch1.minvalue, ch1.maxvalue, 0.0);
					double r_out = scale_value(perc.right, ch2.neutralvalue, -100.0, 100.0, ch2.minvalue, ch2.maxvalue, 0.0);
					map.outs.at(0).value = (int)l_out;
					map.outs.at(1).value = (int)r_out;
				}
			}
		}
		else if ((map.outputmode.mode == TopicRemapperNodeProcess::OutputModeEnum::DIRECT) ||
			(map.outputmode.mode == TopicRemapperNodeProcess::OutputModeEnum::SWITCH))
		{
			if (map.ins.at(0).topic == topic)
			{
				if (map.ins.at(0).name == "axis")
				{
					for (std::size_t j = 0; j < TopicMaps.at(i).outs.size(); j++)
					{
						bool update_output = false;
						if (TopicMaps.at(i).outputmode.mode == TopicRemapperNodeProcess::OutputModeEnum::DIRECT)
						{
							update_output = true;
						}
						else
						{
							if (TopicMaps.at(i).outputmode.mode == TopicRemapperNodeProcess::OutputModeEnum::DIRECT)
							{
								if (TopicMaps.at(i).outputmode.topic == map.ins.at(0).topic)
								{
									if (TopicMaps.at(i).outputmode.name == "button")
									{
										if (msg.buttons[TopicMaps.at(i).outputmode.index] == TopicMaps.at(i).outputmode.required_value)
										{
											update_output = true;
										}
									}
								}
							}
						}
						OutputChannel ch = TopicMaps.at(i).outs.at(j);
						if (update_output == true)
						{
							double in_value = msg.axes[map.ins.at(0).index];

							if (ch.type == "eros/pin")
							{
								double out = scale_value(in_value, ch.neutralvalue, map.ins.at(0).minvalue,
														 map.ins.at(0).maxvalue, ch.minvalue, ch.maxvalue, ch.deadband);
								eros::pin newpin;
								newpin.stamp = msg.header.stamp;
								newpin.ParentDevice = ch.parentdevice;
								newpin.DefaultValue = (int)ch.neutralvalue;
								newpin.Function = ch.function;
								newpin.Value = (int)out;
								newpin.Name = ch.pinname;
								//p_pwmoutputs.pins.push_back(newpin);
								map.outs.at(j).value = newpin.Value;
								//map.pubs.at(j).publish(newpin);
							}
							else if (ch.type == "std_msgs/Float32")
							{
								double out = scale_value(in_value, ch.neutralvalue, map.ins.at(0).minvalue, map.ins.at(0).maxvalue,
														 ch.minvalue, ch.maxvalue, ch.deadband);
								std_msgs::Float32 value;
								value.data = out;
								map.outs.at(j).value = out;
								//map.pubs.at(j).publish(value);
							}
							else if (ch.type == "sensor_msgs/JointState")
							{
								std::string member = TopicMaps.at(i).outs.at(j).topic;
								double out = scale_value(in_value, ch.neutralvalue, map.ins.at(0).minvalue, map.ins.at(0).maxvalue,
														 ch.minvalue, ch.maxvalue, ch.deadband);
								sensor_msgs::JointState joint;
								//joint.header.stamp = ros::Time::now();
								joint.name.resize(1);

								std::size_t found1 = member.substr(1).find("/");
								std::size_t found2 = member.substr(found1 + 2).find("/") + found1;
								std::string variable = member.substr(found1 + 2, found2 - found1);
								std::string name = member.substr(found2 + 3);
								joint.name[0] = name;
								if (variable == "position")
								{
									joint.position.resize(1);
									joint.position[0] = out;
								}
								else if (variable == "velocity")
								{
									joint.velocity.resize(1);
									joint.velocity[0] = out;
								}
								else if (variable == "effort")
								{
									joint.effort.resize(1);
									joint.effort[0] = out;
								}
								else
								{
									char tempstr[512];
									sprintf(tempstr, "OutputChannel JointState Not Supported: %s\n", member.c_str());
									diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
									return diag;
									//logger->log_warn(std::string(tempstr));
								}
								map.outs.at(j).value = out;
								//map.pubs.at(j).publish(joint);
							}
						}
						else
						{

							if (ch.type == "eros/pin")
							{
								eros::pin newpin;
								newpin.stamp = msg.header.stamp;
								newpin.ParentDevice = ch.parentdevice;
								newpin.DefaultValue = (int)ch.neutralvalue;
								newpin.Function = ch.function;
								newpin.Value = map.outs.at(j).value;
								//map.pubs.at(j).publish(newpin);
							}

							else if (ch.type == "std_msgs/Float32")
							{
								std_msgs::Float32 value;
								value.data = map.outs.at(j).value;
								//map.pubs.at(j).publish(value);
							}

							else if (ch.type == "sensor_msgs/JointState")
							{
								std::string member = TopicMaps.at(i).outs.at(j).topic;
								sensor_msgs::JointState joint;
								//joint.header.stamp = ros::Time::now();
								joint.name.resize(1);

								std::size_t found1 = member.substr(1).find("/");
								std::size_t found2 = member.substr(found1 + 2).find("/") + found1;
								std::string variable = member.substr(found1 + 2, found2 - found1);
								std::string name = member.substr(found2 + 3);
								joint.name[0] = name;
								if (variable == "position")
								{
									joint.position.resize(1);
									joint.position[0] = map.outs.at(j).value;
								}
								else if (variable == "velocity")
								{
									joint.velocity.resize(1);
									joint.velocity[0] = map.outs.at(j).value;
								}
								else if (variable == "effort")
								{
									joint.effort.resize(1);
									joint.effort[0] = map.outs.at(j).value;
								}
								else
								{
									char tempstr[512];
									sprintf(tempstr, "OutputChannel JointState Not Supported: %s\n", member.c_str());
									diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
									//logger->log_warn(std::string(tempstr));
								}
								//map.pubs.at(j).publish(joint);
							}
						}
					}
				}
				if (map.ins.at(0).name == "button")
				{
					for (std::size_t j = 0; j < TopicMaps.at(i).outs.size(); j++)
					{
						OutputChannel ch = TopicMaps.at(i).outs.at(j);
						if (ch.type == "eros/pin")
						{
							eros::pin newpin;
							newpin.stamp = msg.header.stamp;
							newpin.ParentDevice = ch.parentdevice;
							newpin.Function = ch.function;
							newpin.Value = msg.buttons[map.ins.at(0).index];
						}
						else if (ch.type == "std_msgs/Bool")
						{

							bool in_value = msg.buttons[map.ins.at(0).index];
							if (in_value == true)
							{
								map.outs.at(j).value = 1.0;
							}
							else
							{
								map.outs.at(j).value = 0.0;
							}
						}
					}
				}
			}
		}
		TopicMaps.at(i) = map;
	}
	char tempstr[512];
	sprintf(tempstr, "Processed joystick message from topic: %s", topic.c_str());
	diag = update_diagnostic(REMOTE_CONTROL, INFO, NOERROR, std::string(tempstr));
	return diag;
}
std::vector<eros::pin> TopicRemapperNodeProcess::get_outputs_pins()
{
	std::vector<eros::pin> outs;
	for (std::size_t i = 0; i < TopicMaps.size(); i++)
	{
		for (std::size_t j = 0; j < TopicMaps.at(i).outs.size(); j++)
		{
			if (TopicMaps.at(i).outs.at(j).type == "eros/pin")
			{
				eros::pin out;
				OutputChannel ch = TopicMaps.at(i).outs.at(j);
				out.ParentDevice = ch.parentdevice;
				out.ConnectedDevice = ch.topic.substr(1,ch.topic.size());
				out.Function = ch.function;
				out.Value = ch.value;
				out.Name = ch.pinname;
				outs.push_back(out);
			}
		}
	}
	return outs;
}
std::vector<std_msgs::Bool> TopicRemapperNodeProcess::get_outputs_bool()
{
	std::vector<std_msgs::Bool> outs;
	for (std::size_t i = 0; i < TopicMaps.size(); ++i)
	{
		for (std::size_t j = 0; j < TopicMaps.at(i).outs.size(); ++j)
		{
			if (TopicMaps.at(i).outs.at(j).type == "std_msgs/Bool")
			{
				std_msgs::Bool out;
				OutputChannel ch = TopicMaps.at(i).outs.at(j);
				if (fabs(ch.value) > 0.1)
				{
					out.data = true;
				}
				else
				{

					out.data = false;
				}
				outs.push_back(out);
			}
		}
	}
	return outs;
}
std::vector<std_msgs::Float32> TopicRemapperNodeProcess::get_outputs_float32()
{
	std::vector<std_msgs::Float32> outs;
	for (std::size_t i = 0; i < TopicMaps.size(); i++)
	{
		for (std::size_t j = 0; j < TopicMaps.at(i).outs.size(); j++)
		{
			if (TopicMaps.at(i).outs.at(j).type == "std_msgs/Float32")
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
TopicRemapperNodeProcess::OutputModeEnum TopicRemapperNodeProcess::map_outputmode_enum(std::string v)
{
	if (v == "Direct")
	{
		return OutputModeEnum::DIRECT;
	}
	else if (v == "Switch")
	{
		return OutputModeEnum::SWITCH;
	}
	else if (v == "ArcadeMix")
	{
		return OutputModeEnum::ARCADEMIX;
	}
	else
	{
		return OutputModeEnum::UNKNOWN;
	}
}
std::string TopicRemapperNodeProcess::map_outputmode_string(TopicRemapperNodeProcess::OutputModeEnum v)
{
	if (v == TopicRemapperNodeProcess::OutputModeEnum::DIRECT)
	{
		return "Direct";
	}
	else if (v == TopicRemapperNodeProcess::OutputModeEnum::SWITCH)
	{
		return "Switch";
	}
	else if (v == TopicRemapperNodeProcess::OutputModeEnum::ARCADEMIX)
	{
		return "ArcadeMix";
	}
	else
	{
		return "UNKNOWN";
	}
}
TopicRemapperNodeProcess::DrivePerc TopicRemapperNodeProcess::arcade_mix(double throttle_perc,double steer_perc)
{
	DrivePerc d;
	double v =(100.0-fabs(steer_perc)) * (throttle_perc/100.0) + throttle_perc;
	double w= (100.0-fabs(throttle_perc)) * (steer_perc/100.0) + steer_perc;
	d.left = (v+w)/2.0;
	d.right = (v-w)/2.0;
	if(d.left > 100.0) { d.left = 100.0; }
	if(d.left < -100.0) { d.right = -100.0; }
	if(d.right > 100.0) { d.right = 100.0; }
	if(d.right < -100.0) { d.right = -100.0; }
	return d;
}