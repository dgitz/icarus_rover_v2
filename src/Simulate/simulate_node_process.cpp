#include "simulate_node_process.h"
SimulateNodeProcess::SimulateNodeProcess()
{
	action_index = 0;
	time_in_action = 0.0;
	pose_ready = false;
	script_running = false;
}
SimulateNodeProcess::~SimulateNodeProcess()
{

}

icarus_rover_v2::diagnostic SimulateNodeProcess::update(double dt)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::pose temp_pose = pose;
	if(simulation_mode == SCRIPT)
	{
		time_in_action += dt;
		if(time_in_action > current_action.Option1)
		{
			time_in_action = 0.0;
			action_index++;
			if((action_index+1) > actions.size())
			{
				if(script_running == true)
				{
					printf("Actions complete.  Stopping.\n");
				}
				script_running = false;
				current_action.Command = "Stop";
				
			}
			else
			{
				
				current_action = actions.at(action_index);
				printf("Changing to action: [%d/%d]:%s\n",action_index,actions.size()-1,current_action.Comment.c_str());
			}
		}
		
		if(current_action.Command == "Drive")
		{
			steer_command = current_action.Option2/100.0;
			throttle_command = current_action.Option3/100.0;
		}
		else if(current_action.Command == "Stop")
		{
			steer_command = 0.0;
			throttle_command = 0.0;
		}
		else if(current_action.Command == "Wait")
		{
			//No updates, hold last value
		}
	}
	else if(simulation_mode == REMOTECONTROL)
	{
	}
	if(vehicle_model == TANK)
	{
		//printf("v: %f %f\n",throttle_command,steer_command);
		temp_pose.wheelspeed.value = throttle_command*vehicle_params.tirediameter_m/2.0;
		double x_dot = temp_pose.wheelspeed.value * cos(pose.yaw.value);
		double y_dot = temp_pose.wheelspeed.value * sin(pose.yaw.value);
		//double psi_dot = (temp_pose.wheelspeed.value/l_f)*sin(beta);
		//beta = atan((l_f/vehicle_length_m)*tan(steer_command));
		temp_pose.east.value += (x_dot*dt);
		temp_pose.north.value += (y_dot*dt);
		temp_pose.yaw.value += (steer_command*dt);
		temp_pose.yaw.value = fmod(temp_pose.yaw.value + M_PI,2*M_PI);
		if(temp_pose.yaw.value < 0)
		{
			temp_pose.yaw.value += 2*M_PI;
		}
		temp_pose.yaw.value -= M_PI;
		pose_ready = true;

		double a1 = ((1-fabs(steer_command))*throttle_command)+throttle_command;
		double a2 = -1.0*(((1-fabs(throttle_command))*steer_command)+steer_command);
		left_wheelspeed_command = (a1-a2)/2.0;
		right_wheelspeed_command = (a1+a2)/2.0;
		double perfect_left_encoder = (vehicle_params.maxspeed_mps*left_wheelspeed_command)/(M_PI*vehicle_params.tirediameter_m);
		double perfect_right_encoder = (vehicle_params.maxspeed_mps*left_wheelspeed_command)/(M_PI*vehicle_params.tirediameter_m);
		
		left_encoder = perfect_left_encoder+get_rand()*0;
		right_encoder = perfect_right_encoder+get_rand()*0;
		//yaw_rate = 0+temp_pose.yawrate.value+get_rand()*0;


		
		
		pose = temp_pose;
		pose_ready = true;
	}
	pose = temp_pose;
	return diag;
}
double SimulateNodeProcess::get_rand()
{
	int num = rand() % 2000 - 1000;
	return (double)(num)/1000.0;
}
bool SimulateNodeProcess::load_configfiles()
{
	TiXmlDocument miscconfig_doc("/home/robot/config/MiscConfig.xml");
	bool miscconfig_loaded = miscconfig_doc.LoadFile();
	if(miscconfig_loaded == false) { return false; }
	TiXmlElement *l_pRootElement = miscconfig_doc.RootElement();

	if( NULL != l_pRootElement )
	{
		TiXmlElement *l_pVehicleParameters = l_pRootElement->FirstChildElement( "VehicleParameters" );
		if ( NULL != l_pVehicleParameters )
		{
			TiXmlElement *l_pWheelbase = l_pVehicleParameters->FirstChildElement( "Wheelbase" );
			if(NULL != l_pWheelbase)
			{
				vehicle_params.wheelbase_m = std::atof(l_pWheelbase->GetText());
			}
			else
			{
				printf("Wheelbase undefined.\n");
				return false;
			}

			TiXmlElement *l_pTireDiameter = l_pVehicleParameters->FirstChildElement( "TireDiameter" );
			if(NULL != l_pTireDiameter)
			{
				vehicle_params.tirediameter_m = std::atof(l_pTireDiameter->GetText());
			}
			else
			{
				printf("TireDiameter undefind.\n");
				return false;
			}

			TiXmlElement *l_pLength = l_pVehicleParameters->FirstChildElement( "Length" );
			if(NULL != l_pLength)
			{
				vehicle_params.vehiclelength_m = std::atof(l_pLength->GetText());
			}
			else
			{
				printf("Length undefined.\n");
				return false;
			}

			TiXmlElement *l_pMaxSpeed = l_pVehicleParameters->FirstChildElement( "MaxSpeed" );
			if(NULL != l_pMaxSpeed)
			{
				vehicle_params.maxspeed_mps = std::atof(l_pMaxSpeed->GetText());
			}
			else
			{
				printf("MaxSpeed undefined.\n");
				return false;
			}

		}
	}
	return true;
}
icarus_rover_v2::diagnostic SimulateNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
    diagnostic = indiag;
	if(load_configfiles() == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Diagnostic_Message = ERROR;
		char tempstr[512];
		sprintf(tempstr,"Unable to load config files.");
		diagnostic.Description = std::string(tempstr);
		return diagnostic;
	}
	vehicle_model = UNDEFINED;
	
	pose.east.value = 0.0;
    pose.north.value = 0.0;
    pose.elev.value = 0.0;
    pose.wheelspeed.value = 0.0;
	pose.yaw.value = 0.0;
	pose.yawrate.value = 0.0;
	
	left_wheelspeed_command = 0.0;
	right_wheelspeed_command = 0.0;
	left_encoder = 0.0;
	right_encoder = 0.0;
	
    pose.east.status = SIGNALSTATE_INITIALIZING;
    pose.north.status = SIGNALSTATE_INITIALIZING;
    pose.elev.status = SIGNALSTATE_INITIALIZING;
    pose.wheelspeed.status = SIGNALSTATE_INITIALIZING;
    pose.yaw.status = SIGNALSTATE_INITIALIZING;
    pose.yawrate.status = SIGNALSTATE_INITIALIZING;
    
	throttle_command = 0;
    steer_command = 0.1;
	
	return diagnostic;
}
bool SimulateNodeProcess::load_scriptfile(std::string path)
{
	ifstream myfile ( path.c_str() ); 
	std::string line;
	bool read_header = false;
	
	if (myfile.is_open())
	{
		while ( getline (myfile,line) )
		{
			if(read_header == true)
			{
				ScriptAction action;
				std::vector<std::string> strs;
				boost::split(strs,line,boost::is_any_of(","));
				action.Command = strs.at(0);
				action.Option1 = std::atof(strs.at(1).c_str());
				action.Option2 = std::atof(strs.at(2).c_str());
				action.Option3 = std::atof(strs.at(3).c_str());
				action.Option4 = std::atof(strs.at(4).c_str());
				action.Option5 = std::atof(strs.at(5).c_str());
				action.Comment = strs.at(6);
				actions.push_back(action);
			}
			if(read_header == false) { read_header = true; }
		}
		myfile.close();
	}
	else
	{
		return false;
	}
	if(actions.size() == 0) { return false; }
	current_action = actions.at(0);
	script_running = true;
	return true;
}
uint8_t SimulateNodeProcess::map_simulationmode_toint(std::string v)
{
	if(v == "Script") { return SCRIPT; }
	else if(v == "RemoteControl") { return REMOTECONTROL; }
	else{ return UNDEFINED; }
}
