#include "hatcontroller_node.h"
//Start User Code: Firmware Definition
#define HATCONTROLLERNODE_MAJOR_RELEASE 1
#define HATCONTROLLERNODE_MINOR_RELEASE 0
#define HATCONTROLLERNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
bool run_loop1_code()
{
	diagnostic_status = process.update(0.01);
    bool process_ready = process.is_ready();

    if(process_ready == true)
    {
    	 if(ServoHats_running == false)
    	 {
    		 std::vector<uint16_t> ServoHats_ids = process.get_servohataddresses();
    		 for(std::size_t i = 0; i < ServoHats_ids.size(); i++)
    		 {
    			 ServoHatDriver servohat;
    			 ServoHats.push_back(servohat);
    			 ServoHats.at(i).init(ServoHats_ids.at(i));
                 logger->log_notice("Initializing Servo Hats.");
    			 diagnostic_status = process.set_servohat_initialized(ServoHats_ids.at(i));
    			 logger->log_diagnostic(diagnostic_status);
    			 if(diagnostic_status.Level > INFO) { diagnostic_pub.publish(diagnostic_status); }
    		 }
    		 ServoHats_running = true;
             logger->log_notice("Servo Hats Initialized.");
    	 }
    	 else
    	 {

    		 std::vector<uint16_t> servohats = process.get_servohataddresses();
    		 for(std::size_t i = 0; i < servohats.size(); i++)
    		 {
    			 if(servohats.at(i) == ServoHats.at(i).get_address())
    			 {
					 std::vector<icarus_rover_v2::pin> pins = process.get_servohatpins(servohats.at(i));
					 for(std::size_t j = 0; j < pins.size(); j++)
					 {
						 ServoHats.at(i).setServoValue(pins.at(j).Number, pins.at(j).Value);
					 }
    			 }
    			 else
    			 {
    				 diagnostic_status.Diagnostic_Type = SOFTWARE;
    				 diagnostic_status.Level = ERROR;
    				 diagnostic_status.Diagnostic_Message = DIAGNOSTIC_FAILED;
    				 char tempstr[512];
    				 sprintf(tempstr,"Trying to Set ServoHat Address: %d but got: %d",servohats.at(i),ServoHats.at(i).get_address());
    				 diagnostic_status.Description = std::string(tempstr);
    				 logger->log_diagnostic(diagnostic_status);
    			 }
    		 }

    	 }
    	 /*
         if(TerminalHat_running == false) !!!THIS IS CAUSING NODE TO FREEZE
         {
             logger->log_notice("Initializing Terminal Hat.");
             TerminalHat.init();
             std::vector<icarus_rover_v2::pin> input_pins;
             std::vector<icarus_rover_v2::pin> output_nonactuator_pins;
             std::vector<icarus_rover_v2::pin> output_actuator_pins;
             input_pins = process.get_terminalhatpins("DigitalInput");
             output_nonactuator_pins = process.get_terminalhatpins("DigitalOutput-NonActuator");
             output_actuator_pins = process.get_terminalhatpins("DigitalOutput");

             for(std::size_t i = 0; i < input_pins.size(); i++)
             {
                 if(TerminalHat.configure_pin(input_pins.at(i).Number,input_pins.at(i).Function) == false)
                 {
                     diagnostic_status.Diagnostic_Type = SOFTWARE;
    				 diagnostic_status.Level = ERROR;
    				 diagnostic_status.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
    				 char tempstr[512];
    				 sprintf(tempstr,"Failed to configure TerminalHat %s pin: %d",input_pins.at(i).Function.c_str(),input_pins.at(i).Number);
    				 diagnostic_status.Description = std::string(tempstr);
    				 logger->log_diagnostic(diagnostic_status);
                     kill_node = 1;
                 }
             }
             for(std::size_t i = 0; i < output_nonactuator_pins.size(); i++)
             {
                 if(TerminalHat.configure_pin(output_nonactuator_pins.at(i).Number,output_nonactuator_pins.at(i).Function) == false)
                 {
                     diagnostic_status.Diagnostic_Type = SOFTWARE;
    				 diagnostic_status.Level = ERROR;
    				 diagnostic_status.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
    				 char tempstr[512];
    				 sprintf(tempstr,"Failed to configure TerminalHat %s pin: %d",output_nonactuator_pins.at(i).Function.c_str(),output_nonactuator_pins.at(i).Number);
    				 diagnostic_status.Description = std::string(tempstr);
    				 logger->log_diagnostic(diagnostic_status);
                     kill_node = 1;
                 }
             }
             for(std::size_t i = 0; i < output_actuator_pins.size(); i++)
             {
                 if(TerminalHat.configure_pin(output_actuator_pins.at(i).Number,output_actuator_pins.at(i).Function) == false)
                 {
                     diagnostic_status.Diagnostic_Type = SOFTWARE;
    				 diagnostic_status.Level = ERROR;
    				 diagnostic_status.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
    				 char tempstr[512];
    				 sprintf(tempstr,"Failed to configure TerminalHat %s pin: %d",output_actuator_pins.at(i).Function.c_str(),output_actuator_pins.at(i).Number);
    				 diagnostic_status.Description = std::string(tempstr);
    				 logger->log_diagnostic(diagnostic_status);
                     kill_node = 1;
                 }
             }
             diagnostic_status = process.set_terminalhat_initialized();
             TerminalHat_running = true;
             logger->log_notice("Terminal Hat Initialized.");
        }
        else
        {
            
            std::vector<icarus_rover_v2::pin> output_nonactuator_pins;
            std::vector<icarus_rover_v2::pin> output_actuator_pins;
            output_nonactuator_pins = process.get_terminalhatpins("DigitalOutput-NonActuator");
            output_actuator_pins = process.get_terminalhatpins("DigitalOutput");
            for(std::size_t i = 0; i < output_nonactuator_pins.size(); i++)
            {
                if(TerminalHat.set_pin(output_nonactuator_pins.at(i).Number,output_nonactuator_pins.at(i).Value) == false)
                {
                    diagnostic_status.Diagnostic_Type = SOFTWARE;
    				diagnostic_status.Level = ERROR;
    				diagnostic_status.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
    				char tempstr[512];
    				sprintf(tempstr,"Failed to set TerminalHat %s pin: %d",output_nonactuator_pins.at(i).Function.c_str(),output_nonactuator_pins.at(i).Number);
    				diagnostic_status.Description = std::string(tempstr);
    				logger->log_diagnostic(diagnostic_status);
                }
            }
            for(std::size_t i = 0; i < output_actuator_pins.size(); i++)
            {
                if(TerminalHat.set_pin(output_actuator_pins.at(i).Number,output_actuator_pins.at(i).Value) == false)
                {
                    diagnostic_status.Diagnostic_Type = SOFTWARE;
    				diagnostic_status.Level = ERROR;
    				diagnostic_status.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
    				char tempstr[512];
    				sprintf(tempstr,"Failed to set TerminalHat %s pin: %d",output_actuator_pins.at(i).Function.c_str(),output_actuator_pins.at(i).Number);
    				diagnostic_status.Description = std::string(tempstr);
    				logger->log_diagnostic(diagnostic_status);
                }
            }
            
            std::vector<icarus_rover_v2::pin> input_pins;
            input_pins = process.get_terminalhatpins("DigitalInput");
            for(std::size_t i = 0; i < input_pins.size(); i++)
            {
                input_pins.at(i).Value = TerminalHat.read_pin(input_pins.at(i).Number);
                diagnostic_status = process.new_pinmsg(input_pins.at(i));
                logger->log_diagnostic(diagnostic_status);
                if(diagnostic_status.Level > INFO) {   diagnostic_pub.publish(diagnostic_status); }
                digitalinput_pub.publish(input_pins.at(i));
            }

        }
        */
         
    }
    if(diagnostic_status.Level > INFO)
    {
    	diagnostic_pub.publish(diagnostic_status);
    	logger->log_diagnostic(diagnostic_status);
    }
	return true;
}
bool run_loop2_code()
{
    bool ready_to_arm = process.get_ready_to_arm();
	std_msgs::Bool bool_ready_to_arm;
    bool_ready_to_arm.data = ready_to_arm;
    ready_to_arm_pub.publish(bool_ready_to_arm);
 	return true;
}
bool run_loop3_code()
{
 	return true;
}
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	diagnostic_status = process.new_armedstatemsg((uint8_t)msg->data);
	if(diagnostic_status.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic_status);
	}
}
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "hatcontroller_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 29-May-2017";
	fw.Major_Release = HATCONTROLLERNODE_MAJOR_RELEASE;
	fw.Minor_Release = HATCONTROLLERNODE_MINOR_RELEASE;
	fw.Build_Number = HATCONTROLLERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
}
void DigitalOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg)
{
	double dt = measure_time_diff(ros::Time::now(),last_digitaloutput_time);
	//if(dt < .05) { return; } //Only update at 20 Hz
	last_digitaloutput_time = ros::Time::now();
	icarus_rover_v2::pin pinmsg;
	pinmsg.ParentDevice = msg->ParentDevice;
	pinmsg.Number = msg->Number;
	pinmsg.Function = msg->Function;
	pinmsg.DefaultValue = msg->DefaultValue;
	pinmsg.ConnectedDevice = msg->ConnectedDevice;
	pinmsg.Value = msg->Value;
	//diagnostic_status = process.new_pinmsg(pinmsg);
    logger->log_diagnostic(diagnostic_status);
    if(diagnostic_status.Level > INFO) {   diagnostic_pub.publish(diagnostic_status); }
    
}
void PwmOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg)
{
	//ros::Time now = ros::Time::now();

	//double dt = measure_time_diff(now,last_pwmoutput_sub_time);
	//if(dt < .05) { return; } //Only update at 20 Hz
	//last_pwmoutput_sub_time = ros::Time::now();

	icarus_rover_v2::pin pinmsg;
    pinmsg.stamp = msg->stamp;
	pinmsg.ParentDevice = msg->ParentDevice;
	pinmsg.Number = msg->Number;
	pinmsg.Function = msg->Function;
	pinmsg.DefaultValue = msg->DefaultValue;
	pinmsg.ConnectedDevice = msg->ConnectedDevice;
	pinmsg.Value = msg->Value;
	diagnostic_status = process.new_pinmsg(pinmsg);
    logger->log_diagnostic(diagnostic_status);
    if(diagnostic_status.Level > INFO) {   diagnostic_pub.publish(diagnostic_status); }
	
}
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	if(process.get_analyzetiming() == true)
	{
		char tempstr[512];
		sprintf(tempstr,"Average Time delay for pin update: %f (sec)",process.get_timedelay());
		logger->log_info(std::string(tempstr));
	}
	received_pps = true;
    std_msgs::Bool pps;
    pps.data = msg->data;
    diagnostic_status = process.new_ppsmsg(pps);
    if(diagnostic_status.Level > NOTICE) {   diagnostic_pub.publish(diagnostic_status); }
    if(device_initialized == true)
	{
		icarus_rover_v2::diagnostic resource_diagnostic;
		resource_diagnostic = resourcemonitor->update();
		if(resource_diagnostic.Diagnostic_Message == DEVICE_NOT_AVAILABLE)
		{
			diagnostic_pub.publish(resource_diagnostic);
			logger->log_warn("Couldn't read resources used.");
		}
		else if(resource_diagnostic.Level >= WARN)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
			diagnostic_pub.publish(resource_diagnostic);
		}
		else if(resource_diagnostic.Level <= NOTICE)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
	}
}
std::vector<icarus_rover_v2::diagnostic> check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	bool status = true;
	logger->log_notice("checking program variables.");

	if(status == true)
	{
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = NOTICE;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED";
		diaglist.push_back(diag);
	}
	else
	{
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED";
		diaglist.push_back(diag);
	}
	return diaglist;
}

void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	icarus_rover_v2::command newcommand;
	newcommand.Command = msg->Command;
	newcommand.Option1 = msg->Option1;
	newcommand.Option2 = msg->Option2;
	newcommand.Option3 = msg->Option3;
	newcommand.CommandText = msg->CommandText;
	newcommand.Description = msg->Description;
	diagnostic_status = process.new_commandmsg(newcommand);
    logger->log_diagnostic(diagnostic_status);
    if(diagnostic_status.Level > INFO) {   diagnostic_pub.publish(diagnostic_status); }
	
}
//End User Code: Functions
bool run_10Hz_code()
{
    beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
    if(diagnostic_status.Level > NOTICE)
    {
        diagnostic_pub.publish(diagnostic_status);
    }
    return true;
}
int main(int argc, char **argv)
{
	node_name = "hatcontroller_node";
    ros::init(argc, argv, node_name);
    n.reset(new ros::NodeHandle);
    node_name = ros::this_node::getName();
    

    if(initializenode() == false)
    {
        logger->log_fatal("Unable to Initialize.  Exiting.");
    	diagnostic_status.Diagnostic_Type = SOFTWARE;
		diagnostic_status.Level = FATAL;
		diagnostic_status.Diagnostic_Message = INITIALIZING_ERROR;
		diagnostic_status.Description = "Node Initializing Error.";
		diagnostic_pub.publish(diagnostic_status);
		kill_node = 1;
    }
    ros::Rate loop_rate(ros_rate);
	boot_time = ros::Time::now();
    last_10Hz_timer = ros::Time::now();
    double mtime;
    double loop1_t, loop2_t = 0.0;
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
            if(run_loop1 == true)
            {
            	ros::Time start = ros::Time::now();
                mtime = measure_time_diff(ros::Time::now(),last_loop1_timer);
                if(mtime >= (1.0/loop1_rate))
                {
                    run_loop1_code();
                    loop1_t = measure_time_diff(ros::Time::now(),start);
                    last_loop1_timer = ros::Time::now();
                }

            }
            if(run_loop2 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop2_timer);
                ros::Time start = ros::Time::now();
                if(mtime >= (1.0/loop2_rate))
                {
                    run_loop2_code();
                    loop2_t = measure_time_diff(ros::Time::now(),start);
                    last_loop2_timer = ros::Time::now();
                }

            }
            if(run_loop3 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop3_timer);
                if(mtime >= (1.0/loop3_rate))
                {
                    run_loop3_code();
                    last_loop3_timer = ros::Time::now();
                }
            }
            
            mtime = measure_time_diff(ros::Time::now(),last_10Hz_timer);
            if(mtime >= 0.1)
            {
            	//printf("beat: %f %f %f\n",mtime,loop1_t,loop2_t);
                run_10Hz_code();
                last_10Hz_timer = ros::Time::now();
            }
    	}
		else
		{
			logger->log_warn("Waiting on PPS to Start.");
		}
		ros::spinOnce();
		loop_rate.sleep();
    }
    kill_node = true;
    sleep(2.0);
    logger->log_notice("Node Finished Safely.");
    return 0;
}

bool initializenode()
{
    //Start Template Code: Initialization, Parameters and Topics
	kill_node = 0;
	signal(SIGINT,signalinterrupt_handler);
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    hostname[1023] = '\0';
    gethostname(hostname,1023);
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  n->advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
    diagnostic_status.DeviceName = hostname;
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = GPIO_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing2";
	diagnostic_pub.publish(diagnostic_status);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = n->advertise<icarus_rover_v2::resource>(resource_topic,1000);

    std::string param_verbosity_level = node_name +"/verbosity_level";
    if(n->getParam(param_verbosity_level,verbosity_level) == false)
    {
        logger = new Logger("WARN",ros::this_node::getName());
        logger->log_warn("Missing Parameter: verbosity_level");
        return false;
    }
    else
    {
        logger = new Logger(verbosity_level,ros::this_node::getName());      
    }
    
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = n->advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1);
    beat.Node_Name = node_name;
    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    device_sub = n->subscribe<icarus_rover_v2::device>(device_topic,1,Device_Callback);

    pps01_sub = n->subscribe<std_msgs::Bool>("/01PPS",1000,PPS01_Callback);
    pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1000,PPS1_Callback);  
    command_sub = n->subscribe<icarus_rover_v2::command>("/command",1,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(n->getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  n->advertise<icarus_rover_v2::firmware>(firmware_topic,1);

	double max_rate = 0.0;
    std::string param_loop1_rate = node_name + "/loop1_rate";
    if(n->getParam(param_loop1_rate,loop1_rate) == false)
    {
        logger->log_warn("Missing parameter: loop1_rate.  Not running loop1 code.");
        run_loop1 = false;
    }
    else 
    { 
        last_loop1_timer = ros::Time::now();
        run_loop1 = true; 
        if(loop1_rate > max_rate) { max_rate = loop1_rate; }
    }
    
    std::string param_loop2_rate = node_name + "/loop2_rate";
    if(n->getParam(param_loop2_rate,loop2_rate) == false)
    {
        logger->log_warn("Missing parameter: loop2_rate.  Not running loop2 code.");
        run_loop2 = false;
    }
    else 
    { 
        last_loop2_timer = ros::Time::now();
        run_loop2 = true; 
        if(loop2_rate > max_rate) { max_rate = loop2_rate; }
    }
    
    std::string param_loop3_rate = node_name + "/loop3_rate";
    if(n->getParam(param_loop3_rate,loop3_rate) == false)
    {
        logger->log_warn("Missing parameter: loop3_rate.  Not running loop3 code.");
        run_loop3 = false;
    }
    else 
    { 
        last_loop3_timer = ros::Time::now();
        run_loop3 = true; 
        if(loop3_rate > max_rate) { max_rate = loop3_rate; }
    }
    ros_rate = max_rate * 50.0;
    if(ros_rate < 100.0) { ros_rate = 100.0; }
    char tempstr[512];
    sprintf(tempstr,"Running Node at Rate: %f",ros_rate);
    logger->log_notice(std::string(tempstr));
    //End Template Code: Initialization and Parameters

    //Start User Code: Initialization and Parameters
    ready_to_arm  = false;
    std::string sensor_spec_path;
    std::string armed_state_topic = "/armed_state";
    armed_state_sub = n->subscribe<std_msgs::UInt8>(armed_state_topic,1,ArmedState_Callback);

    //process = new BoardControllerNodeProcess;
	//diagnostic_status = process->init(diagnostic_status,logger,std::string(hostname),sensor_spec_path,extrapolate);
    bool analyze_timing;
    process.init(hostname,diagnostic_status);
    std::string param_analyze_timing = node_name + "/analyze_timing";
    if(n->getParam(param_analyze_timing,analyze_timing) == false)
    {
        logger->log_notice("Missing parameter: analyze_timing.  Not analyzing timing.");
    }
    else 
    {
        if(analyze_timing == true)
        {
            logger->log_notice("Analyzing Timing.");
        }
        process.set_analyzetiming(analyze_timing);
    }
    last_message_received_time = ros::Time::now();
    std::string digitalinput_topic = "/" + node_name + "/DigitalInput";
    digitalinput_pub = n->advertise<icarus_rover_v2::pin>(digitalinput_topic,1);
    std::string analoginput_topic = "/" + node_name + "/AnalogInput";
	analoginput_pub = n->advertise<icarus_rover_v2::pin>(analoginput_topic,1);
	std::string forcesensorinput_topic = "/" + node_name + "/ForceSensorInput";
	forcesensorinput_pub = n->advertise<icarus_rover_v2::pin>(forcesensorinput_topic,1);
	std::string digitaloutput_topic = "/" + node_name + "/DigitalOutput";
	digitaloutput_sub = n->subscribe<icarus_rover_v2::pin>(digitaloutput_topic,5,DigitalOutput_Callback);
	last_digitaloutput_time = ros::Time::now();
	std::string pwmoutput_topic = "/" + node_name + "/PWMOutput";
	pwmoutput_sub = n->subscribe<icarus_rover_v2::pin>(pwmoutput_topic,5,PwmOutput_Callback);
	last_pwmoutput_sub_time = ros::Time::now();


	std::string ready_to_arm_topic = node_name + "/ready_to_arm";
	ready_to_arm_pub = n->advertise<std_msgs::Bool>(ready_to_arm_topic,1);

	ServoHats_running = false;
	ServoHats.clear();
    
    TerminalHat_running = false;
    
    //Finish User Code: Initialization and Parameters

    //Start Template Code: Final Initialization.
	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Initialized";
	diagnostic_pub.publish(diagnostic_status);
    logger->log_info("Initialized!");
    return true;
    //End Template Code: Finish Initialization.
}
//Start Template Code: Functions
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
    if(process.is_ready() == false)
    {
		icarus_rover_v2::device newdevice;
		newdevice.Architecture = msg->Architecture;
		newdevice.BoardCount = msg->BoardCount;
		newdevice.Capabilities = msg->Capabilities;
		newdevice.DeviceName = msg->DeviceName;
		newdevice.DeviceParent = msg->DeviceParent;
		newdevice.DeviceType = msg->DeviceType;
		newdevice.ID = msg->ID;
		newdevice.SensorCount = msg->SensorCount;
		newdevice.ShieldCount = msg->ShieldCount;
		newdevice.pins = msg->pins;
		diagnostic_status = process.new_devicemsg(newdevice);
        logger->log_diagnostic(diagnostic_status);
        if(diagnostic_status.Level > INFO) { diagnostic_pub.publish(diagnostic_status); }
	}
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
