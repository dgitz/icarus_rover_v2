#include "cameracapture_node.h"
//Start User Code: Firmware Definition
#define CAMERACAPTURENODE_MAJOR_RELEASE 2
#define CAMERACAPTURENODE_MINOR_RELEASE 4
#define CAMERACAPTURENODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
/*! \brief User Loop1 Code
 */
bool run_loop1_code()
{
	if(operation_mode == "capture")
	{
		ros::Time time_a = ros::Time::now();
		capture_image(capture);
		//printf("Capture time: %f\n",measure_time_diff(ros::Time::now(),time_a));
	}
	return true;
}
/*! \brief User Loop2 Code
 */
bool run_loop2_code()
{
 	return true;
}
/*! \brief User Loop3 Code
 */
bool run_loop3_code()
{
 	return true;
}
void Edge_Detect_Threshold_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	logger->log_debug("Got edge detect threshold");
	edge_detect_threshold = msg->data;
}
void ResizeImage_Callback(const sensor_msgs::Image::ConstPtr& msg,const std::string &topicname)
{
	for(int i = 0; i < resample_maps.size();i++)
	{
		if(topicname == resample_maps.at(i).input_topic)
		{
			double width_scale_factor = (double)msg->width/(double)resample_maps.at(i).width;
			double height_scale_factor = (double)msg->height/(double)resample_maps.at(i).height;

			cv_bridge::CvImagePtr cv_ptr;
			if(resample_maps.at(i).encoding == "rgb8")
			{
				cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
			}
			else if(resample_maps.at(i).encoding == "32FC1")
			{
				cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
			}
			else
			{
				char tempstr[255];
				sprintf(tempstr,"Image Encoding: %s Not Supported.",resample_maps.at(i).encoding.c_str());
				logger->log_error(tempstr);
				diagnostic_status.Diagnostic_Type = SOFTWARE;
				diagnostic_status.Level = ERROR;
				diagnostic_status.Diagnostic_Message = SENSORS;
				diagnostic_status.Description = tempstr;
				diagnostic_pub.publish(diagnostic_status);
				return;

			}

			if(width_scale_factor > 1.0)
			{
				cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(resample_maps.at(i).width,resample_maps.at(i).height),0,0,cv::INTER_LINEAR);
			}
			else
			{
				cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(resample_maps.at(i).width,resample_maps.at(i).height),0,0,cv::INTER_AREA);
			}

			char tempstr[1024];
			sprintf(tempstr,"Published resized image to topic: %s",resample_maps.at(i).output_topic.c_str());
			logger->log_debug(tempstr);
			resample_maps.at(i).image_pub.publish(cv_ptr->toImageMsg());
			break;
		}
	}
}
bool capture_image(cv::VideoCapture cap)
{
    ros::Time start = ros::Time::now();
    cv_bridge::CvImage cv_image;
    cv::Mat frame;
    cap >> frame;
    /*cv_image.encoding = "bgr8";
    cv_image.header.stamp = ros::Time::now();
    cv_image.header.frame_id = "/world";
    cv_image.image = frame;
    */
    int bytes = frame.total() * frame.elemSize();
    //if(bytes > 0)
    //{
    	//cv::Mat emptyframe(image_width,image_height,CV_8UC3,cv::Scalar(0,0,0));

    	cv_image.encoding = "bgr8";
    	cv_image.header.stamp = ros::Time::now();
    	cv_image.header.frame_id = "/world";
    	cv_image.image = frame;
		raw_image_pub.publish(cv_image.toImageMsg());

		sensor_msgs::CameraInfo caminfo;
		caminfo.header = cv_image.header;
		caminfo.width = cv_image.image.cols;
		caminfo.height = cv_image.image.rows;
		raw_imageinfo_pub.publish(caminfo);
		//printf("Delay: %f\n",measure_time_diff(ros::Time::now(),start));
    //}
	if(save_images == true)
	{
		time_t rawtime;
		struct tm * timeinfo;
		char datebuffer[80];

		time (&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(datebuffer,80,"%Y_%m_%d_%I_%M_%S",timeinfo);
		char tempstr[256];
		sprintf(tempstr,"%s/%s.jpg",storage_location.c_str(),datebuffer);
		cv::imwrite(tempstr,frame);
		//printf("Saving to: %s\n",tempstr);
	}
    /*
    cv::Mat src_gray;
    cv::cvtColor(frame,src_gray,cv::COLOR_BGR2GRAY);
    cv::Mat edge_image = visionhelper->Detect_Edges(src_gray,edge_detect_threshold);

    cv_bridge::CvImage proc_image;
	proc_image.encoding = "mono8";
	proc_image.header.stamp = ros::Time::now();
	proc_image.header.frame_id = "/world";
	proc_image.image = edge_image;

	proc_image_pub.publish(proc_image.toImageMsg());
	*/
    //Edge_Detection(src_gray,0,0);
    return true;
}
bool Edge_Detection(cv::Mat gray_image,int,void*)
{
	cv::Mat det_edges;
	cv::blur(gray_image,det_edges,cv::Size(3,3));
	cv::Canny(det_edges,det_edges,edge_detect_threshold,edge_detect_threshold*3,3);
	cv_bridge::CvImage proc_image;
	proc_image.encoding = "mono8";
	proc_image.header.stamp = ros::Time::now();
	proc_image.header.frame_id = "/world";
	proc_image.image = det_edges;
	proc_image_pub.publish(proc_image.toImageMsg());
	return true;
}
/*! \brief 0.1 PULSE PER SECOND User Code
 */
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "cameracapture_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 8-April-2017";
	fw.Major_Release = CAMERACAPTURENODE_MAJOR_RELEASE;
	fw.Minor_Release = CAMERACAPTURENODE_MINOR_RELEASE;
	fw.Build_Number = CAMERACAPTURENODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
}
/*! \brief 1.0 PULSE PER SECOND User Code
 */
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	received_pps = true;
    if(device_initialized == true)
	{
		icarus_rover_v2::diagnostic resource_diagnostic = resourcemonitor->update();
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
	else
    {
    	{
			icarus_rover_v2::srv_device srv;
			srv.request.query = "SELF";
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() != 1)
				{
					logger->log_error("Got unexpected device message");
				}
				else
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(0));
				}
			}
    	}
    }
}
/*! \brief Self-Diagnostic-Check Program Variables
 */
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
	//logger->log_info("Got command");
	if (msg->Command ==  DIAGNOSTIC_ID)
	{
		if(msg->Option1 == LEVEL1)
		{
			diagnostic_pub.publish(diagnostic_status);
		}
		else if(msg->Option1 == LEVEL2)
		{
			std::vector<icarus_rover_v2::diagnostic> diaglist = check_program_variables();
			for(int i = 0; i < diaglist.size();i++) { diagnostic_pub.publish(diaglist.at(i)); }
		}
		else if(msg->Option1 == LEVEL3)
		{
		}
		else if(msg->Option1 == LEVEL4)
		{
		}
		else
		{
			logger->log_error("Shouldn't get here!!!");
		}
	}
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
	node_name = "cameracapture_node";
    ros::init(argc, argv, node_name);
    node_name = ros::this_node::getName();
    ros::NodeHandle n;
    if(initialize(n) == false)
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
    now = ros::Time::now();
    last_10Hz_timer = ros::Time::now();
    double mtime;
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
            if(run_loop1 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop1_timer);
                if(mtime >= (1.0/loop1_rate))
                {
                    run_loop1_code();
                    last_loop1_timer = ros::Time::now();
                }
            }
            if(run_loop2 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop2_timer);
                if(mtime >= (1.0/loop2_rate))
                {
                    run_loop2_code();
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
    if(operation_mode == "capture")
    {
    	capture.release();
    }
    logger->log_notice("Node Finished Safely.");
    return 0;
}

bool initialize(ros::NodeHandle nh)
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
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
    diagnostic_status.DeviceName = hostname;
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = VISION_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic_status);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = nh.advertise<icarus_rover_v2::resource>(resource_topic,1000);

    std::string param_verbosity_level = node_name +"/verbosity_level";
    if(nh.getParam(param_verbosity_level,verbosity_level) == false)
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
    heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
    beat.Node_Name = node_name;

    std::string param_startup_delay = node_name + "/startup_delay";
    double startup_delay = 0.0;
    if(n->getParam(param_startup_delay,startup_delay) == false)
    {
    	logger->log_notice("Missing Parameter: startup_delay.  Using Default: 0.0 sec.");
    }
    else
    {
    	char tempstr[128];
    	sprintf(tempstr,"Using Parameter: startup_delay = %4.2f sec.",startup_delay);
    	logger->log_notice(std::string(tempstr));
    }
    printf("[%s] Using Parameter: startup_delay = %4.2f sec.\n",node_name.c_str(),startup_delay);
    ros::Duration(startup_delay).sleep();
    std::string device_topic = "/" + std::string(hostname) + "_master_node/srv_device";
    srv_device = nh.serviceClient<icarus_rover_v2::srv_device>(device_topic);

    //std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    //device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
    pps01_sub = nh.subscribe<std_msgs::Bool>("/01PPS",1000,PPS01_Callback); 
    pps1_sub = nh.subscribe<std_msgs::Bool>("/1PPS",1000,PPS1_Callback); 
    command_sub = nh.subscribe<icarus_rover_v2::command>("/command",1000,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
    
    double max_rate = 0.0;
    std::string param_loop1_rate = node_name + "/loop1_rate";
    if(nh.getParam(param_loop1_rate,loop1_rate) == false)
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
    if(nh.getParam(param_loop2_rate,loop2_rate) == false)
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
    if(nh.getParam(param_loop3_rate,loop3_rate) == false)
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
    save_images = false;
    storage_location = "";
    std::string param_operation_mode = node_name +"/operation_mode";
    if(nh.getParam(param_operation_mode,operation_mode) == false)
    {
    	logger->log_warn("Missing Parameter: operation_mode.");
    	return false;
    }

	if(operation_mode == "capture")
	{
		std::string param_width = node_name + "/image_width";
		if(nh.getParam(param_width,image_width) == false)
		{
			char tempstr[255];
			sprintf(tempstr,"Didn't find %s Exiting.",param_width.c_str());
			logger->log_error(tempstr);
			return false;
		}
		std::string param_height = node_name + "/image_height";
		if(nh.getParam(param_height,image_height) == false)
		{
			char tempstr[255];
			sprintf(tempstr,"Didn't find %s Exiting.",param_height.c_str());
			logger->log_error(tempstr);
			return false;
		}
		std::string folder;
		std::string param_storage_location = node_name +"/storage_location";
		if(nh.getParam(param_storage_location,folder) == false)
		{
			logger->log_notice("Didn't find: storage_location. Not saving images.");
		}
		else
		{
			time_t rawtime;
			struct tm * timeinfo;
			char datebuffer[80];
			time (&rawtime);
			timeinfo = localtime(&rawtime);
			strftime(datebuffer,80,"%Y_%m_%d",timeinfo);
			char tempstr[256];
			sprintf(tempstr,"%s/%s",folder.c_str(),datebuffer);
			struct stat st = {0};

			if (stat(tempstr, &st) == -1)
			{
			    mkdir(tempstr, 0700);
			}
			storage_location = std::string(tempstr);
			save_images = true;
		}
		int video_device;
		std::string param_video_device = node_name +"/video_device";
		if(nh.getParam(param_video_device,video_device) == false)
		{
			logger->log_fatal("Missing Parameter: video_device. Exiting");
			return false;
		}
		std::string rawimage_topic = "/" + node_name + "/raw_image";
		raw_image_pub = nh.advertise<sensor_msgs::Image>(rawimage_topic,1);
		std::string rawimageinfo_topic = "/" + node_name + "/raw_image_info";
		raw_imageinfo_pub = nh.advertise<sensor_msgs::CameraInfo>(rawimageinfo_topic,1);
		//std::string procimage_topic = "/" + node_name + "/proc_image";
		//proc_image_pub = nh.advertise<sensor_msgs::Image>(procimage_topic,1000);
		//counter = 0;
		//edge_detect_threshold = 100;
		//std::string edge_detect_topic = "/" + node_name +"/edge_detect_threshold";
		//edge_threshold_sub = nh.subscribe<std_msgs::UInt8>(edge_detect_topic,1000,Edge_Detect_Threshold_Callback);
		printf("using width: %d height: %d\n",image_width,image_height);
		capture.open(video_device);
		capture.set(CV_CAP_PROP_FRAME_WIDTH, image_width);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT, image_height);
		//printf("Auto exposure: %f\n",capture.get(CV_CAP_PROP_AUTO_EXPOSURE));
		//printf("Exposure: %f\n",capture.get(CV_CAP_PROP_EXPOSURE));
		//printf("Frame rate: %f\n",capture.get(CV_CAP_PROP_FPS));
		//compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //CV_IMWRITE_PNG_COMPRESSION
		//compression_params.push_back(9);  //3
		if(!capture.isOpened())  // check if we succeeded
		{
			logger->log_fatal("Can't initialize camera. Exiting.");
			return false;
		}
		else
		{
		   logger->log_info("Camera working!");
		}
	}
	else if(operation_mode == "resample")
	{

		bool search_for_topics = true;
		int topicindex = 1;
		while(search_for_topics == true)
		{
			std::string topicname;
			std::string imageencoding;
			int width;
			int height;
			bool add_new_topic = false;
			std::string param_topic = node_name +"/image_topic" + boost::lexical_cast<std::string>(topicindex);
			if(nh.getParam(param_topic,topicname) == false)
			{
				char tempstr[255];
				sprintf(tempstr,"Didn't find image_topic at: %s Not adding anymore.",param_topic.c_str());
				logger->log_info(tempstr);
				add_new_topic = false;
				search_for_topics = false;
			}
			else
			{
				add_new_topic = true;
				search_for_topics = true;
			}
			if(add_new_topic == true)
			{
				std::string param_encoding = node_name + "/image_encoding" + boost::lexical_cast<std::string>(topicindex);
				if(nh.getParam(param_encoding,imageencoding) == false)
				{
					char tempstr[255];
					sprintf(tempstr,"Didn't find %s Exiting.",param_encoding.c_str());
					logger->log_error(tempstr);
					return false;
				}
				std::string param_width = node_name + "/image_width" + boost::lexical_cast<std::string>(topicindex);
				if(nh.getParam(param_width,width) == false)
				{
					char tempstr[255];
					sprintf(tempstr,"Didn't find %s Exiting.",param_width.c_str());
					logger->log_error(tempstr);
					return false;
				}
				std::string param_height = node_name + "/image_height" + boost::lexical_cast<std::string>(topicindex);
				if(nh.getParam(param_height,height) == false)
				{
					char tempstr[255];
					sprintf(tempstr,"Didn't find %s Exiting.",param_height.c_str());
					logger->log_error(tempstr);
					return false;
				}
				imageresample_map newmap;
				newmap.input_topic = topicname;
				newmap.encoding = imageencoding;
				newmap.width = width;
				newmap.height = height;
				resample_maps.push_back(newmap);
				topicindex = topicindex +1;
			}
		}
		for(int i = 0; i < resample_maps.size();i++)
		{
			ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>(resample_maps.at(i).input_topic,1000,boost::bind(ResizeImage_Callback,
					_1,resample_maps.at(i).input_topic));
			resample_maps.at(i).image_sub = sub;
			std::string outputimage_topic = resample_maps.at(i).input_topic + "_resized";
			ros::Publisher pub = nh.advertise<sensor_msgs::Image>(outputimage_topic,1000);
			resample_maps.at(i).output_topic = outputimage_topic;
			resample_maps.at(i).image_pub = pub;
			char tempstr[255];
			sprintf(tempstr,"Added Topic Map input topic: %s with encoding: %s and linked to output topic: %s",
					resample_maps.at(i).input_topic.c_str(),
					resample_maps.at(i).encoding.c_str(),
					resample_maps.at(i).output_topic.c_str());
			logger->log_info(tempstr);
		}
	}
	else
	{
		char tempstr[255];
		sprintf(tempstr,"Mode: %s Not Supported.  Exiting.",operation_mode.c_str());
		logger->log_fatal(tempstr);
		return false;
	}
	//End User Code: Initialization, Parameters and Topics
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
bool new_devicemsg(std::string query,icarus_rover_v2::device device)
{

	if(query == "SELF")
	{
		if((device.DeviceName == hostname))
		{
			myDevice = device;
			resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
			process->set_mydevice(device);
			device_initialized = true;
		}
	}

	if((device_initialized == true))
	{
		icarus_rover_v2::diagnostic diag = process->new_devicemsg(device);
		if(process->get_initialized() == true)
		{
		}
	}
	return true;
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
