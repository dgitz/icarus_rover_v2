#include "cameracapture_node.h"
//Start Template Code: Firmware Definition
#define CAMERACAPTURENODE_MAJOR_RELEASE 2
#define CAMERACAPTURENODE_MINOR_RELEASE 2
#define CAMERACAPTURENODE_BUILD_NUMBER 2
//End Template Code: Firmware Definition
//Start User Code: Functions
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
    cv_image.encoding = "bgr8";
    cv_image.header.stamp = ros::Time::now();
    cv_image.header.frame_id = "/world";
    cv_image.image = frame;
    int bytes = frame.total() * frame.elemSize();
    if(bytes > 0)
    {
		raw_image_pub.publish(cv_image.toImageMsg());
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
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);

	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);
	//logger->log_debug("Running medium rate code.");
	if(operation_mode == "capture")
	{
		capture_image(capture);
	}
	return true;
}
bool run_slowrate_code()
{
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
	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "cameracapture_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 30-Nov-2016";
	fw.Major_Release = CAMERACAPTURENODE_MAJOR_RELEASE;
	fw.Minor_Release = CAMERACAPTURENODE_MINOR_RELEASE;
	fw.Build_Number = CAMERACAPTURENODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
}

//End User Code: Functions

//Start Initialize Function


//Start Main Loop
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
    ros::Rate loop_rate(rate);
    now = ros::Time::now();
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		now = ros::Time::now();
    		mtime = measure_time_diff(now,fast_timer);
			if(mtime > .02)
			{
				run_fastrate_code();
				fast_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,medium_timer);
			if(mtime > 0.1)
			{
				run_mediumrate_code();
				medium_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,slow_timer);
			if(mtime > 1.0)
			{
				run_slowrate_code();           
				slow_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,veryslow_timer);
			if(mtime > 10.0)
			{
				run_veryslowrate_code();
				veryslow_timer = ros::Time::now();
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
//End Main Loop
bool initialize(ros::NodeHandle nh)
{
    //Start Template Code: Initialization, Parameters and Topics
	kill_node = 0;
	signal(SIGINT,signalinterrupt_handler);
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = nh.advertise<icarus_rover_v2::resource>(resource_topic,1000);

	std::string param_verbosity_level = node_name +"/verbosity_level";
	if(nh.getParam(param_verbosity_level,verbosity_level) == false)
	{
		logger = new Logger("FATAL",ros::this_node::getName());
		logger->log_fatal("Missing Parameter: verbosity_level. Exiting");
		return false;
	}
	else
	{
		logger = new Logger(verbosity_level,ros::this_node::getName());
	}
	std::string param_loop_rate = node_name +"/loop_rate";
	if(nh.getParam(param_loop_rate,rate) == false)
	{
		logger->log_fatal("Missing Parameter: loop_rate.  Exiting.");
		return false;
	}
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	std::string heartbeat_topic = "/" + node_name + "/heartbeat";
	heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
	beat.Node_Name = node_name;
	std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
	device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
	pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
	std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
	if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_fatal("Missing Parameter: require_pps_to_start.  Exiting.");
		return false;
	}
	std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
	//End Template Code: Initialization, Parameters and Topics

	//Start User Code: Initialization, Parameters and Topics
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

	visionhelper = new VisionHelper();

	std::string param_operation_mode = node_name + "/operation_mode";
	if(nh.getParam(param_operation_mode,operation_mode) == false)
	{
		logger->log_fatal("Missing Parameter: operation_mode. Exiting");
		return false;
	}
	if(operation_mode == "capture")
	{
		std::string param_image_width = node_name +"/image_width";
		if(nh.getParam(param_image_width,image_width) == false)
		{
			logger->log_fatal("Missing Parameter: image_width. Exiting");
			return false;
		}
		std::string param_image_height = node_name +"/image_height";
		if(nh.getParam(param_image_height,image_height) == false)
		{
			logger->log_fatal("Missing Parameter: image_height. Exiting");
			return false;
		}

		int video_device;
		std::string param_video_device = node_name +"/video_device";
		if(nh.getParam(param_video_device,video_device) == false)
		{
			logger->log_fatal("Missing Parameter: video_device. Exiting");
			return false;
		}
		std::string rawimage_topic = "/" + node_name + "/raw_image";
		raw_image_pub = nh.advertise<sensor_msgs::Image>(rawimage_topic,1000);
		std::string procimage_topic = "/" + node_name + "/proc_image";
		proc_image_pub = nh.advertise<sensor_msgs::Image>(procimage_topic,1000);
		counter = 0;
		edge_detect_threshold = 100;
		std::string edge_detect_topic = "/" + node_name +"/edge_detect_threshold";
		edge_threshold_sub = nh.subscribe<std_msgs::UInt8>(edge_detect_topic,1000,Edge_Detect_Threshold_Callback);
		capture.open(video_device);
		capture.set(CV_CAP_PROP_FRAME_WIDTH, image_width);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT, image_height);
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //CV_IMWRITE_PNG_COMPRESSION
		compression_params.push_back(3);  //3
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
	//End User Code: Initialization, Parameters and Topics
    logger->log_info("Initialized!");
    return true;
}
//End Initialize Function

//Start Template Code: Functions
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//logger->log_info("Got pps");
	received_pps = true;
}
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;

	if((newdevice.DeviceName == hostname) && (device_initialized == false))
	{
		myDevice = newdevice;
		resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
		device_initialized = true;
	}
}
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
