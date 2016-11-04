#include "tracking_node.h"
//Start Template Code: Firmware Definition
#define TRACKINGNODE_MAJOR_RELEASE 1
#define TRACKINGNODE_MINOR_RELEASE 1
#define TRACKINGNODE_BUILD_NUMBER 0
//End Template Code: Firmware Definition
//Start User Code: Functions
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
	//logger->log_debug("Running medium rate code.");
	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);
	return true;
}
bool run_slowrate_code()
{
	if(device_initialized == true)
	{
		bool status = resourcemonitor->update();
		if(status == true)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
		else
		{
			logger->log_warn("Couldn't read resources used.");
		}
	}
	//logger->log_debug("Running slow rate code.");

	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "tracking_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 10-Sep-2016";
	fw.Major_Release = TRACKINGNODE_MAJOR_RELEASE;
	fw.Minor_Release = TRACKINGNODE_MINOR_RELEASE;
	fw.Build_Number = TRACKINGNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
}
void Image_Callback(const sensor_msgs::Image::ConstPtr& msg)
{

	ros::Time start_time;
	start_time = ros::Time::now();
	logger->log_debug("Got image.");
	/*
	cv_bridge::CvImagePtr newimage_ptr;
	newimage_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	cv::Mat newimage = newimage_ptr->image;
	std::vector<cv::KeyPoint> keypoints_newimage;
	for(int i = 0; i < target_images.size();i++)
	{
		target_feature_detectors.at(i).detect(newimage,keypoints_newimage);
		cv::Mat descriptors_newimage;
		target_descriptor_extractors.at(i).compute(newimage,keypoints_newimage,descriptors_newimage);
		cv::FlannBasedMatcher matcher;
		std::vector<cv::DMatch> matches;
		matcher.match(descriptors_target_images.at(i),descriptors_newimage,matches);
		double max_dist = 0;
		double min_dist = 10000;
		for( int j = 0; j < descriptors_target_images.at(i).rows; j++ )
		{
			double dist = matches[j].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}
		printf("k1: %d k2: %d Matches: %d\n",keypoints_target_images.at(i).size(),keypoints_newimage.size(),matches.size());
		printf("-- Max dist : %f \n", max_dist );
		printf("-- Min dist : %f \n", min_dist );
		std::vector< cv::DMatch > good_matches;

		for( int j = 0; j < descriptors_target_images.at(i).rows; j++ )
		{
			if( matches[j].distance <= max(2*min_dist, 0.02) )
			{
				good_matches.push_back( matches[j]);
			}
		}


		std::vector<std::vector<cv::DMatch> > matches;
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
		matcher->knnMatch(descriptors_target_images.at(i),descriptors_newimage,matches,500);
		double tresholdDist = 0.25 * sqrt(double(target_images.at(i).size().height*target_images.at(i).size().height + target_images.at(i).size().width*target_images.at(i).size().width));

		std::vector<cv::DMatch > good_matches2;
		good_matches2.reserve(matches.size());
		for (size_t k = 0; k < matches.size(); ++k)
		{
		    for (int j = 0; j < matches[k].size(); j++)
		    {
		        cv::Point2f from = keypoints_target_images.at(i)[matches[k][j].queryIdx].pt;
		        cv::Point2f to = keypoints_newimage[matches[k][j].trainIdx].pt;

		        //calculate local distance for each possible match
		        double dist = sqrt((from.x - to.x) * (from.x - to.x) + (from.y - to.y) * (from.y - to.y));
		        //save as best match if local distance is in specified area and on same height
		        if (dist < tresholdDist && abs(from.y-to.y)<5)
		        {
		            good_matches2.push_back(matches[k][j]);
		            j = matches[k].size();
		        }
		    }
		}

		cv::Mat tracked;
		cv::drawMatches(target_images.at(i),keypoints_target_images.at(i),newimage_ptr->image,keypoints_newimage,
				good_matches,tracked,cv::Scalar::all(-1),cv::Scalar::all(-1),
				vector<char>(),cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		cv_bridge::CvImage tracked_image;
		tracked_image.encoding = "bgr8";
		tracked_image.header.stamp = ros::Time::now();
		tracked_image.header.frame_id = "/world";
		tracked_image.image = tracked;
		tracked_image_pub.publish(tracked_image.toImageMsg());

	}
	*/
	//char tempstr[128];
	//sprintf(tempstr,"Image Callback Duration: %f",measure_time_diff(ros::Time::now(),start_time));
	//logger->log_debug(tempstr);


}
//End User Code: Functions

//Start Template Code: Functions
void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	logger->log_info("Got command");
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
int main(int argc, char **argv)
{
 
	node_name = "tracking_node";


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
        return 0; 
    }
    ros::Rate loop_rate(rate);
    now = ros::Time::now();
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
    while (ros::ok())
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
    return 0;
}

bool initialize(ros::NodeHandle nh)
{
    //Start Template Code: Initialization and Parameters
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = TIMING_NODE;

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
    std::string param_loop_rate = node_name +"/loop_rate";
    if(nh.getParam(param_loop_rate,rate) == false)
    {
        logger->log_warn("Missing Parameter: loop_rate.");
        return false;
    }
	hostname[1023] = '\0';
	gethostname(hostname,1023);
    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);

    pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
    command_sub = nh.subscribe<icarus_rover_v2::command>("/command",1000,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
    //End Template Code: Initialization and Parameters

    //Start User Code: Initialization and Parameters
    visionhelper = new VisionHelper();


    target_images.push_back(cv::imread("/home/robot/config/targets/outlet/outlet1.jpg"));
	//target_images.push_back(cv::imread("/home/robot/config/targets/outlet/outlet2.jpg"));
	//target_images.push_back(cv::imread("/home/robot/config/targets/outlet/outlet3.jpg"));
	//target_images.push_back(cv::imread("/home/robot/config/targets/outlet/outlet4.jpg"));
	std::string trackedimage_topic = "/" + node_name + "/tracked_image";
	tracked_image_pub = nh.advertise<sensor_msgs::Image>(trackedimage_topic,1000);
	minHessian = 100;
	for(int i = 0; i < target_images.size();i++)
	{
		cv::SurfFeatureDetector detector(minHessian);
		std::vector<cv::KeyPoint> keypoints;
		detector.detect(target_images.at(0),keypoints);
		keypoints_target_images.push_back(keypoints);
		target_feature_detectors.push_back(detector);
		cv::SurfDescriptorExtractor extractor;
		cv::Mat descriptor;
		extractor.compute(target_images.at(0),keypoints_target_images.at(0),descriptor);
		target_descriptor_extractors.push_back(extractor);
		descriptors_target_images.push_back(descriptor);
	}
	std::string param_image_to_track_topic = node_name +"/image_to_track_topic";
	if(nh.getParam(param_image_to_track_topic,image_to_track_topic) == false)
	{
		logger->log_fatal("Missing Parameter: image_to_track_topic. Exiting.");
		return false;
	}
	image_sub = nh.subscribe<sensor_msgs::Image>(image_to_track_topic,1000,Image_Callback);
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
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	logger->log_info("Got pps");
	received_pps = true;
}
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;

	if(newdevice.DeviceName == hostname)
	{
		myDevice = newdevice;
		resourcemonitor = new ResourceMonitor(myDevice.Architecture,myDevice.DeviceName,node_name);
		device_initialized = true;
	}
}
std::vector<icarus_rover_v2::diagnostic> check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	bool status = true;
	logger->log_notice("checking program variables.");
	//Start: User Code Diagnostic Checks
	/*if(otherDevices.size() < 0) //SAMPLE
	{
		status = false;
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables: otherDevices.size() <= 0";
		diaglist.push_back(diag);
	}
	*/
	//End: User Code Diagnostic Checks
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
//End Template Code: Functions
