#include "BaseNodeProcess.h"
icarus_rover_v2::diagnostic BaseNodeProcess::update_baseprocess(double t_dt,double t_ros_time)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	run_time+=t_dt;
	ros_time = t_ros_time;
	diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "Base Process Updated.";
	diagnostic = diag;
	return diag;
}
void BaseNodeProcess::set_mydevice(icarus_rover_v2::device t_device)
{
	mydevice = t_device;
	initialized = true;

}
ros::Time BaseNodeProcess::convert_time(struct timeval t_)
{
	ros::Time t;
	t.sec = t_.tv_sec;
	t.nsec = t_.tv_usec*1000;
	return t;
}
ros::Time BaseNodeProcess::convert_time(double t_)
{
	ros::Time t;
	t.sec = (int64_t)t_;
	double rem = t_-(double)t.sec;
	t.nsec = (int64_t)(rem*1000000.0);
	return t;
}
std::vector<icarus_rover_v2::diagnostic> BaseNodeProcess::run_unittest()
{

	std::vector<icarus_rover_v2::diagnostic> diaglist;
	if (unittest_running == false) {
		unittest_running = true;
		icarus_rover_v2::diagnostic diag = diagnostic;
		bool status = true;
		std::string data;
		std::string cmd =
				"cd ~/catkin_ws && "
				"bash devel/setup.bash && catkin_make run_tests_icarus_rover_v2_gtest_test_"
				+ base_node_name
				+ "_process >/dev/null 2>&1 && "
				"mv /home/robot/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_"
				+ base_node_name
				+ "_process.xml "
				"/home/robot/catkin_ws/build/test_results/icarus_rover_v2/"
				+ base_node_name + "/ >/dev/null 2>&1";
		system(cmd.c_str());
		cmd =
				"cd ~/catkin_ws && bash devel/setup.bash && catkin_test_results build/test_results/icarus_rover_v2/"
				+ base_node_name + "/";
		FILE * stream;

		const int max_buffer = 256;
		char buffer[max_buffer];
		cmd.append(" 2>&1");
		stream = popen(cmd.c_str(), "r");
		if (stream) {
			if (!feof(stream)) {
				if (fgets(buffer, max_buffer, stream) != NULL) {
					data.append(buffer);
				}
				pclose(stream);
			}
		}
		std::vector<std::string> strs;
		std::size_t start = data.find(":");
		data.erase(0, start + 1);
		boost::split(strs, data, boost::is_any_of(",: "),
				boost::token_compress_on);
		if(strs.size() < 6)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[1024];
			sprintf(tempstr,"Unable to process Unit Test Result: %s.",data.c_str());
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			return diaglist;
		}
		int test_count = std::atoi(strs.at(1).c_str());
		int error_count = std::atoi(strs.at(3).c_str());
		int failure_count = std::atoi(strs.at(5).c_str());
		if (test_count == 0)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			diag.Description = "Test Count: 0.";
			diaglist.push_back(diag);
			status = false;
		}
		if (error_count > 0)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[512];
			sprintf(tempstr, "Error Count: %d.", error_count);
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			status = false;
		}
		if (failure_count > 0)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[512];
			sprintf(tempstr, "Failure Count: %d.", failure_count);
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			status = false;
		}
		if (status == true)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = NOTICE;
			diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
			diag.Description = "Unit Test -> PASSED.";
			diaglist.push_back(diag);
		}
		else
		{
			diag.Diagnostic_Type = SOFTWARE;
			uint8_t highest_error = INFO;
			for (std::size_t i = 0; i < diaglist.size(); i++)
			{
				if (diaglist.at(i).Level > highest_error)
				{
					highest_error = diaglist.at(i).Level;
				}
			}
			diag.Level = highest_error;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			diag.Description = "Unit Test -> FAILED.";
			diaglist.push_back(diag);
		}
		unittest_running = false;
	}
	else
	{

		icarus_rover_v2::diagnostic diag = diagnostic;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		diag.Description = "Unit Test -> IS STILL IN PROGRESS.";
		diaglist.push_back(diag);
	}
	return diaglist;
}
icarus_rover_v2::device BaseNodeProcess::convert_fromptr(const icarus_rover_v2::device::ConstPtr& t_ptr)
{
	icarus_rover_v2::device dev;
	dev.Architecture = t_ptr->Architecture;
	dev.BoardCount = t_ptr->BoardCount;
	dev.Capabilities = t_ptr->Capabilities;
	dev.DeviceName = t_ptr->DeviceName;
	dev.DeviceParent = t_ptr->DeviceParent;
	dev.DeviceType = t_ptr->DeviceType;
	dev.HatCount = t_ptr->HatCount;
	dev.ID = t_ptr->ID;
	dev.PartNumber = t_ptr->PartNumber;
	dev.PrimaryIP = t_ptr->PrimaryIP;
	dev.SensorCount = t_ptr->SensorCount;
	dev.ShieldCount = t_ptr->ShieldCount;
	dev.pins = t_ptr->pins;
	return dev;
}
icarus_rover_v2::pin BaseNodeProcess::convert_fromptr(const icarus_rover_v2::pin::ConstPtr& t_ptr)
{
	icarus_rover_v2::pin pin;
	pin.AuxTopic = t_ptr->AuxTopic;
	pin.ConnectedDevice = t_ptr->ConnectedDevice;
	pin.ConnectedSensor = t_ptr->ConnectedSensor;
	pin.DefaultValue = t_ptr->DefaultValue;
	pin.Function = t_ptr->Function;
	pin.MaxValue = t_ptr->MaxValue;
	pin.MinValue = t_ptr->MinValue;
	pin.Name = t_ptr->Name;
	pin.Number = t_ptr->Number;
	pin.ParentDevice = t_ptr->ParentDevice;
	pin.ScaleFactor = t_ptr->ScaleFactor;
	pin.Value = t_ptr->Value;
	pin.stamp = t_ptr->stamp;
	return pin;
}
icarus_rover_v2::command BaseNodeProcess::convert_fromptr(const icarus_rover_v2::command::ConstPtr& t_ptr)
{
	icarus_rover_v2::command cmd;
	cmd.Command = t_ptr->Command;
	cmd.CommandText = t_ptr->CommandText;
	cmd.Description = t_ptr->Description;
	cmd.Option1 = t_ptr->Option1;
	cmd.Option2 = t_ptr->Option2;
	cmd.Option3 = t_ptr->Option3;
	return cmd;
}
icarus_rover_v2::diagnostic BaseNodeProcess::convert_fromptr(const icarus_rover_v2::diagnostic::ConstPtr& t_ptr)
{
	icarus_rover_v2::diagnostic diag;
	diag.Component = t_ptr->Component;
	diag.Description = t_ptr->Description;
	diag.DeviceName = t_ptr->DeviceName;
	diag.Diagnostic_Message = t_ptr->Diagnostic_Message;
	diag.Diagnostic_Type = t_ptr->Diagnostic_Type;
	diag.Level = t_ptr->Level;
	diag.Node_Name = t_ptr->Node_Name;
	diag.SubSystem = t_ptr->SubSystem;
	diag.System = t_ptr->System;
	return diag;
}
icarus_rover_v2::imu BaseNodeProcess::convert_fromptr(const icarus_rover_v2::imu::ConstPtr& t_ptr)
{
	icarus_rover_v2::imu imu;
	imu.header = t_ptr->header;
	imu.tov = t_ptr->tov;
	imu.xacc = t_ptr->xacc;
	imu.yacc = t_ptr->yacc;
	imu.zacc = t_ptr->zacc;
	imu.xgyro = t_ptr->xgyro;
	imu.ygyro = t_ptr->ygyro;
	imu.zgyro = t_ptr->zgyro;
	imu.xmag = t_ptr->xmag;
	imu.ymag = t_ptr->ymag;
	imu.zmag = t_ptr->zmag;
	return imu;
}
