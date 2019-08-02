#include "BaseNodeProcess.h"
eros::diagnostic BaseNodeProcess::update_baseprocess(double t_dt, double t_ros_time)
{
	
	run_time += t_dt;
	ros_time = t_ros_time;
	eros::diagnostic diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Base Process Updated.");
	return diag;
}
void BaseNodeProcess::set_mydevice(eros::device t_device)
{
	mydevice = t_device;
	initialized = true;
}
ros::Time BaseNodeProcess::convert_time(struct timeval t_)
{
	ros::Time t;
	t.sec = t_.tv_sec;
	t.nsec = t_.tv_usec * 1000;
	return t;
}
std::string BaseNodeProcess::convert_time_tostr(double t_)
{
	boost::posix_time::ptime my_posix_time = convert_time(t_).toBoost();
	return boost::posix_time::to_iso_extended_string(my_posix_time);
}
ros::Time BaseNodeProcess::convert_time(double t_)
{
	ros::Time t;
	t.sec = (int64_t)t_;
	double rem = t_ - (double)t.sec;
	t.nsec = (int64_t)(rem * 1000000.0);
	return t;
}
std::vector<eros::diagnostic> BaseNodeProcess::run_unittest()
{

	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	if (unittest_running == false)
	{
		unittest_running = true;
		bool status = true;
		std::string data;
		std::string cmd =
			"cd ~/catkin_ws && "
			"bash devel/setup.bash && catkin_make run_tests_icarus_rover_v2_gtest_test_" +
			base_node_name + "_process >/dev/null 2>&1 && "
							 "mv /home/robot/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_" +
			base_node_name + "_process.xml "
							 "/home/robot/catkin_ws/build/test_results/icarus_rover_v2/" +
			base_node_name + "/ >/dev/null 2>&1";
		//system(cmd.c_str());
		cmd =
			"cd ~/catkin_ws && bash devel/setup.bash && catkin_test_results build/test_results/icarus_rover_v2/" + base_node_name + "/";
		FILE *stream;

		const int max_buffer = 256;
		char buffer[max_buffer];
		cmd.append(" 2>&1");
		stream = popen(cmd.c_str(), "r");
		if (stream)
		{
			if (!feof(stream))
			{
				if (fgets(buffer, max_buffer, stream) != NULL)
				{
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
		if (strs.size() < 6)
		{
			char tempstr[1024];
			sprintf(tempstr, "Unable to process Unit Test Result: %s.", data.c_str());
			diag.Description = std::string(tempstr);
			diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,std::string(tempstr));
			diaglist.push_back(diag);
			return diaglist;
		}
		int test_count = std::atoi(strs.at(1).c_str());
		int error_count = std::atoi(strs.at(3).c_str());
		int failure_count = std::atoi(strs.at(5).c_str());
		if (test_count == 0)
		{
			diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,"Test Count: 0");
			diaglist.push_back(diag);
			status = false;
		}
		if (error_count > 0)
		{
			char tempstr[512];
			sprintf(tempstr, "Error Count: %d.", error_count);
			diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,std::string(tempstr));
			diaglist.push_back(diag);
			status = false;
		}
		if (failure_count > 0)
		{
			char tempstr[512];
			sprintf(tempstr, "Failure Count: %d.", failure_count);
			diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,std::string(tempstr));
			diaglist.push_back(diag);
			status = false;
		}
		if (status == true)
		{
			diag = update_diagnostic(SOFTWARE,NOTICE,DIAGNOSTIC_PASSED,"Unit Test -> PASSED.");
			diaglist.push_back(diag);
		}
		else
		{
			uint8_t highest_error = INFO;
			for (std::size_t i = 0; i < diaglist.size(); i++)
			{
				if (diaglist.at(i).Level > highest_error)
				{
					highest_error = diaglist.at(i).Level;
				}
			}
			diag = update_diagnostic(SOFTWARE,highest_error,DIAGNOSTIC_FAILED,"Unit Test -> FAILED.");
			diaglist.push_back(diag);
		}
		unittest_running = false;
	}
	else
	{
		diag = update_diagnostic(SOFTWARE,WARN,DROPPING_PACKETS,"Unit Test -> IS STILL IN PROGRESS.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
eros::device BaseNodeProcess::convert_fromptr(const eros::device::ConstPtr &t_ptr)
{
	eros::device dev;
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
eros::pin BaseNodeProcess::convert_fromptr(const eros::pin::ConstPtr &t_ptr)
{
	eros::pin pin;
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
eros::command BaseNodeProcess::convert_fromptr(const eros::command::ConstPtr &t_ptr)
{
	eros::command cmd;
	cmd.Command = t_ptr->Command;
	cmd.CommandText = t_ptr->CommandText;
	cmd.Description = t_ptr->Description;
	cmd.Option1 = t_ptr->Option1;
	cmd.Option2 = t_ptr->Option2;
	cmd.Option3 = t_ptr->Option3;
	return cmd;
}
eros::diagnostic BaseNodeProcess::convert_fromptr(const eros::diagnostic::ConstPtr &t_ptr)
{
	eros::diagnostic diag;
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
eros::imu BaseNodeProcess::convert_fromptr(const eros::imu::ConstPtr &t_ptr)
{
	eros::imu imu;
	imu.timestamp = t_ptr->timestamp;
	imu.sequence_number = t_ptr->sequence_number;
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
std::string BaseNodeProcess::exec(const char *cmd, bool wait_for_result)
{
	char buffer[512];
	std::string result = "";
	FILE *pipe = popen(cmd, "r");
	if (wait_for_result == false)
	{
		pclose(pipe);
		return "";
	}
	if (!pipe)
		throw std::runtime_error("popen() failed!");
	try
	{
		while (!feof(pipe))
		{
			if (fgets(buffer, 512, pipe) != NULL)
				result += buffer;
		}
	}
	catch (...)
	{
		pclose(pipe);
		throw std::runtime_error("popen() failed!");
	}
	return result;
}
eros::diagnostic BaseNodeProcess::update_diagnostic(uint8_t diagnostic_type, uint8_t level, uint8_t message, std::string description)
{
	return update_diagnostic(host_name,diagnostic_type,level,message,description);
}
eros::diagnostic BaseNodeProcess::update_diagnostic(eros::diagnostic diag)
{
	return update_diagnostic(diag.DeviceName,diag.Diagnostic_Type,diag.Level,diag.Diagnostic_Message,diag.Description);
}
eros::diagnostic BaseNodeProcess::update_diagnostic(std::string device_name, uint8_t diagnostic_type, uint8_t level, uint8_t message, std::string description)
{
	bool devicetype_found = false;
	bool devicename_found = false;
	eros::diagnostic diag;
	uint8_t insert_index = -1;
	for (std::size_t i = 0; i < diagnostics.size(); ++i)
	{
		if (diagnostic_type == diagnostics.at(i).Diagnostic_Type)
		{
			devicetype_found = true;
			insert_index = i;
			if (diagnostics.at(i).DeviceName == device_name)
			{
				devicename_found = true;
				diag = diagnostics.at(i);
				diag.Level = level;
				diag.Diagnostic_Message = message;
				diag.Description = description;
				diagnostics.at(i) = diag;
			}
		}
	}
	if((devicetype_found == true) and (devicename_found == false))
	{
		diag = root_diagnostic;
		diag.Diagnostic_Type = diagnostic_type;
		diag.DeviceName = device_name;
		diag.Level = level;
		diag.Diagnostic_Message = message;
		diag.Description = description;
		std::vector<eros::diagnostic>::iterator it;
		it = diagnostics.begin();
		diagnostics.insert(it+insert_index,diag);
	}
	if (devicetype_found == true)
	{
		return diag;
	}
	else
	{
		diag = root_diagnostic;
		diag.Diagnostic_Type = diagnostic_type;
		diag.Level = ERROR;
		diag.Diagnostic_Message = UNKNOWN_MESSAGE;
		char tempstr[512];
		sprintf(tempstr, "Unsupported Diagnostic Type: %s.  Did you forget to enable it?", diagnostic_helper.get_DiagTypeString(diagnostic_type).c_str());
		return diag;
	}
}
uint8_t BaseNodeProcess::convert_signaltype(std::string units,double *conversion_factor)
{
	*conversion_factor = 1.0;
	if(units == "m/s^2")
	{
		return SIGNALTYPE_ACCELERATION;
	}
	else if(units == "deg/s")
	{
		return SIGNALTYPE_ROTATION_RATE;
	}
	else if(units == "uT")
	{
		return SIGNALTYPE_MAGNETIC_FIELD;
	}
	else if(units == "C")
	{
		return SIGNALTYPE_TEMPERATURE;
	}
	else if(units == "meter")
	{
		return SIGNALTYPE_DISTANCE;
	}
	else if(units == "inch")
	{
		*conversion_factor = 0.0254;
		return SIGNALTYPE_DISTANCE;
	}
	else if(units == "degree")
	{
		return SIGNALTYPE_ANGLE;
	}
	else
	{
		return SIGNALTYPE_UNDEFINED;
	}
}