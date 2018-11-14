#include "sample_node_process.h"
/*! \brief Constructor
 */
SampleNodeProcess::SampleNodeProcess(std::string _base_node_name,std::string _node_name)
{
	base_node_name = _base_node_name;
	node_name = _node_name;
	unittest_running = false;
	run_time = 0.0;
	initialized = false;
	ready = false;
}
/*! \brief Deconstructor
 */
SampleNodeProcess::~SampleNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic SampleNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic SampleNodeProcess::update(double dt)
{
	run_time += dt;
	if((mydevice.BoardCount == 0) and (mydevice.SensorCount == 0))
	{
		if(initialized == true) { ready = true; }
	}
	icarus_rover_v2::diagnostic diag = diagnostic;
	diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "Node Running";
	diagnostic = diag;
	return diag;
}
/*! \brief Setup Process Device info
 */
icarus_rover_v2::diagnostic SampleNodeProcess::new_devicemsg(icarus_rover_v2::device device)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	bool new_device = true;
	if(device.DeviceName == myhostname)
	{

	}
	return diag;
}
/*! \brief Process Command Message
 */
std::vector<icarus_rover_v2::diagnostic> SampleNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
	if (cmd.Command ==  ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		if(cmd.Option1 == LEVEL1)
		{
		}
		else if(cmd.Option1 == LEVEL2)
		{
			diaglist = check_program_variables();
			return diaglist;
		}
		else if(cmd.Option1 == LEVEL3)
		{
			diaglist = run_unittest();
			return diaglist;
		}
		else if(cmd.Option1 == LEVEL4)
		{
		}
	}
	return diaglist;
}
/*! \brief Self-Diagnostic-Check Program Variables
 */
std::vector<icarus_rover_v2::diagnostic> SampleNodeProcess::check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag=diagnostic;
	bool status = true;

	if(status == true)
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED";
		diaglist.push_back(diag);
	}
	else
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED";
		diaglist.push_back(diag);
	}
	return diaglist;
}
/*! \brief Self-Diagnostic-Check Program Variables
 */
std::vector<icarus_rover_v2::diagnostic> SampleNodeProcess::run_unittest()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	if(unittest_running == false)
	{
		unittest_running = true;
		icarus_rover_v2::diagnostic diag=diagnostic;
		bool status = true;
		std::string data;
		std::string cmd = "cd ~/catkin_ws && "
				"bash devel/setup.bash && catkin_make run_tests_icarus_rover_v2_gtest_test_" + base_node_name + "_process >/dev/null 2>&1 && "
				"mv /home/robot/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_" + base_node_name + "_process.xml "
				"/home/robot/catkin_ws/build/test_results/icarus_rover_v2/" + base_node_name + "/ >/dev/null 2>&1";
		system(cmd.c_str());
		cmd = "cd ~/catkin_ws && bash devel/setup.bash && catkin_test_results build/test_results/icarus_rover_v2/" + base_node_name + "/";
		FILE * stream;

		const int max_buffer = 256;
		char buffer[max_buffer];
		cmd.append(" 2>&1");
		stream = popen(cmd.c_str(), "r");
		if (stream)
		{
			while (!feof(stream))
			{
				if (fgets(buffer, max_buffer, stream) != NULL) { data.append(buffer); }
				pclose(stream);
			}
		}
		std::vector<std::string> strs;
		std::size_t start = data.find(":");
		data.erase(0,start+1);
		boost::split(strs,data,boost::is_any_of(",: "),boost::token_compress_on);

		int test_count = std::atoi(strs.at(1).c_str());
		int error_count = std::atoi(strs.at(3).c_str());
		int failure_count = std::atoi(strs.at(5).c_str());
		int skipped_count = std::atoi(strs.at(7).c_str());
		if(test_count == 0)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			diag.Description = "Test Count: 0.";
			diaglist.push_back(diag);
			status = false;
		}
		if(error_count > 0)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[512];
			sprintf(tempstr,"Error Count: %d",error_count);
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			status = false;
		}
		if(failure_count > 0)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[512];
			sprintf(tempstr,"Failure Count: %d",failure_count);
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			status = false;
		}
		if(skipped_count > 0)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = WARN;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[512];
			sprintf(tempstr,"Skipped Count: %d",skipped_count);
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			status = false;
		}


		if(status == true)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = INFO;
			diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
			diag.Description = "Unit Test -> PASSED";
			diaglist.push_back(diag);
		}
		else
		{
			diag.Diagnostic_Type = SOFTWARE;
			uint8_t highest_error = INFO;
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				if(diaglist.at(i).Level > highest_error)
				{
					highest_error = diaglist.at(i).Level;
				}
			}
			diag.Level = highest_error;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			diag.Description = "Unit Test -> FAILED";
			diaglist.push_back(diag);
		}

		unittest_running = false;
	}
	else
	{

		icarus_rover_v2::diagnostic diag=diagnostic;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		diag.Description = "Unit Test -> IS STILL IN PROGRESS";
		diaglist.push_back(diag);
	}
	return diaglist;
}
