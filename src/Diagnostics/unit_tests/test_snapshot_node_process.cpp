#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include <boost/algorithm/string.hpp>
#include "../SnapshotNodeProcess.h"

std::string Node_Name = "/unittest_snapshot_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
#define DIAGNOSTIC_TYPE_COUNT 3
void print_diagnostic(uint8_t level,eros::diagnostic diagnostic)
{
	if(diagnostic.Level >= level)
	{
		printf("Type: %d Message: %d Level: %d Device: %s Desc: %s\n",diagnostic.Diagnostic_Type,diagnostic.Diagnostic_Message,
			  		diagnostic.Level,diagnostic.DeviceName.c_str(),diagnostic.Description.c_str());
	}
}
std::string exec(const char *cmd, bool wait_for_result)
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
		throw;
	}
	return result;
}

SnapshotNodeProcess *initializeprocess(std::string devicename,std::string mode)
{
	std::string myarchitecture = exec("arch",true);
	boost::trim_right(myarchitecture);
	eros::device device;
	device.DeviceName = devicename;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = myarchitecture;

	SnapshotNodeProcess *process;
	process = new SnapshotNodeProcess;
	process->initialize("snapshot_node", Node_Name, devicename, ROVER, ROBOT_CONTROLLER, CONTROLLER_NODE);
	process->set_config_filepaths("/home/robot/catkin_ws/src/icarus_rover_v2/src/Diagnostics/unit_tests/SnapshotConfig.xml");
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	process->enable_diagnostics(diagnostic_types);
	eros::diagnostic diag = process->setInstanceMode(mode);
	EXPECT_TRUE(diag.Level <= NOTICE);
	diag = process->finish_initialization();

	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
SnapshotNodeProcess *readyprocess(SnapshotNodeProcess *process)
{
	eros::diagnostic diag = process->update(0.0, 0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);
	{
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT);
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
		}
	}
	return process;
}
TEST(Template, Process_Initialization)
{
	SnapshotNodeProcess *process = initializeprocess("TestDevice","SLAVE");
	EXPECT_TRUE(process->is_initialized() == true);
}

TEST(Template, Process_Command_GenerateDeviceSnapshot)
{
	{
		SnapshotNodeProcess *process = initializeprocess("SlaveDevice","SLAVE");
		process = readyprocess(process);
		{
			eros::command cmd;
			cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
			cmd.Option1 = ROVERCOMMAND_SNAPSHOT_CLEARALL;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
		}
		double time_to_run = 20.0;
		double dt = 0.001;
		double current_time = 0.0;
		bool fastrate_fire = false;   //10 Hz
		bool mediumrate_fire = false; //1 Hz
		bool slowrate_fire = false;   //0.1 Hz
		int snapshots_totake = 5;
		int snapshot_index = 0;
		while (snapshot_index < snapshots_totake)
		{
			eros::diagnostic diag = process->update(dt, current_time);
			print_diagnostic(NOTICE,diag);
			EXPECT_TRUE(diag.Level <= NOTICE);

			int current_time_ms = (int)(current_time * 1000.0);
			if ((current_time_ms % 100) == 0)
			{
				fastrate_fire = true;
			}
			else
			{
				fastrate_fire = false;
			}
			if ((current_time_ms % 1000) == 0)
			{
				mediumrate_fire = true;
			}
			else
			{
				mediumrate_fire = false;
			}
			if ((current_time_ms % 20000) == 0)
			{
				slowrate_fire = true;
			}
			else
			{
				slowrate_fire = false;
			}

			eros::command cmd;
			

			if (fastrate_fire == true) //Nothing to do here
			{
				cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
				cmd.Option1 = LEVEL1;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
			}
			if (mediumrate_fire == true)
			{
				cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
				cmd.Option1 = LEVEL2;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
			}
			if (slowrate_fire == true)
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() >= DIAGNOSTIC_TYPE_COUNT);
				for (std::size_t i = 0; i < diagnostics.size(); ++i)
				{
					EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
				}

				cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
				cmd.Option1 = ROVERCOMMAND_SNAPSHOT_NEWSNAPSHOT;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::string snapshot_path,snapshot_name;
				EXPECT_TRUE(process->isDeviceSnapshotComplete(snapshot_path,snapshot_name));
				process->resetDeviceSnapshotState();
				snapshot_index++;
			}
			usleep((int)(dt * 10000.0));
			current_time += dt;
		}
		EXPECT_TRUE(process->get_runtime() >= time_to_run);
	}
	{
		SnapshotNodeProcess *process = initializeprocess("MasterDevice","MASTER");
		process = readyprocess(process);
		{
			eros::command cmd;
			cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
			cmd.Option1 = ROVERCOMMAND_SNAPSHOT_CLEARALL;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
		}
		double time_to_run = 20.0;
		double dt = 0.001;
		double current_time = 0.0;
		bool fastrate_fire = false;   //10 Hz
		bool mediumrate_fire = false; //1 Hz
		bool slowrate_fire = false;   //0.1 Hz
		int snapshots_totake = 5;
		int snapshot_index = 0;
		while (snapshot_index < snapshots_totake)
		{
			eros::diagnostic diag = process->update(dt, current_time);
			EXPECT_TRUE(diag.Level <= NOTICE);

			int current_time_ms = (int)(current_time * 1000.0);
			if ((current_time_ms % 100) == 0)
			{
				fastrate_fire = true;
			}
			else
			{
				fastrate_fire = false;
			}
			if ((current_time_ms % 1000) == 0)
			{
				mediumrate_fire = true;
			}
			else
			{
				mediumrate_fire = false;
			}
			if ((current_time_ms % 20000) == 0)
			{
				slowrate_fire = true;
			}
			else
			{
				slowrate_fire = false;
			}

			eros::command cmd;
			

			if (fastrate_fire == true) //Nothing to do here
			{
				cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
				cmd.Option1 = LEVEL1;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
			}
			if (mediumrate_fire == true)
			{
				cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
				cmd.Option1 = LEVEL2;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
			}
			if (slowrate_fire == true)
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() >= DIAGNOSTIC_TYPE_COUNT);
				for (std::size_t i = 0; i < diagnostics.size(); ++i)
				{
					print_diagnostic(WARN,diagnostics.at(i));
					EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
				}

				cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
				cmd.Option1 = ROVERCOMMAND_SNAPSHOT_NEWSNAPSHOT;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				{
					process->update(SYSTEMSNAPSHOT_TIMEOUT*2.0,current_time+(SYSTEMSNAPSHOT_TIMEOUT*2.0));
				}
				eros::diagnostic snap_diag;
				bool v = process->isSystemSnapshotComplete(snap_diag);
				EXPECT_TRUE(v);
				process->resetDeviceSnapshotState();
				snapshot_index++;
			}
			usleep((int)(dt * 10000.0));
			current_time += dt;
		}
		EXPECT_TRUE(process->get_runtime() >= time_to_run);
	}
}
TEST(Template, Process_Command_GenerateSystemSnapshot)
{
	SnapshotNodeProcess *slave_process = initializeprocess("SlaveDevice","SLAVE");
	slave_process = readyprocess(slave_process);
	SnapshotNodeProcess *master_process = initializeprocess("MasterDevice","MASTER");
	master_process = readyprocess(master_process);
	{
			eros::command cmd;
			cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
			cmd.Option1 = ROVERCOMMAND_SNAPSHOT_CLEARALL;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			{
				std::vector<eros::diagnostic> diaglist = slave_process->new_commandmsg(cmd_ptr);
			}
			{
				std::vector<eros::diagnostic> diaglist = master_process->new_commandmsg(cmd_ptr);
			}
	}

	{
		eros::command cmd;
		cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
		cmd.Option1 = ROVERCOMMAND_SNAPSHOT_NEWSNAPSHOT;
		eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
		{
			std::vector<eros::diagnostic> diaglist = slave_process->new_commandmsg(cmd_ptr);
			for (std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);
			std::string snapshot_path,snapshot_name;
			EXPECT_TRUE(slave_process->isDeviceSnapshotComplete(snapshot_path,snapshot_name));
		}
		{
			std::vector<eros::diagnostic> diaglist = master_process->new_commandmsg(cmd_ptr);
			for (std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);
			std::string snapshot_path,snapshot_name;
			{
				master_process->update(SYSTEMSNAPSHOT_TIMEOUT*2.0,(SYSTEMSNAPSHOT_TIMEOUT*2.0));
			}
			eros::diagnostic snap_diag;
			bool v = master_process->isSystemSnapshotComplete(snap_diag);
			EXPECT_TRUE(v);
		}
	}
	/*
	{
		SnapshotNodeProcess *process = initializeprocess("SLAVE");
		process = readyprocess(process);
		{
			eros::command cmd;
			cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
			cmd.Option1 = ROVERCOMMAND_SNAPSHOT_CLEARALL;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
		}
		double time_to_run = 20.0;
		double dt = 0.001;
		double current_time = 0.0;
		bool fastrate_fire = false;   //10 Hz
		bool mediumrate_fire = false; //1 Hz
		bool slowrate_fire = false;   //0.1 Hz
		int snapshots_totake = 5;
		int snapshot_index = 0;
		while (snapshot_index < snapshots_totake)
		{
			eros::diagnostic diag = process->update(dt, current_time);
			EXPECT_TRUE(diag.Level <= NOTICE);

			int current_time_ms = (int)(current_time * 1000.0);
			if ((current_time_ms % 100) == 0)
			{
				fastrate_fire = true;
			}
			else
			{
				fastrate_fire = false;
			}
			if ((current_time_ms % 1000) == 0)
			{
				mediumrate_fire = true;
			}
			else
			{
				mediumrate_fire = false;
			}
			if ((current_time_ms % 20000) == 0)
			{
				slowrate_fire = true;
			}
			else
			{
				slowrate_fire = false;
			}

			eros::command cmd;
			

			if (fastrate_fire == true) //Nothing to do here
			{
				cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
				cmd.Option1 = LEVEL1;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
			}
			if (mediumrate_fire == true)
			{
				cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
				cmd.Option1 = LEVEL2;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
			}
			if (slowrate_fire == true)
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() >= DIAGNOSTIC_TYPE_COUNT);
				for (std::size_t i = 0; i < diagnostics.size(); ++i)
				{
					EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
				}

				cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
				cmd.Option1 = ROVERCOMMAND_SNAPSHOT_NEWSNAPSHOT;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::string snapshot_path,snapshot_name;
				EXPECT_TRUE(process->isDeviceSnapshotComplete(snapshot_path,snapshot_name));
				snapshot_index++;
			}
			usleep((int)(dt * 10000.0));
			current_time += dt;
		}
		EXPECT_TRUE(process->get_runtime() >= time_to_run);
	}
	{
		SnapshotNodeProcess *process = initializeprocess("MASTER");
		process = readyprocess(process);
		{
			eros::command cmd;
			cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
			cmd.Option1 = ROVERCOMMAND_SNAPSHOT_CLEARALL;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
		}
		double time_to_run = 20.0;
		double dt = 0.001;
		double current_time = 0.0;
		bool fastrate_fire = false;   //10 Hz
		bool mediumrate_fire = false; //1 Hz
		bool slowrate_fire = false;   //0.1 Hz
		int snapshots_totake = 5;
		int snapshot_index = 0;
		while (snapshot_index < snapshots_totake)
		{
			eros::diagnostic diag = process->update(dt, current_time);
			EXPECT_TRUE(diag.Level <= NOTICE);

			int current_time_ms = (int)(current_time * 1000.0);
			if ((current_time_ms % 100) == 0)
			{
				fastrate_fire = true;
			}
			else
			{
				fastrate_fire = false;
			}
			if ((current_time_ms % 1000) == 0)
			{
				mediumrate_fire = true;
			}
			else
			{
				mediumrate_fire = false;
			}
			if ((current_time_ms % 20000) == 0)
			{
				slowrate_fire = true;
			}
			else
			{
				slowrate_fire = false;
			}

			eros::command cmd;
			

			if (fastrate_fire == true) //Nothing to do here
			{
				cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
				cmd.Option1 = LEVEL1;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
			}
			if (mediumrate_fire == true)
			{
				cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
				cmd.Option1 = LEVEL2;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
			}
			if (slowrate_fire == true)
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() >= DIAGNOSTIC_TYPE_COUNT);
				for (std::size_t i = 0; i < diagnostics.size(); ++i)
				{
					EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
				}

				cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
				cmd.Option1 = ROVERCOMMAND_SNAPSHOT_NEWSNAPSHOT;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::string snapshot_path,snapshot_name;
				EXPECT_TRUE(process->isDeviceSnapshotComplete(snapshot_path,snapshot_name));
				snapshot_index++;
			}
			usleep((int)(dt * 10000.0));
			current_time += dt;
		}
		EXPECT_TRUE(process->get_runtime() >= time_to_run);
	}
	*/
}
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
