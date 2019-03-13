#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../TimeSlaveNodeProcess.h"

std::string Node_Name = "/unittest_timeslave_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;

void print_timeserverinfo(std::vector<TimeSlaveNodeProcess::TimeServer> time_servers)
{
	for(std::size_t i = 0; i < time_servers.size(); i++)
	{
		printf("[%d] %s %ld %f Delay: %f Offset: %f Jitter: %f\n",
				(int)i,time_servers.at(i).name.c_str(),time_servers.at(i).update_count,
				time_servers.at(i).updated_time,time_servers.at(i).delay,time_servers.at(i).offset,time_servers.at(i).jitter);
	}
}
std::map<std::string,std::string> initialize_timeservers()
		{
	std::map<std::string,std::string> time_servers;
	time_servers["0.ubuntu.pool.n"] = " +0.ubuntu.pool.n .POOL.          16 p    -   64    0    0.000    0.000   0.000";
	time_servers["dgitzrosmaster"] = "*dgitzrosmaster  187.196.188.141  3 u   63   64    1   2.583    2.912 5.010";
	return time_servers;
		}
TimeSlaveNodeProcess* initializeprocess(std::string server)
{
	eros::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = TIMING_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";

	eros::device device;
	device.DeviceName = diagnostic.DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	TimeSlaveNodeProcess *process;
	process = new TimeSlaveNodeProcess;
	process->initialize("timeslave_node",Node_Name,Host_Name);
	process->set_diagnostic(diagnostic);
	process->finish_initialization();
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_unittestingenabled(true);
	process->set_primary_timeserver(server);
	process->set_ntp_initialized(true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
TimeSlaveNodeProcess* readyprocess(TimeSlaveNodeProcess* process)
{
	eros::diagnostic diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);
	return process;
}
TEST(Template,Process_Initialization)
{
	std::map<std::string,std::string> time_servers = initialize_timeservers();
	std::map<std::string, std::string>::iterator it;
	for(it = time_servers.begin();it != time_servers.end(); ++it)
	{
		TimeSlaveNodeProcess* process = initializeprocess(it->first);
	}
}

TEST(Template,Process_Command)
{
	std::map<std::string,std::string> time_servers = initialize_timeservers();
	std::map<std::string, std::string>::iterator it;
	for(it = time_servers.begin();it != time_servers.end(); ++it)
	{
		TimeSlaveNodeProcess* process = initializeprocess(it->first);
		process = readyprocess(process);
		process->set_exec_result(it->second);
		double time_to_run = 20.0;
		double dt = 0.001;
		double current_time = 0.0;
		bool fastrate_fire = false; //10 Hz
		bool mediumrate_fire = false; //1 Hz
		bool slowrate_fire = false; //0.1 Hz
		while(current_time <= time_to_run)
		{
			eros::diagnostic diag = process->update(dt,current_time);
			EXPECT_TRUE(diag.Level <= NOTICE);
			int current_time_ms = (int)(current_time*1000.0);
			if((current_time_ms % 100) == 0)
			{
				fastrate_fire = true;
			}
			else { fastrate_fire = false; }
			if((current_time_ms % 1000) == 0)
			{
				mediumrate_fire = true;
			}
			else { mediumrate_fire = false; }
			if((current_time_ms % 10000) == 0)
			{
				slowrate_fire = true;
			}
			else { slowrate_fire = false; }

			eros::command cmd;
			cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;

			if(fastrate_fire == true) //Nothing to do here
			{
				cmd.Option1 = LEVEL1;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for(std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);


			}
			if(mediumrate_fire == true)
			{
				cmd.Option1 = LEVEL2;
				eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for(std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
				}
				EXPECT_TRUE(diaglist.size() > 0);

			}
			if(slowrate_fire == true)
			{
				diag = process->update_timeservers();
				EXPECT_TRUE(diag.Level <= NOTICE);
				//Don't run LEVEL3 Test, as this will be called circularly and is only responsible for running this test anyways.
				/*
            cmd.Option1 = LEVEL3;
            std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd);
            for(std::size_t i = 0; i < diaglist.size(); i++)
            {
                EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
            }
             EXPECT_TRUE(diaglist.size() > 0);
				 */
			}
			current_time += dt;
		}
		EXPECT_TRUE(process->get_runtime() >= time_to_run);
		std::vector<TimeSlaveNodeProcess::TimeServer> time_servers = process->get_timeservers();
		eros::timesyncinfo timesyncinfo = process->get_timesyncinfo();
		EXPECT_TRUE(time_servers.size() > 0);
		EXPECT_TRUE(time_servers.size() == timesyncinfo.servers.size());
		EXPECT_TRUE(time_servers.size() == timesyncinfo.delay.size());
		EXPECT_TRUE(time_servers.size() == timesyncinfo.jitter.size());
		EXPECT_TRUE(time_servers.size() == timesyncinfo.offset.size());
		EXPECT_TRUE(time_servers.size() == timesyncinfo.time_updated.size());
		EXPECT_TRUE(time_servers.size() == timesyncinfo.update_count.size());
		print_timeserverinfo(time_servers);
		for(std::size_t i = 0; i < time_servers.size(); i++)
		{
			EXPECT_TRUE(time_servers.at(i).update_count > 0);
		}
	}

}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

