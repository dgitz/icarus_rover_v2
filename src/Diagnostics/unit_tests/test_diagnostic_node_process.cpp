#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../DiagnosticNodeProcess.h"

std::string Node_Name = "/unittest_diagnostic_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;


DiagnosticNodeProcess* initializeprocess()
{
	eros::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = DIAGNOSTIC_NODE;

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

	DiagnosticNodeProcess *process;
	process = new DiagnosticNodeProcess;
	process->initialize("diagnostic_node",Node_Name,Host_Name);
	process->set_diagnostic(diagnostic);
	process->finish_initialization();
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
DiagnosticNodeProcess* readyprocess(DiagnosticNodeProcess* process)
{
	eros::diagnostic diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);
	std::vector<std::string> topics_to_add;
	topics_to_add.push_back("/Task0/diagnostic");
	topics_to_add.push_back("/Task1/diagnostic");
	topics_to_add.push_back("/Task2/diagnostic");
	topics_to_add.push_back("/Task3/diagnostic");
	topics_to_add.push_back("/Task4/diagnostic");
	for(int i = 0; i < topics_to_add.size(); i++)
	{
		std::string taskname = topics_to_add.at(i).substr(1,topics_to_add.at(i).find("/diagnostic")-1);;
		DiagnosticNodeProcess::Task newTask;
		newTask.Task_Name = taskname;
		newTask.diagnostic_topic = topics_to_add.at(i);
		newTask.heartbeat_topic = "/" + taskname + "/heartbeat";
		newTask.resource_topic = "/" + taskname + "/resource";
		process->add_Task(newTask);
	}
	EXPECT_TRUE(process->get_TaskList().size()==topics_to_add.size());
	return process;
}
void print_lcdmessage(DiagnosticNodeProcess* process)
{
	std::string msg = process->build_lcdmessage();
	int width = process->get_lcdwidth();
	int height = process->get_lcdheight();
	printf("\n\n");
	for(int i = 0; i < height; i++)
	{
		printf("%s\n",msg.substr(i*(width),width).c_str());
	}
	printf("\n\n");

}
TEST(Template,Process_Command)
{

	DiagnosticNodeProcess* process = initializeprocess();
	process->no_connectedlcd();
	process = readyprocess(process);
	double time_to_run = 50.0;
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
		if(fastrate_fire == true)
		{
			std::vector<DiagnosticNodeProcess::Task> tasklist = process->get_TaskList();
			for(std::size_t i = 0; i < tasklist.size(); i++)
			{
				process->new_heartbeatmsg(tasklist.at(i).heartbeat_topic);
			}
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
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
TEST(Template,LCDMessage)
{
	DiagnosticNodeProcess* process = initializeprocess();
	eros::device lcd;
	lcd.DeviceName = "LCD1";
	lcd.DeviceType = "LCD";
	lcd.PartNumber = "617003";
	lcd.DeviceParent = process->get_mydevice().DeviceName;
	eros::device::ConstPtr lcd_ptr(new eros::device(lcd));
	eros::diagnostic diag = process->new_devicemsg(lcd_ptr);
	EXPECT_TRUE(diag.Level <= NOTICE);

	process = readyprocess(process);
	double time_to_run = 100.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	uint8_t armed_state = ARMEDSTATUS_UNDEFINED;

	while(current_time <= time_to_run)
	{
		if(current_time < process->get_worstdiag_timelimit())
		{
			eros::diagnostic bad_diag = process->get_diagnostic();
			bad_diag.Level = ERROR;
			bad_diag.DeviceName = "BigDeviceName";
			bad_diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			bad_diag.Description = "1234567890123456789012345678901234567890";
			eros::diagnostic::ConstPtr diag_ptr(new eros::diagnostic(bad_diag));
			process->new_diagnosticmsg("test1",diag_ptr);
		}
		else if(current_time < 2.0*process->get_worstdiag_timelimit())
		{
			eros::diagnostic bad_diag = process->get_diagnostic();
			bad_diag.Level = ERROR;
			bad_diag.DeviceName = "SmDe";
			bad_diag.Diagnostic_Message = MISSING_HEARTBEATS;
			bad_diag.Description = "1234";
			eros::diagnostic::ConstPtr diag_ptr(new eros::diagnostic(bad_diag));
			process->new_diagnosticmsg("test1",diag_ptr);
		}
		diag = process->update(dt,current_time);
		double battery_level = 100.0*current_time/time_to_run;
		process->set_batterylevel(battery_level);
		double battery_voltage = 12.0*current_time/time_to_run;
		process->set_batteryvoltage(battery_voltage);
		process->new_armedstatemsg(armed_state);
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
		if(fastrate_fire == true)
		{
			printf("%f/%f\n",current_time,time_to_run);
			print_lcdmessage(process);
			std::vector<DiagnosticNodeProcess::Task> tasklist = process->get_TaskList();
			for(std::size_t i = 0; i < tasklist.size(); i++)
			{
				process->new_heartbeatmsg(tasklist.at(i).heartbeat_topic);
			}
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
			armed_state++;
			if(armed_state > ARMEDSTATUS_ARMING)
			{
				armed_state = ARMEDSTATUS_UNDEFINED;
			}
		}
		current_time += dt;
		usleep((int)(dt*10000.0));
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

