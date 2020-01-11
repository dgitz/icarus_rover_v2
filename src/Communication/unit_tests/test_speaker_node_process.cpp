#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../SpeakerNodeProcess.h"

std::string Node_Name = "/unittest_speaker_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Node_Name;
#define DIAGNOSTIC_TYPE_COUNT 4
void print_diagnostic(uint8_t level,eros::diagnostic diagnostic)
{
	DiagnosticClass diag_helper;
	if(diagnostic.Level >= level)
	{
		printf("Type: %s Message: %s Level: %s Device: %s Desc: %s\n",
			diag_helper.get_DiagTypeString(diagnostic.Diagnostic_Type).c_str(),
			diag_helper.get_DiagMessageString(diagnostic.Diagnostic_Message).c_str(),
			diag_helper.get_DiagLevelString(diagnostic.Level).c_str(),
			diagnostic.DeviceName.c_str(),diagnostic.Description.c_str());
	}
}
SpeakerNodeProcess *initializeprocess()
{
	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	SpeakerNodeProcess *process;
	process = new SpeakerNodeProcess;
	process->initialize("speaker_node", Node_Name, Host_Name, ROVER, ROBOT_CONTROLLER, CONTROLLER_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZING);
	process->set_mydevice(device);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZED);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
SpeakerNodeProcess *readyprocess(SpeakerNodeProcess *process)
{
	eros::diagnostic diag = process->update(0.0, 0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
	process->new_soundplaystatus();
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
TEST(ProcessInitialization,NormalOperation)
{
	SpeakerNodeProcess *process = initializeprocess();
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZED);
	EXPECT_TRUE(process->readytospeak() == false);
	process = readyprocess(process);
}
TEST(ProcessOperation,ReceiveUserMessages)
{
	SpeakerNodeProcess *process = initializeprocess();
	process = readyprocess(process);
	EXPECT_TRUE(process->readytospeak() == false);

	eros::usermessage usermsg1,usermsg2,usermsg3,usermsg4,usermsg0;
	usermsg1.Level = LEVEL1;
	usermsg1.message = "Test_Level1";
	usermsg2.Level = LEVEL2;
	usermsg2.message = "Test_Level2";
	usermsg3.Level = LEVEL3;
	usermsg3.message = "Test_Level3";
	usermsg4.Level = LEVEL4;
	usermsg4.message = "Test_Level4";
	usermsg0.Level = LEVEL4+1;
	usermsg0.message = "Test_BadLevel";
	eros::usermessage::ConstPtr user_msg0(new eros::usermessage(usermsg0));
	eros::usermessage::ConstPtr user_msg1(new eros::usermessage(usermsg1));
	eros::usermessage::ConstPtr user_msg2(new eros::usermessage(usermsg2));
	eros::usermessage::ConstPtr user_msg3(new eros::usermessage(usermsg3));
	eros::usermessage::ConstPtr user_msg4(new eros::usermessage(usermsg4));
	for(int i = 0; i < 10; i++)
	{
		eros::diagnostic diag = process->new_usermessage(user_msg1);
		EXPECT_TRUE(diag.Level < NOTICE);
		EXPECT_TRUE((int)process->get_usermessages(LEVEL1).size() == i+1);

		diag = process->new_usermessage(user_msg2);
		EXPECT_TRUE(diag.Level < NOTICE);
		EXPECT_TRUE((int)process->get_usermessages(LEVEL2).size() == i+1);

		diag = process->new_usermessage(user_msg3);
		EXPECT_TRUE(diag.Level < NOTICE);
		EXPECT_TRUE((int)process->get_usermessages(LEVEL3).size() == i+1);

		diag = process->new_usermessage(user_msg4);
		EXPECT_TRUE(diag.Level < NOTICE);
		EXPECT_TRUE((int)process->get_usermessages(LEVEL4).size() == i+1);

		diag = process->new_usermessage(user_msg0);
		EXPECT_TRUE(diag.Level > NOTICE);
		EXPECT_TRUE((int)process->get_usermessages(LEVEL4+1).size() == 0);
	}
}
TEST(ProcessOperation,SpeechPublish_Level1)
{
	std::vector<std::vector<unsigned char> > tx_buffers;
	SpeakerNodeProcess *process = initializeprocess();
	process = readyprocess(process);
	EXPECT_TRUE(process->readytospeak() == false);
	EXPECT_TRUE(process->isspeaking() == false);
	eros::usermessage usermsg1,usermsg2,usermsg3,usermsg4,usermsg0;
	usermsg1.Level = LEVEL1;
	usermsg1.message = "Test_Level1";
	eros::usermessage::ConstPtr user_msg1(new eros::usermessage(usermsg1));
	eros::diagnostic diag = process->new_usermessage(user_msg1);
	EXPECT_TRUE(diag.Level < NOTICE);
	EXPECT_TRUE(process->get_usermessages(LEVEL1).size() == 1);

	diag = process->update(0.01,0.01);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->readytospeak() == true);
	EXPECT_TRUE(process->isspeaking() == false);
	EXPECT_TRUE(process->get_usermessages(LEVEL1).size() == 0);
	std::string speechout = process->get_speechoutput();
	EXPECT_TRUE(speechout == usermsg1.message);
	diag = process->update(0.01,0.02);
	EXPECT_TRUE(process->readytospeak() == false);
	EXPECT_TRUE(process->isspeaking() == true);

	double time_to_finish = process->get_timeleft_tofinishspeaking();
	double run_time = process->get_runtime();
	double dt = 0.01;
	while(run_time <= time_to_finish)
	{
		run_time += dt;
		diag = process->update(dt,run_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
	}
	diag = process->update(0.1,run_time);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->isspeaking() == false);
}

TEST(ProcessOperation,SpeechPublish_Level1_Level2)
{
	std::string speechout;
	double run_time, time_to_finish;
	SpeakerNodeProcess *process = initializeprocess();
	process = readyprocess(process);
	EXPECT_EQ(process->readytospeak(),false);
	EXPECT_EQ(process->isspeaking(),false);
	eros::usermessage usermsg1,usermsg2,usermsg3,usermsg4,usermsg0;
	usermsg2.Level = LEVEL2;
	usermsg2.message = "Test_Level2";
	usermsg1.Level = LEVEL1;
	usermsg1.message = "Test_Level1";
	eros::usermessage::ConstPtr user_msg1(new eros::usermessage(usermsg1));
	eros::usermessage::ConstPtr user_msg2(new eros::usermessage(usermsg2));
	eros::diagnostic diag = process->new_usermessage(user_msg2);
	diag = process->new_usermessage(user_msg1);
	EXPECT_TRUE(diag.Level < NOTICE);
	EXPECT_EQ(process->get_usermessages(LEVEL1).size(),1);
	EXPECT_EQ(process->get_usermessages(LEVEL2).size(),1);
	diag = process->update(0.01,0.01); //Should speak Level2 Message first
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_EQ(process->readytospeak(),true);
	EXPECT_EQ(process->isspeaking(),false);
	speechout = process->get_speechoutput();
	EXPECT_TRUE(speechout == usermsg2.message);
	diag = process->update(0.01,0.02);
	EXPECT_EQ(process->readytospeak(),false);
	EXPECT_EQ(process->isspeaking(),true);

	time_to_finish = process->get_timeleft_tofinishspeaking() + process->get_runtime();
	run_time = process->get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	double dt = 0.01;
	while(run_time <= time_to_finish)
	{
		run_time += dt;
		diag = process->update(dt,run_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
		
	}
	EXPECT_EQ(process->isspeaking(),false);

	EXPECT_EQ(process->get_usermessages(LEVEL1).size(),1);//Should speak Level1 Message next
	EXPECT_EQ(process->ispaused(),true);
	
	while(process->ispaused() == true)
	{
		diag = process->update(0.01,run_time);//Should be paused for a while
		EXPECT_TRUE(diag.Level <= NOTICE);
	}
	EXPECT_EQ(process->readytospeak(),true);
	EXPECT_EQ(process->isspeaking(),false);
	speechout = process->get_speechoutput();
	EXPECT_TRUE(speechout == usermsg1.message);
	diag = process->update(0.01,run_time);
	EXPECT_EQ(process->readytospeak(),false);
	EXPECT_EQ(process->isspeaking(),true);

	time_to_finish = process->get_timeleft_tofinishspeaking() + process->get_runtime();
	run_time = process->get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	dt = 0.01;
	while(run_time <= time_to_finish)
	{
		run_time += dt;
		diag = process->update(dt,run_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
	}
	EXPECT_EQ(process->isspeaking(),false);
}
TEST(ProcessOperation,SpeechPublish_Level1_Level4)
{
	std::string speechout;
	double run_time, time_to_finish;
	SpeakerNodeProcess *process = initializeprocess();
	process = readyprocess(process);
	EXPECT_EQ(process->readytospeak(),false);
	EXPECT_EQ(process->isspeaking(),false);
	eros::usermessage usermsg1,usermsg2,usermsg3,usermsg4,usermsg0;
	usermsg4.Level = LEVEL4;
	usermsg4.message = "Test_Level4";
	usermsg3.Level = LEVEL3;
	usermsg3.message = "Test_Level3";
	usermsg2.Level = LEVEL2;
	usermsg2.message = "Test_Level2";
	usermsg1.Level = LEVEL1;
	usermsg1.message = "Test_Level1";
	eros::usermessage::ConstPtr user_msg1(new eros::usermessage(usermsg1));
	eros::usermessage::ConstPtr user_msg2(new eros::usermessage(usermsg2));
	eros::usermessage::ConstPtr user_msg3(new eros::usermessage(usermsg3));
	eros::usermessage::ConstPtr user_msg4(new eros::usermessage(usermsg4));
	eros::diagnostic diag = process->new_usermessage(user_msg4);
	diag = process->new_usermessage(user_msg3);
	diag = process->new_usermessage(user_msg2);
	diag = process->new_usermessage(user_msg1);
	EXPECT_TRUE(diag.Level < NOTICE);
	EXPECT_EQ(process->get_usermessages(LEVEL1).size(),1);
	EXPECT_EQ(process->get_usermessages(LEVEL2).size(),1);
	EXPECT_EQ(process->get_usermessages(LEVEL3).size(),1);
	EXPECT_EQ(process->get_usermessages(LEVEL4).size(),1);
	diag = process->update(0.01,0.0); //Should speak Level4 Message first
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_EQ(process->readytospeak(),true);
	EXPECT_EQ(process->isspeaking(),false);
	speechout = process->get_speechoutput();
	EXPECT_TRUE(speechout == usermsg4.message);
	diag = process->update(0.01,.01);
	EXPECT_EQ(process->readytospeak(),false);
	EXPECT_EQ(process->isspeaking(),true);

	time_to_finish = process->get_timeleft_tofinishspeaking() + process->get_runtime();
	run_time = process->get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	double dt = 0.01;
	while(run_time <= time_to_finish)
	{
		run_time += dt;
		diag = process->update(dt,run_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
	}
	EXPECT_EQ(process->isspeaking(),false);

	EXPECT_EQ(process->get_usermessages(LEVEL3).size(),1);//Should speak Level3 Message next
	EXPECT_EQ(process->ispaused(),true);
	while(process->ispaused() == true)
	{
		diag = process->update(dt,run_time);//Should be paused for a while
		EXPECT_TRUE(diag.Level <= NOTICE);
	}
	EXPECT_EQ(process->readytospeak(),true);
	EXPECT_EQ(process->isspeaking(),false);
	speechout = process->get_speechoutput();
	EXPECT_TRUE(speechout == usermsg3.message);
	diag = process->update(0.01,run_time);
	EXPECT_EQ(process->readytospeak(),false);
	EXPECT_EQ(process->isspeaking(),true);

	time_to_finish = process->get_timeleft_tofinishspeaking() + process->get_runtime();
	run_time = process->get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	while(run_time <= time_to_finish)
	{
		run_time += dt;
		diag = process->update(dt,run_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
	}
	EXPECT_EQ(process->isspeaking(),false);

	EXPECT_EQ(process->get_usermessages(LEVEL2).size(),1);//Should speak Level2 Message next
	EXPECT_EQ(process->ispaused(),true);
	while(process->ispaused() == true)
	{
		diag = process->update(0.01,run_time);//Should be paused for a while
		EXPECT_TRUE(diag.Level <= NOTICE);
	}
	EXPECT_EQ(process->readytospeak(),true);
	EXPECT_EQ(process->isspeaking(),false);
	speechout = process->get_speechoutput();
	EXPECT_TRUE(speechout == usermsg2.message);
	diag = process->update(0.01,run_time);
	EXPECT_EQ(process->readytospeak(),false);
	EXPECT_EQ(process->isspeaking(),true);

	time_to_finish = process->get_timeleft_tofinishspeaking() + process->get_runtime();
	run_time = process->get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	while(run_time <= time_to_finish)
	{
		run_time += dt;
		diag = process->update(dt,run_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
	}
	EXPECT_EQ(process->isspeaking(),false);

	EXPECT_EQ(process->get_usermessages(LEVEL1).size(),1);//Should speak Level1 Message next
	EXPECT_EQ(process->ispaused(),true);
	while(process->ispaused() == true)
	{
		diag = process->update(0.01,run_time);//Should be paused for a while
		EXPECT_TRUE(diag.Level <= NOTICE);
	}
	EXPECT_EQ(process->readytospeak(),true);
	EXPECT_EQ(process->isspeaking(),false);
	speechout = process->get_speechoutput();
	EXPECT_TRUE(speechout == usermsg1.message);
	diag = process->update(0.01,run_time);
	EXPECT_EQ(process->readytospeak(),false);
	EXPECT_EQ(process->isspeaking(),true);

	time_to_finish = process->get_timeleft_tofinishspeaking() + process->get_runtime();
	run_time = process->get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	while(run_time <= time_to_finish)
	{
		run_time += dt;
		diag = process->update(dt,run_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
	}
	diag = process->update(0.1,run_time);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_EQ(process->isspeaking(),false);
}
TEST(Template, Process_Command)
{
	SpeakerNodeProcess *process = initializeprocess();
	process = readyprocess(process);
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false;   //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false;   //0.1 Hz
	bool pause_resume_ran = false;
	while (current_time <= time_to_run)
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
		if ((current_time_ms % 10000) == 0)
		{
			slowrate_fire = true;
		}
		else
		{
			slowrate_fire = false;
		}

		eros::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;

		if (fastrate_fire == true) //Nothing to do here
		{
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
			if(pause_resume_ran == false)
			{
				{
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
					eros::command cmd_taskcontrol;
					cmd_taskcontrol.Command = ROVERCOMMAND_TASKCONTROL;
					cmd_taskcontrol.Option1 = SUBSYSTEM_UNKNOWN;
					cmd_taskcontrol.Option2 = TASKSTATE_PAUSE;
					cmd_taskcontrol.CommandText = Node_Name;
					eros::command::ConstPtr cmd_ptr(new eros::command(cmd_taskcontrol));
					std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
					for (std::size_t i = 0; i < diaglist.size(); i++)
					{
						EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					}
					EXPECT_TRUE(diaglist.size() > 0);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_PAUSE);
				}
				{
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_PAUSE);
					eros::command cmd_taskcontrol;
					cmd_taskcontrol.Command = ROVERCOMMAND_TASKCONTROL;
					cmd_taskcontrol.Option1 = SUBSYSTEM_UNKNOWN;
					cmd_taskcontrol.Option2 = TASKSTATE_RUNNING;
					cmd_taskcontrol.CommandText = Node_Name;
					eros::command::ConstPtr cmd_ptr(new eros::command(cmd_taskcontrol));

					std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
					for (std::size_t i = 0; i < diaglist.size(); i++)
					{
						EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					}
					EXPECT_TRUE(diaglist.size() > 0);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
				}
				{
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
					eros::command cmd_taskcontrol;
					cmd_taskcontrol.Command = ROVERCOMMAND_TASKCONTROL;
					cmd_taskcontrol.Option1 = SUBSYSTEM_UNKNOWN;
					cmd_taskcontrol.Option2 = TASKSTATE_RESET;
					cmd_taskcontrol.CommandText = Node_Name;
					eros::command::ConstPtr cmd_ptr(new eros::command(cmd_taskcontrol));

					std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
					for (std::size_t i = 0; i < diaglist.size(); i++)
					{
						EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					}
					EXPECT_TRUE(diaglist.size() > 0);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RESET);
					diag = process->update(0.0, 0.0);
					EXPECT_TRUE(diag.Level <= NOTICE);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
				}
				pause_resume_ran = true;
			}
		}
		current_time += dt;
	}
	EXPECT_TRUE(pause_resume_ran == true);
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
