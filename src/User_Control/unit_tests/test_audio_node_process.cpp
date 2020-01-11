#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../AudioNodeProcess.h"
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#define AUDIORECORD_DURATION 2
#define LENGTH_AUDIOFILES_TOKEEP 10.0f

std::string Node_Name = "/unittest_audio_node_process";
std::string Host_Name = "unittest";
std::string storage_directory = "/home/robot/storage/AUDIO/";
std::string archive_directory = "/home/robot/storage/AUDIO/archive/";
std::string ros_DeviceName = Host_Name;
#define DIAGNOSTIC_TYPE_COUNT 5
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
AudioNodeProcess *initializeprocess(bool stereo)
{
	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	AudioNodeProcess *process;
	process = new AudioNodeProcess;
	process->initialize("audio_node", Node_Name, Host_Name, ROVER, ROBOT_CONTROLLER, DIAGNOSTIC_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(SENSORS);
	diagnostic_types.push_back(REMOTE_CONTROL);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZING);
	process->set_mydevice(device);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZED);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	eros::diagnostic diagnostic;
	if (stereo == true)
	{

		eros::device microphone;
		microphone.DeviceName = "MainMicrophone";
		microphone.DeviceType = DEVICETYPE_MICROPHONE;
		microphone.DeviceParent = ros_DeviceName;
		microphone.Capabilities.push_back("stereo");
		eros::device::ConstPtr device_ptr(new eros::device(microphone));
		diagnostic = process->new_devicemsg(device_ptr);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	else
	{
		eros::device left_microphone;
		left_microphone.DeviceName = "LeftMicrophone";
		left_microphone.DeviceType = DEVICETYPE_MICROPHONE;
		left_microphone.DeviceParent = ros_DeviceName;
		left_microphone.Capabilities.push_back("mono");
		eros::device::ConstPtr microphoneleft_ptr(new eros::device(left_microphone));
		diagnostic = process->new_devicemsg(microphoneleft_ptr);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);

		eros::device right_microphone;
		right_microphone.DeviceName = "RightMicrophone";
		right_microphone.DeviceType = DEVICETYPE_MICROPHONE;
		right_microphone.DeviceParent = ros_DeviceName;
		right_microphone.Capabilities.push_back("mono");
		eros::device::ConstPtr microphoneright_ptr(new eros::device(right_microphone));
		diagnostic = process->new_devicemsg(microphoneright_ptr);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}

	{
		eros::device amplifier;
		amplifier.DeviceName = "AudioAmplifier1";
		amplifier.DeviceType = DEVICETYPE_AUDIOAMPLIFIER;
		amplifier.DeviceParent = ros_DeviceName;
		eros::device::ConstPtr device_ptr(new eros::device(amplifier));
		diagnostic = process->new_devicemsg(device_ptr);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	EXPECT_FALSE(process->set_audiostoragedirectory("A Directory that should never exist."));
	EXPECT_TRUE(process->set_audiostoragedirectory(storage_directory));
	process->set_audiorecord_duration(AUDIORECORD_DURATION);
	process->set_totalaudiofiletimetokeep(LENGTH_AUDIOFILES_TOKEEP);
	return process;
}
AudioNodeProcess *readyprocess(AudioNodeProcess *process)
{
	eros::diagnostic diag = process->update(0.0, 0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
	{
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT + process->get_microphone_count() + 1));
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
		}
	}
	return process;
}
TEST(Template, Process_Initialization)
{
	AudioNodeProcess *process = initializeprocess(true);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZED);
}
TEST(Template, AudioStorage_Play)
{
	AudioNodeProcess *process = initializeprocess(false);

	process->new_audioplaytrigger("Robot:Booting", true);
	process = readyprocess(process);
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false;   //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false;   //0.1 Hz
	bool fired_once = false;
	uint8_t armed_state = 0;
	std::string trigger_event1 = "Trigger1";
	EXPECT_FALSE(process->add_audioplayfile(storage_directory + "output/AFileThatShouldNeverExist.mp3", trigger_event1, 1));
	EXPECT_TRUE(process->add_audioplayfile(storage_directory + "output/UnitTestStart.mp3", "UnitTest:Start", 1));
	EXPECT_TRUE(process->add_audioplayfile(storage_directory + "output/UnitTestComplete.mp3", "UnitTest:Complete", 1));
	EXPECT_TRUE(process->add_audioplayfile(storage_directory + "output/ALongFile.mp3", trigger_event1, 1));
	EXPECT_TRUE(process->new_audioplaytrigger("UnitTest:Start", false));
	usleep(2000000);
	
	current_time += 2.0;
	eros::diagnostic diag = process->update(2.0, current_time);
	EXPECT_FALSE(process->new_audioplaytrigger("A Trigger that will never exist", false));
	EXPECT_TRUE(process->new_audioplaytrigger(trigger_event1, false));
	while (current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt,current_time);
		if ((current_time > 5.5) and (fired_once == false))
		{
			EXPECT_TRUE(process->new_audioplaytrigger("ArmedState:Armed", false));
			fired_once = true;
		}
		EXPECT_TRUE(diag.Level <= NOTICE);
		std::string command, filepath;

		int current_time_ms = (int)(current_time * 1000.0);
		if ((current_time_ms % 100) == 0)
		{
			fastrate_fire = true;
		}
		else
		{
			fastrate_fire = false;
		}
		if ((current_time_ms % 3000) == 0)
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
		if (fastrate_fire == true)
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
			if (current_time > 5.0)
			{
				if (armed_state > 6)
				{
					armed_state = 0;
				}
				process->new_armedstatemsg(armed_state);
				armed_state++;
			}
		}
		if (slowrate_fire == true)
		{
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT + process->get_microphone_count() + 1));
				for (std::size_t i = 0; i < diagnostics.size(); ++i)
				{
					EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
				}
			}
		}
		current_time += dt;
		usleep((int)(dt * 1000000.0));
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
	EXPECT_TRUE(process->new_audioplaytrigger("UnitTest:Complete", false));
	usleep(3000000);
}
TEST(Template, AudioStorage_Delete)
{
	return;
	AudioNodeProcess *process = initializeprocess(false);
	process = readyprocess(process);
	double time_to_run = 30.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false;   //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false;   //0.1 Hz
	bool pause_resume_ran = false;
	int audiotrigger_count = 0;
	while (current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt, current_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
		std::string command, filepath;
		if (process->get_audiorecordtrigger(command, filepath))
		{
			audiotrigger_count++;
			printf("[Create] %s\n%s\n", command.c_str(), filepath.c_str());

			//Simulate creating a new file
			char tempstr[256];
			sprintf(tempstr, "exec touch %s", filepath.c_str());
			system(tempstr);
		}
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
		if (fastrate_fire == true)
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
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT + process->get_microphone_count() + 1));
				for (std::size_t i = 0; i < diagnostics.size(); ++i)
				{
					printf("%d %d\n", diagnostics.at(i).Diagnostic_Type, diagnostics.at(i).Level);
					EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
				}
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
		usleep((int)(dt * 1000000.0));
	}
	EXPECT_TRUE(pause_resume_ran == true);
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
	EXPECT_TRUE(audiotrigger_count > 0);

	DIR *dp;
	int audiofile_count = 0;
	struct dirent *ep;
	dp = opendir(process->get_audiostoragedirectory().c_str());

	if (dp != NULL)
	{
		while (ep = readdir(dp))
		{
			audiofile_count++;
		}
		(void)closedir(dp);
	}
	else
	{
		perror("Couldn't open the directory");
	}
	audiofile_count = audiofile_count - 2; //Subtract hidden files, : ".", ".."
	int expected_count = (int)(LENGTH_AUDIOFILES_TOKEEP / (double)AUDIORECORD_DURATION);
	if ((expected_count >= (audiofile_count - 1)) and
		(expected_count <= (audiofile_count + 1)))
	{
		EXPECT_TRUE(true);
	}
	else
	{
		printf("File Count mismatch: %d %d\n", expected_count, audiofile_count);
		EXPECT_TRUE(false);
	}
}

TEST(Template, AudioStorage_Archive)
{
	return;
	AudioNodeProcess *process = initializeprocess(false);
	process = readyprocess(process);
	process->enable_archive(true);
	EXPECT_TRUE(process->set_audioarchivedirectory(archive_directory));
	return;
	char tempstr[512];
	sprintf(tempstr, "exec rm -r %s/*", process->get_audioarchivedirectory().c_str());
	system(tempstr);
	double time_to_run = 30.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false;   //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false;   //0.1 Hz
	int audiotrigger_count = 0;
	while (current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt, current_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
		std::string command, filepath;
		if (process->get_audiorecordtrigger(command, filepath))
		{
			audiotrigger_count++;
			printf("[Create] %s\n%s\n", command.c_str(), filepath.c_str());

			//Simulate creating a new file
			char tempstr[256];
			sprintf(tempstr, "exec touch %s", filepath.c_str());
			system(tempstr);
		}
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
		if (fastrate_fire == true)
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
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() == (DIAGNOSTIC_TYPE_COUNT + process->get_microphone_count() + 1));
				for (std::size_t i = 0; i < diagnostics.size(); ++i)
				{
					printf("%d %d\n", diagnostics.at(i).Diagnostic_Type, diagnostics.at(i).Level);
					EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
				}
			}
		}
		current_time += dt;
		usleep((int)(dt * 1000000.0));
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
	EXPECT_TRUE(audiotrigger_count > 0);

	DIR *dp;
	int audiofile_count = 0;
	struct dirent *ep;
	dp = opendir(process->get_audiostoragedirectory().c_str());

	if (dp != NULL)
	{
		while (ep = readdir(dp))
		{
			audiofile_count++;
		}
		(void)closedir(dp);
	}
	else
	{
		perror("Couldn't open the directory");
	}
	audiofile_count = audiofile_count - 2; //Subtract hidden files, : ".", ".."
	int expected_count = (int)(LENGTH_AUDIOFILES_TOKEEP / (double)AUDIORECORD_DURATION);
	if ((expected_count >= (audiofile_count - 1)) and
		(expected_count <= (audiofile_count + 1)))
	{
		EXPECT_TRUE(true);
	}
	else
	{
		printf("File Count mismatch: %d %d\n", expected_count, audiofile_count);
		EXPECT_TRUE(false);
	}

	unsigned long archivedfile_count = 0;
	dp = opendir(process->get_audioarchivedirectory().c_str());

	if (dp != NULL)
	{
		while (ep = readdir(dp))
		{
			archivedfile_count++;
		}
		(void)closedir(dp);
	}
	else
	{
		perror("Couldn't open the directory");
	}
	archivedfile_count = archivedfile_count - 2;
	EXPECT_TRUE(archivedfile_count == (process->get_numberaudiofiles_removed()));
}
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
