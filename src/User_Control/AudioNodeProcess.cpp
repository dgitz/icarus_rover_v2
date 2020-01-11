#include "AudioNodeProcess.h"
eros::diagnostic AudioNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	reset();
	query_for_device_configuration = true;
	audiorecord_timer = 0.0;
	totalaudio_tokeep = 30.0;
	number_files_removed = 0;
	left_microphone_initialized = false;
	right_microphone_initialized = false;
	left_microphone_available = true;
	right_microphone_available = true;
	amplifier_initialized = false;
	amplifier_available = false;
	microphone_count = 0;
	audioplay_nextimeavailable = 0.0;
	volume_perc = 100.0;
	last_armedstate = ARMEDSTATUS_UNDEFINED;
	return diag;
}
eros::diagnostic AudioNodeProcess::update(double t_dt, double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	diag = update_baseprocess(t_dt, t_ros_time);
	if (task_state == TASKSTATE_PAUSE)
	{
	}
	else if (task_state == TASKSTATE_RESET)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if (v == false)
		{
			diag = update_diagnostic(SOFTWARE, ERROR, DIAGNOSTIC_FAILED,
									 "Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
	}
	else if(task_state == TASKSTATE_INITIALIZING)
	{
		diag = update_diagnostic(DATA_STORAGE, NOTICE, INITIALIZING, "Node Not Initialized Yet.");
	}
	else if (task_state == TASKSTATE_INITIALIZED)
	{
		diag = update_diagnostic(DATA_STORAGE, NOTICE, INITIALIZING, "Node Initialized but not Running.");
		if (microphone_count == 0)
		{
		}
		else if (microphone_count == 1)
		{
			if ((left_microphone_available == true) and (left_microphone_initialized == true) and (amplifier_available == true) and (amplifier_initialized == true))
			{
				request_statechange(TASKSTATE_RUNNING);
				diag = update_diagnostic(SENSORS, INFO, NOERROR, "Microphone Ready.");
			}
			else
			{
				//ready = false;
			}
		}
		else if (microphone_count == 2)
		{
			if ((left_microphone_available == true) and
				(left_microphone_initialized == true) and
				(right_microphone_available == true) and
				(right_microphone_initialized == true) and
				(amplifier_available == true) and
				(amplifier_initialized == true))
			{
				diag = update_diagnostic(SENSORS, INFO, NOERROR, "Microphone Ready.");
				request_statechange(TASKSTATE_RUNNING);
			}
			else
			{
				//ready = false;
			}
		}
		else
		{
			//ready = false;
		}
	}
	else if (task_state == TASKSTATE_RUNNING)
	{
	}
	else if (task_state != TASKSTATE_RUNNING)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if (v == false)
		{
			diag = update_diagnostic(SOFTWARE, ERROR, DIAGNOSTIC_FAILED,
									 "Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
	}
	audiorecord_timer += t_dt;

	if (task_state == TASKSTATE_RUNNING)
	{

		for (std::size_t i = 0; i < audiorecord_files.size(); i++)
		{
			if (ros_time > (audiorecord_files.at(i).time_created + totalaudio_tokeep))
			{
				if (archive == false)
				{
					char tempstr[256];
					sprintf(tempstr, "rm %s", audiorecord_files.at(i).filepath.c_str());
					exec(tempstr, false);
				}
				else
				{
					printf("[Archive] %d: %s\n", (int)i, audiorecord_files.at(i).filepath.c_str());
					char tempstr[256];
					sprintf(tempstr, "mv %s %s", audiorecord_files.at(i).filepath.c_str(), audioarchive_directory.c_str());
					exec(tempstr, false);
				}
				number_files_removed++;
				audiorecord_files.erase(audiorecord_files.begin() + i);
			}
		}
		for (std::size_t i = 0; i < audioplay_files.size(); i++)
		{

			if (audioplay_files.at(i).playing == true)
			{
				audioplay_files.at(i).play_time += t_dt;
				if (audioplay_files.at(i).play_time >= audioplay_files.at(i).duration_sec)
				{
					audioplay_files.at(i).playing = false;
					audioplay_files.at(i).play_time = 0.0;
				}
			}
		}
		diag = update_diagnostic(DATA_STORAGE, INFO, NOERROR, "No Error.");
		diag = update_diagnostic(SOFTWARE, INFO, NOERROR, "Node Running.");
	}
	return diag;
}
eros::diagnostic AudioNodeProcess::new_devicemsg(const eros::device::ConstPtr &t_device)
{
	eros::diagnostic diag = root_diagnostic;
	eros::device device = convert_fromptr(t_device);
	if (t_device->DeviceName == host_name)
	{
	}
	else if (t_device->DeviceType == DEVICETYPE_MICROPHONE)
	{
		if ((left_microphone_initialized == false) ||
			(right_microphone_initialized == false))
		{
			for (std::size_t i = 0; i < t_device->Capabilities.size(); i++)
			{
				if (t_device->Capabilities.at(i) == "stereo")
				{

					microphone_count = 1;
					left_microphone_initialized = true;
					right_microphone_initialized = false;
					left_microphone_available = true;
					right_microphone_available = false;
					left_microphone = device;
					diag = update_diagnostic(t_device->DeviceName, SENSORS, INFO, NOERROR, "Microphone Initialized.");
				}
				else if (t_device->Capabilities.at(i) == "mono")
				{
					microphone_count = 2;
					if (t_device->DeviceName.find("Left") != std::string::npos)
					{
						left_microphone_initialized = true;
						left_microphone_available = true;
						left_microphone = device;
						diag = update_diagnostic(t_device->DeviceName, SENSORS, INFO, NOERROR, "Microphone Initialized.");
					}
					else if (t_device->DeviceName.find("Right") != std::string::npos)
					{
						right_microphone_initialized = true;
						right_microphone_available = true;
						right_microphone = device;
						diag = update_diagnostic(t_device->DeviceName, SENSORS, INFO, NOERROR, "Microphone Initialized.");
					}
				}
			}
		}
	}
	else if (t_device->DeviceType == DEVICETYPE_AUDIOAMPLIFIER)
	{
		amplifier = device;
		amplifier_available = true;
		amplifier_initialized = true;
		diag = update_diagnostic(t_device->DeviceName, REMOTE_CONTROL, INFO, NOERROR, "Audio Amplifier Initialized.");
		diag = update_diagnostic(REMOTE_CONTROL, INFO, NOERROR, "Audio Amplifier Initialized.");
	}
	return diag;
}
std::vector<eros::diagnostic> AudioNodeProcess::new_commandmsg(const eros::command::ConstPtr &t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	if (t_msg->Command == ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		if (t_msg->Option1 == LEVEL1)
		{
			diaglist.push_back(diag);
		}
		else if (t_msg->Option1 == LEVEL2)
		{
			diaglist = check_programvariables();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL3)
		{
			diaglist = run_unittest();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL4)
		{
		}
	}
	else if (t_msg->Command == ROVERCOMMAND_TASKCONTROL)
	{
		if (node_name.find(t_msg->CommandText) != std::string::npos)
		{
			uint8_t prev_taskstate = get_taskstate();
			bool v = request_statechange(t_msg->Option2);
			if (v == false)
			{
				diag = update_diagnostic(SOFTWARE, ERROR, DIAGNOSTIC_FAILED,
										 "Unallowed State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}
			else
			{
				if (task_state == TASKSTATE_RESET)
				{
					reset();
				}
				diag = update_diagnostic(SOFTWARE, NOTICE, DIAGNOSTIC_PASSED,
										 "Commanded State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}
		}
	}
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		diag = update_diagnostic(diaglist.at(i));
	}
	return diaglist;
}
std::vector<eros::diagnostic> AudioNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true)
	{
		diag = update_diagnostic(SOFTWARE, INFO, DIAGNOSTIC_PASSED, "Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	}
	else
	{
		diag = update_diagnostic(SOFTWARE, WARN, DIAGNOSTIC_FAILED, "Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
void AudioNodeProcess::new_armedstatemsg(uint8_t armed_state)
{
	if (armed_state != last_armedstate)
	{
		switch (armed_state)
		{
		case ARMEDSTATUS_UNDEFINED:
			new_audioplaytrigger("ArmedState:Undefined", true);
			break;
		case ARMEDSTATUS_ARMED:
			new_audioplaytrigger("ArmedState:Armed", true);
			break;
		case ARMEDSTATUS_DISARMED_CANNOTARM:
			new_audioplaytrigger("ArmedState:DisarmedCannotArm", true);
			break;
		case ARMEDSTATUS_DISARMED:
			new_audioplaytrigger("ArmedState:Disarmed", true);
			break;
		case ARMEDSTATUS_ARMING:
			new_audioplaytrigger("ArmedState:Arming", true);
			break;
		case ARMEDSTATUS_DISARMING:
			new_audioplaytrigger("ArmedState:Disarming", true);
			break;
		default:
			new_audioplaytrigger("Sorry", true);
			break;
		}
	}
	last_armedstate = armed_state;
}

bool AudioNodeProcess::set_audiostoragedirectory(std::string v)
{

	struct stat status;
	if (stat(v.c_str(), &status) == 0)
	{
		audiostorage_directory = v;

		char tempstr[256];
		sprintf(tempstr, "rm -r -f %s/input/*", v.c_str());
		exec(tempstr, false);
		init_audioplayfiles();
		return true;
	}
	else
	{
		printf("Parameter: %s is not a valid directory.\n", v.c_str());
		return false;
	}
}
bool AudioNodeProcess::set_audioarchivedirectory(std::string v)
{
	struct stat status;
	if (stat(v.c_str(), &status) == 0)
	{
		audioarchive_directory = v;
		return true;
	}
	else
	{
		printf("Parameter: %s is not a valid directory.\n", v.c_str());
		return false;
	}
}
bool AudioNodeProcess::get_audiorecordtrigger(std::string &command, std::string &filepath)
{
	if (task_state == TASKSTATE_RUNNING)
	{
		if (audiorecord_timer > ((double)audiorecord_duration + AUDIOWAIT_TIME))
		{
			audiorecord_timer = 0.0;
			char tempstr[512];
			double timestamp = ros_time;
			unsigned long long t = (unsigned long long)(1000.0 * timestamp);
			sprintf(tempstr, "arecord -q -d %d -D plughw:1 -c2 -r 48000 -f S32_LE -t wav %s/input/%llu.wav </dev/null &>/dev/null &", audiorecord_duration, audiostorage_directory.c_str(), t);
			command = std::string(tempstr);
			char tempstr2[256];
			sprintf(tempstr2, "%s/input/%llu.wav", audiostorage_directory.c_str(), t);
			filepath = std::string(tempstr2);
			AudioRecordFile f;
			f.filepath = filepath;
			f.time_created = timestamp;
			audiorecord_files.push_back(f);

			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}
bool AudioNodeProcess::get_audioplaytrigger(std::string &command, std::string &filepath)
{
	bool trigger = false;
	if (task_state == TASKSTATE_RUNNING)
	{
		if (audio_playing == true)
		{
			command = "";
			filepath = "";
			trigger = false;
			return trigger;
		}
		else
		{

			return true;
		}
	}
	else
	{
		return false;
	}
}
void AudioNodeProcess::init_audioplayfiles()
{

	{
		AudioPlayFile file;
		file.trigger = "Robot:Booting";
		file.filepath = audiostorage_directory + "/output/" + "RobotBooting.mp3";
		file.priority = 3;
		audioplay_files.push_back(file);
	}
	{
		AudioPlayFile file;
		file.trigger = "Robot:PowerDown";
		file.filepath = audiostorage_directory + "/output/" + "RobotPoweringDown.mp3";
		file.priority = 3;
		audioplay_files.push_back(file);
	}
	{
		AudioPlayFile file;
		file.trigger = "ArmedState:Undefined";
		file.filepath = audiostorage_directory + "/output/" + "RobotArmUndefined.mp3";
		file.priority = 3;
		audioplay_files.push_back(file);
	}
	{
		AudioPlayFile file;
		file.trigger = "ArmedState:Undefined";
		file.filepath = audiostorage_directory + "/output/" + "RobotArmUndefined.mp3";
		file.priority = 3;
		audioplay_files.push_back(file);
	}
	{
		AudioPlayFile file;
		file.trigger = "ArmedState:Armed";
		file.filepath = audiostorage_directory + "/output/" + "RobotArmed.mp3";
		file.priority = 3;
		audioplay_files.push_back(file);
	}
	{
		AudioPlayFile file;
		file.trigger = "ArmedState:Disarmed";
		file.filepath = audiostorage_directory + "/output/" + "RobotDisarmed.mp3";
		file.priority = 3;
		audioplay_files.push_back(file);
	}
	{
		AudioPlayFile file;
		file.trigger = "ArmedState:DisarmedCannotArm";
		file.filepath = audiostorage_directory + "/output/" + "RobotDisarmedCannotArm.mp3";
		file.priority = 3;
		audioplay_files.push_back(file);
	}
	{
		AudioPlayFile file;
		file.trigger = "ArmedState:Arming";
		file.filepath = audiostorage_directory + "/output/" + "RobotArming.mp3";
		file.priority = 3;
		audioplay_files.push_back(file);
	}
	{
		AudioPlayFile file;
		file.trigger = "ArmedState:Disarming";
		file.filepath = audiostorage_directory + "/output/" + "RobotDisarming.mp3";
		file.priority = 3;
		audioplay_files.push_back(file);
	}
	{
		AudioPlayFile file;
		file.trigger = "Sorry";
		file.filepath = audiostorage_directory + "/output/" + "Sorry.mp3";
		file.priority = 3;
		audioplay_files.push_back(file);
	}

	for (std::size_t i = 0; i < audioplay_files.size(); i++)
	{
		audioplay_files.at(i).duration_sec = -1.0;
		audioplay_files.at(i).last_playtime = 0.0;
		audioplay_files.at(i).play_time = 0.0;
		audioplay_files.at(i).playing = false;
		char tempstr[512];
		sprintf(tempstr, "mediainfo --Inform=\"Audio;%%Duration%%\" %s\n", audioplay_files.at(i).filepath.c_str());
		std::string result = exec(tempstr, true);
		audioplay_files.at(i).duration_sec = std::atof(result.c_str()) / 1000.0;
	}
}
bool AudioNodeProcess::add_audioplayfile(std::string filepath, std::string trigger, uint8_t priority)
{
	std::ifstream infile(filepath.c_str());
	if (infile.good() == false)
	{
		return false;
	}
	AudioPlayFile file;
	file.trigger = trigger;
	file.filepath = filepath;
	file.priority = priority;
	file.play_time = 0.0;
	file.playing = false;
	char tempstr[512];
	sprintf(tempstr, "mediainfo --Inform=\"Audio;%%Duration%%\" %s\n", filepath.c_str());
	std::string result = exec(tempstr, true);
	file.duration_sec = std::atof(result.c_str()) / 1000.0;
	audioplay_files.push_back(file);
	return true;
}
bool AudioNodeProcess::new_audioplaytrigger(std::string trigger, bool bypass)
{
	if (bypass == false)
	{
		if (task_state == TASKSTATE_INITIALIZED)
		{
			return false;
		}
	}
	for (std::size_t i = 0; i < audioplay_files.size(); i++)
	{
		if (audioplay_files.at(i).playing == true)
		{
			audioplay_files.at(i).playing = false;
			char tempstr[256];
			sprintf(tempstr, "pidof mpg321");
			std::string result = exec(tempstr, true);
			if (result != "")
			{
				int pid = std::atoi(result.c_str());
				char tempstr2[256];
				sprintf(tempstr2, "kill %d >/dev/null 2>&1 &", pid);
				exec(tempstr2, false);
			}
		}
	}
	for (std::size_t i = 0; i < audioplay_files.size(); i++)
	{
		if (audioplay_files.at(i).trigger == trigger)
		{
			double v_set = 0.0;
			switch (audioplay_files.at(i).priority)
			{
			case 0:
				v_set = 0.0;
				break;
			case 1:
				v_set = 10.0;
				break;
			case 2:
				v_set = 20.0;
				break;
			case 3:
				v_set = 30.0;
				break;
			default:
				v_set = 0.0;
				break;
			}
			char tempstr[512];
			sprintf(tempstr, "mpg321 -g %d -q %s >/dev/null 2>&1 &", (int)(v_set * volume_perc / 100.0), audioplay_files.at(i).filepath.c_str());
			exec(tempstr, false);
			audioplay_files.at(i).playing = true;
			audioplay_files.at(i).last_playtime = ros_time;
			return true;
		}
	}
	return false;
}
