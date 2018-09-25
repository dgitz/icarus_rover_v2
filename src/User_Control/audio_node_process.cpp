#include "audio_node_process.h"
/*! \brief Constructor
 */
AudioNodeProcess::AudioNodeProcess()
{
	run_time = 0.0;
	initialized = false;
	ready = false;
	audiorecord_duration = -1;
	audiostorage_directory = "";
	audiorecord_timer = 0.0;
	current_timestamp = 0.0;
	totalaudio_tokeep = 30.0;
	number_files_removed = 0;
	left_microphone_initialized = false;
	right_microphone_initialized = false;
	left_microphone_available = true;
	right_microphone_available = true;
	amplifier_initialized = false;
	amplifier_available = false;
	microphone_count = 0;
	audio_playing = false;
	audioplay_nextimeavailable = 0.0;
}
/*! \brief Deconstructor
 */
AudioNodeProcess::~AudioNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic AudioNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	char tempstr[256];
	sprintf(tempstr,"killall mpg123 </dev/null &>/dev/null &");
	system(tempstr);
	return diagnostic;
}
void AudioNodeProcess::new_armedstatemsg(uint8_t armed_state)
{
	if(armed_state != last_armedstate)
	{
		switch(armed_state)
		{
		case ARMEDSTATUS_UNDEFINED:
			new_audioplaytrigger("ArmedState:Undefined");
			break;
		case ARMEDSTATUS_ARMED:
			new_audioplaytrigger("ArmedState:Armed");
			break;
		case ARMEDSTATUS_DISARMED_CANNOTARM:
			new_audioplaytrigger("ArmedState:DisarmedCannotArm");
			break;
		case ARMEDSTATUS_DISARMED:
			new_audioplaytrigger("ArmedState:Disarmed");
			break;
		case ARMEDSTATUS_ARMING:
			new_audioplaytrigger("ArmedState:Arming");
			break;
		case ARMEDSTATUS_DISARMING:
			new_audioplaytrigger("ArmedState:Disarming");
			break;
		default:
			new_audioplaytrigger("Sorry");
			break;
		}
	}
	last_armedstate = armed_state;

}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic AudioNodeProcess::update(double timestamp,double dt)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	run_time += dt;
	current_timestamp = timestamp;
	audiorecord_timer += dt;
	if(microphone_count == 0)
	{
		ready = false;  //At least 1 required
	}
	else if(microphone_count == 1)
	{
		if((left_microphone_available == true) and (left_microphone_initialized == true)
			and (amplifier_available == true) and (amplifier_initialized == true))
		{
			ready = true;
		}
		else
		{
			ready = false;
		}
	}
	else if(microphone_count == 2)
	{
		if((left_microphone_available == true) and
				(left_microphone_initialized == true) and
				(right_microphone_available == true) and
				(right_microphone_initialized == true) and
				(amplifier_available == true) and
				(amplifier_initialized == true))
		{
			ready = true;
		}
		else
		{
			ready = false;
		}
	}
	else
	{
		ready = false;
	}
	if(ready == true)
	{

		for(std::size_t i = 0; i < audiorecord_files.size(); i++)
		{
			if(timestamp > (audiorecord_files.at(i).time_created + totalaudio_tokeep))
			{
				if(archive == false)
				{
					printf("[Delete] %d: %s\n",(int)i,audiorecord_files.at(i).filepath.c_str());
					char tempstr[256];
					sprintf(tempstr,"exec rm %s",audiorecord_files.at(i).filepath.c_str());
					system(tempstr);
				}
				else
				{
					printf("[Archive] %d: %s\n",(int)i,audiorecord_files.at(i).filepath.c_str());
					char tempstr[256];
					sprintf(tempstr,"exec mv %s %s",audiorecord_files.at(i).filepath.c_str(),audioarchive_directory.c_str());
					system(tempstr);
				}
				number_files_removed++;
				audiorecord_files.erase(audiorecord_files.begin()+i);
			}
		}
		for(std::size_t i = 0; i < audioplay_files.size(); i++)
		{
			if(audioplay_files.at(i).playing == true)
			{
				audioplay_files.at(i).play_time += dt;
				if(audioplay_files.at(i).play_time >= audioplay_files.at(i).duration_sec)
				{
					audioplay_files.at(i).playing = false;
					audioplay_files.at(i).play_time = 0.0;
				}
			}
		}
	}
	if(initialized == false)
	{

		diag.Diagnostic_Type = NOERROR;
		diag.Level = NOTICE;
		diag.Diagnostic_Message = INITIALIZING;
		diag.Description = "Node Not Initialized Yet.";
	}
	else if((initialized == true) and (ready == false))
	{
		diag.Diagnostic_Type = NOERROR;
		diag.Level = NOTICE;
		diag.Diagnostic_Message = INITIALIZING;
		diag.Description = "Node Initialized but not Running";
	}
	else if(ready == true)
	{
		diag.Diagnostic_Type = NOERROR;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Node Running";
	}

	diagnostic = diag;
	return diag;
}
/*! \brief Setup Process Device info
 */
icarus_rover_v2::diagnostic AudioNodeProcess::new_devicemsg(icarus_rover_v2::device device)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	bool new_device = true;
	if(device.DeviceName == myhostname)
	{

	}
	else if(device.DeviceType == "Microphone")
	{
		if((left_microphone_initialized == false) ||
				(right_microphone_initialized == false))
			bool mono_cap;
		bool stereo_cap;
		for(std::size_t i = 0; i < device.Capabilities.size(); i++)
		{
			if(device.Capabilities.at(i) == "stereo")
			{
				microphone_count = 1;
				left_microphone_initialized = true;
				right_microphone_initialized = false;
				left_microphone_available = true;
				right_microphone_available = false;
				left_microphone = device;

			}
			else if(device.Capabilities.at(i) == "mono")
			{
				microphone_count = 2;
				if(device.DeviceName.find("Left") != std::string::npos)
				{
					left_microphone_initialized = true;
					left_microphone_available = true;
					left_microphone = device;
				}
				else if(device.DeviceName.find("Right") != std::string::npos)
				{
					right_microphone_initialized = true;
					right_microphone_available = true;
					right_microphone = device;
				}
			}
		}
	}
	else if(device.DeviceType == "AudioAmplifier")
	{
		amplifier = device;
		amplifier_available = true;
		amplifier_initialized = true;
	}
	return diag;
}
/*! \brief Process Command Message
 */
std::vector<icarus_rover_v2::diagnostic> AudioNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
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
		}
		else if(cmd.Option1 == LEVEL4)
		{
		}
	}
	diaglist.push_back(diag);
	return diaglist;
}
/*! \brief Self-Diagnostic-Check Program Variables
 */
std::vector<icarus_rover_v2::diagnostic> AudioNodeProcess::check_program_variables()
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
bool AudioNodeProcess::set_audiostoragedirectory(std::string v)
{
	if(access(v.c_str(),0) == 0)
	{
		struct stat status;
		stat(v.c_str(),&status);
		if(status.st_mode & S_IFDIR)
		{
			audiostorage_directory = v;
			char tempstr[256];
			sprintf(tempstr,"exec rm -r -f %s/input/*",v.c_str());
			system(tempstr);
			init_audioplayfiles();
			return true;
		}
		else
		{
			printf("Parameter: %s is not a directory, it is a file.\n",v.c_str());
			return false;
		}
	}
	else
	{
		return false;
	}
}
bool AudioNodeProcess::set_audioarchivedirectory(std::string v)
{
	if(access(v.c_str(),0) == 0)
	{
		struct stat status;
		stat(v.c_str(),&status);
		if(status.st_mode & S_IFDIR)
		{
			audioarchive_directory = v;
			char tempstr[256];

			return true;
		}
		else
		{
			printf("Parameter: %s is not a directory, it is a file.\n",v.c_str());
			return false;
		}
	}
	else
	{
		return false;
	}
}
bool AudioNodeProcess::get_audiorecordtrigger(std::string& command,std::string& filepath)
{
	bool trigger = false;
	if(ready == true)
	{
		if(audiorecord_timer > ((double)audiorecord_duration+AUDIOWAIT_TIME))
		{
			audiorecord_timer = 0.0;
			char tempstr[512];
			double timestamp = current_timestamp;
			unsigned long long t = (unsigned long long)(1000.0*timestamp);
			sprintf(tempstr,"arecord -q -d %d -D plughw:1 -c2 -r 48000 -f S32_LE -t wav %s/input/%llu.wav </dev/null &>/dev/null &",audiorecord_duration,audiostorage_directory.c_str(),t);
			command = std::string(tempstr);

			char tempstr2[256];
			sprintf(tempstr2,"%s/input/%llu.wav",audiostorage_directory.c_str(),t);
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
bool AudioNodeProcess::get_audioplaytrigger(std::string& command,std::string& filepath)
{
	bool trigger = false;
	if(ready == true)
	{
		if(audio_playing == true)
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

	for(std::size_t i = 0; i < audioplay_files.size(); i++)
	{
		audioplay_files.at(i).duration_sec = -1.0;
		audioplay_files.at(i).last_playtime = 0.0;
		audioplay_files.at(i).play_time = 0.0;
		audioplay_files.at(i).playing = false;
		char tempstr[512];
		sprintf(tempstr,"mediainfo --Inform=\"Audio;%Duration%\" %s\n",audioplay_files.at(i).filepath.c_str());
		std::string result = exec(tempstr);
		audioplay_files.at(i).duration_sec = std::atof(result.c_str())/1000.0;
	}

}
bool AudioNodeProcess::add_audioplayfile(std::string filepath,std::string trigger,uint8_t priority)
{
	std::ifstream infile(filepath.c_str());
	if(infile.good() == false)
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
	sprintf(tempstr,"mediainfo --Inform=\"Audio;%Duration%\" %s\n",filepath.c_str());
	std::string result = exec(tempstr);
	file.duration_sec = std::atof(result.c_str())/1000.0;
	audioplay_files.push_back(file);
	return true;
}
bool AudioNodeProcess::new_audioplaytrigger(std::string trigger)
{
	if(ready == false) { return false; }
	bool interrupt = false;
	for(std::size_t i =0; i < audioplay_files.size(); i++)
	{
		if(audioplay_files.at(i).playing == true)
		{
			audioplay_files.at(i).playing = false;
			char tempstr[256];
			sprintf(tempstr,"pidof mpg123");
			int pid = std::atoi(exec(tempstr).c_str());
			char tempstr2[256];
			sprintf(tempstr2,"kill %d </dev/null &>/dev/null &",pid);
			system(tempstr2);
		}
	}
	for(std::size_t i = 0; i < audioplay_files.size(); i++)
	{
		if(audioplay_files.at(i).trigger == trigger)
		{
			int v = 0;
			switch(audioplay_files.at(i).priority)
			{
			case 0:
				v = 0;
				break;
			case 1:
				v = 10;
				break;
			case 2:
				v = 20;
				break;
			case 3:
				v = 30;
				break;
			default:
				v = 0;
				break;
			}
			char tempstr[512];

			sprintf(tempstr,"mpg123 -g %d -q %s </dev/null &>/dev/null &",v,audioplay_files.at(i).filepath.c_str());
			system(tempstr);
			audioplay_files.at(i).playing = true;
			audioplay_files.at(i).last_playtime = current_timestamp;
			return true;
		}
	}
	return false;
}
std::string AudioNodeProcess::exec(const char* cmd) {
	char buffer[512];
	std::string result = "";
	FILE* pipe = popen(cmd, "r");
	if (!pipe) throw std::runtime_error("popen() failed!");
	try {
		while (!feof(pipe)) {
			if (fgets(buffer, 512, pipe) != NULL)
				result += buffer;
		}
	} catch (...) {
		pclose(pipe);
		throw;
	}
	pclose(pipe);
	return result;
}
