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
	microphone_count = 0;
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
	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic AudioNodeProcess::update(double timestamp,double dt)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	run_time += dt;
	current_timestamp = timestamp;
	audiorecord_timer += dt;
	if(microphone_count == 1)
	{
		ready = false;  //At least 1 required
	}
	else if(microphone_count == 1)
	{
		if((left_microphone_available == true) and (left_microphone_initialized == true))
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
				(right_microphone_initialized == true))
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

		for(std::size_t i = 0; i < audio_files.size(); i++)
		{
			if(timestamp > (audio_files.at(i).time_created + totalaudio_tokeep))
			{
				if(archive == false)
				{
					printf("[Delete] %d: %s\n",(int)i,audio_files.at(i).filepath.c_str());
					char tempstr[256];
					sprintf(tempstr,"exec rm %s",audio_files.at(i).filepath.c_str());
					system(tempstr);
				}
				else
				{
					printf("[Archive] %d: %s\n",(int)i,audio_files.at(i).filepath.c_str());
					char tempstr[256];
					sprintf(tempstr,"exec mv %s %s",audio_files.at(i).filepath.c_str(),audioarchive_directory.c_str());
					system(tempstr);
				}
				number_files_removed++;
				audio_files.erase(audio_files.begin()+i);
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
			sprintf(tempstr,"exec rm -r -f %s/*",v.c_str());
			system(tempstr);
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
bool AudioNodeProcess::get_audiotrigger(std::string& command,std::string& filepath)
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
			sprintf(tempstr,"arecord -q -d %d -D plughw:1 -c2 -r 48000 -f S32_LE -t wav %s/%llu.wav </dev/null &>/dev/null &",audiorecord_duration,audiostorage_directory.c_str(),t);
			command = std::string(tempstr);

			char tempstr2[256];
			sprintf(tempstr2,"%s/%llu.wav",audiostorage_directory.c_str(),t);
			filepath = std::string(tempstr2);
			AudioFile f;
			f.filepath = filepath;
			f.time_created = timestamp;
			audio_files.push_back(f);
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
