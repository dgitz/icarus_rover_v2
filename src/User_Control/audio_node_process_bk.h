#ifndef AUDIONODEPROCESS_H
#define AUDIONODEPROCESS_H

#include "Definitions.h"
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/pin.h"
#include "icarus_rover_v2/firmware.h"
#include <std_msgs/UInt8.h>
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#include <sys/types.h>  // For stat().
#include <sys/stat.h>   // For stat().
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <fstream>
#define AUDIOWAIT_TIME 0.2f
class AudioNodeProcess
{
public:

	struct AudioRecordFile
	{
		std::string filepath;
		double time_created;
	};
	struct AudioPlayFile
	{
		std::string trigger;
		std::string filepath;
		uint8_t priority;
		double duration_sec;
		double last_playtime;
		double play_time;
		bool playing;
		double repeat_frequency;
	};

	AudioNodeProcess(std::string _base_node_name,std::string _node_name);
	std::string get_basenodename() { return base_node_name; }
	std::string get_nodename() { return node_name; }
	~AudioNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double timestamp,double dt);
	void set_diagnostic(icarus_rover_v2::diagnostic v) { diagnostic = v; }
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }
	void new_armedstatemsg(uint8_t armed_state);
	double get_runtime() { return run_time; }
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device device);
	void set_mydevice(icarus_rover_v2::device device) { mydevice = device; initialized = true; }
	bool get_initialized() { return initialized; }
    bool get_ready() { return ready; }
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(icarus_rover_v2::command cmd);
	bool set_audiostoragedirectory(std::string v);
	bool set_audioarchivedirectory(std::string v);
	std::string get_audiostoragedirectory() { return audiostorage_directory; }
	std::string get_audioarchivedirectory() { return audioarchive_directory; }
	void set_audiorecord_duration(int v) { audiorecord_duration = v; }
	bool get_audiorecordtrigger(std::string& command,std::string& filepath);
	bool get_audioplaytrigger(std::string& command,std::string& filepath);
	void set_totalaudiofiletimetokeep(double v){ totalaudio_tokeep = v; }
	unsigned long int get_numberaudiofiles_removed() { return number_files_removed; }
	void enable_archive(bool v) { archive = v; }
	bool add_audioplayfile(std::string filepath,std::string trigger,uint8_t priority); //Only used for unit testing
	bool new_audioplaytrigger(std::string trigger,bool bypass);
	void set_volume(double v) { volume_perc = v; }
	
    
private:
    std::string base_node_name;
	std::string node_name;
	bool unittest_running;
    std::vector<icarus_rover_v2::diagnostic> check_program_variables();
    std::vector<icarus_rover_v2::diagnostic> run_unittest();
    void init_audioplayfiles();
    std::string exec(const char* cmd);
	double run_time;
	icarus_rover_v2::diagnostic diagnostic;
	icarus_rover_v2::device mydevice;
	std::string myhostname;
	bool initialized;
    bool ready;
    double current_timestamp;
    std::string audiostorage_directory;
    std::string audioarchive_directory;
    int audiorecord_duration;
    double audiorecord_timer;
    double totalaudio_tokeep;
    bool archive;
    std::vector<AudioRecordFile> audiorecord_files;
    unsigned long int number_files_removed;

    icarus_rover_v2::device amplifier;
    bool amplifier_initialized;
    bool amplifier_available;
    icarus_rover_v2::device left_microphone;
    bool left_microphone_initialized;
    bool left_microphone_available;
    icarus_rover_v2::device right_microphone;
    bool right_microphone_initialized;
    bool right_microphone_available;

    int microphone_count;

    std::vector<AudioPlayFile> audioplay_files;
    bool audio_playing;
    double audioplay_nextimeavailable;
    uint8_t last_armedstate;
    double volume_perc;

};
#endif
