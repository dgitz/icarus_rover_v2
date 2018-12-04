#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
#include <fstream>
#include <sys/types.h>  // For stat().
#include <sys/stat.h>   // For stat().
#include <cstdio>
#include <memory>
#include <fstream>
//C++ System Files
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
//ROS Base Functionality
//ROS Messages
//Project
/*! \class AudioNodeProcess AudioNodeProcess.h "AudioNodeProcess.h"
 *  \brief This is a AudioNodeProcess class.  Used for the audio_node node.
 *
 */
class AudioNodeProcess: public BaseNodeProcess {
public:
    //Constants
    const double AUDIOWAIT_TIME = 0.2f;
    //Enums
    //Structs
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
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	icarus_rover_v2::diagnostic finish_initialization();
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	icarus_rover_v2::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
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
	void set_volume(double v) { volume_perc = v; }
	//Message Functions
	/*! \brief  Process Command Message.  
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg);
	icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& device);
    void new_armedstatemsg(uint8_t armed_state);
	//Support Functions
   	bool new_audioplaytrigger(std::string trigger,bool bypass);

    //Printing Functions
    
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();
    void init_audioplayfiles();
    std::string exec(const char* cmd);
    
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
