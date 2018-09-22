#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../audio_node_process.h"
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#define AUDIORECORD_DURATION 2
#define LENGTH_AUDIOFILES_TOKEEP 10.0f

std::string Node_Name = "/unittest_audio_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;


AudioNodeProcess* initializeprocess(bool stereo)
{
    icarus_rover_v2::diagnostic diagnostic;
    diagnostic.DeviceName = ros_DeviceName;
    diagnostic.Node_Name = Node_Name;
    diagnostic.System = ROVER;
    diagnostic.SubSystem = ROBOT_CONTROLLER;
    diagnostic.Component = COMMUNICATION_NODE;

    diagnostic.Diagnostic_Type = NOERROR;
    diagnostic.Level = INFO;
    diagnostic.Diagnostic_Message = INITIALIZING;
    diagnostic.Description = "Node Initializing";
    
    icarus_rover_v2::device device;
    device.DeviceName = diagnostic.DeviceName;
    device.BoardCount = 0;
    device.DeviceParent = "None";
    device.Architecture = "x86_64";



    AudioNodeProcess *process;
    process = new AudioNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process->get_initialized() == false);
    process->set_mydevice(device);
    EXPECT_TRUE(process->get_initialized() == true);
    EXPECT_TRUE(process->get_ready() == false);

    if(stereo == true)
    {

        icarus_rover_v2::device microphone;
        microphone.DeviceName = "MainMicrophone";
        microphone.DeviceType = "Microphone";
        microphone.DeviceParent = ros_DeviceName;
        microphone.Capabilities.push_back("stereo");
        diagnostic = process->new_devicemsg(microphone);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);

    }
    else
    {
    	icarus_rover_v2::device left_microphone;
    	left_microphone.DeviceName = "LeftMicrophone";
    	left_microphone.DeviceType = "Microphone";
    	left_microphone.DeviceParent = ros_DeviceName;
    	left_microphone.Capabilities.push_back("mono");
    	diagnostic = process->new_devicemsg(left_microphone);
    	EXPECT_TRUE(diagnostic.Level <= NOTICE);

    	icarus_rover_v2::device right_microphone;
    	right_microphone.DeviceName = "RightMicrophone";
    	right_microphone.DeviceType = "Microphone";
    	right_microphone.DeviceParent = ros_DeviceName;
    	right_microphone.Capabilities.push_back("mono");
    	diagnostic = process->new_devicemsg(right_microphone);
    	EXPECT_TRUE(diagnostic.Level <= NOTICE);
    }
    EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
    EXPECT_FALSE(process->set_audiostoragedirectory("A Directory that should never exist."));
    EXPECT_TRUE(process->set_audiostoragedirectory("/home/robot/logs/output/AUDIO/unit_test"));
    process->set_audiorecord_duration(AUDIORECORD_DURATION);
    process->set_totalaudiofiletimetokeep(LENGTH_AUDIOFILES_TOKEEP);
    return process;
}
AudioNodeProcess* readyprocess(AudioNodeProcess* process)
{
    icarus_rover_v2::diagnostic diag = process->update(0.0,0);
    EXPECT_TRUE(diag.Level <= NOTICE);
    EXPECT_TRUE(process->get_ready() == true);
    return process;
}
TEST(Template,Process_Initialization)
{
    AudioNodeProcess* process = initializeprocess(true);
}
/*
TEST(Template,AudioStorage_Delete)
{
    AudioNodeProcess* process = initializeprocess();
    process = readyprocess(process);
    double time_to_run = 30.0;
    double dt = 0.001;
    double current_time = 0.0;
    bool fastrate_fire = false; //10 Hz
    bool mediumrate_fire = false; //1 Hz
    bool slowrate_fire = false; //0.1 Hz
    int audiotrigger_count = 0;
    while(current_time <= time_to_run)
    {
        icarus_rover_v2::diagnostic diag = process->update(current_time,dt);
        EXPECT_TRUE(diag.Level <= NOTICE);
        std::string command,filepath;
        if(process->get_audiotrigger(command,filepath))
        {
        	audiotrigger_count++;
        	printf("[Create] %s\n%s\n",command.c_str(),filepath.c_str());

        	//Simulate creating a new file
        	char tempstr[256];
        	sprintf(tempstr,"exec touch %s",filepath.c_str());
        	system(tempstr);
        }
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
        icarus_rover_v2::command cmd;
        cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
        if(fastrate_fire == true)
        {
            cmd.Option1 = LEVEL3;
            std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
            for(std::size_t i = 0; i < diaglist.size(); i++)
            {
                EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
            }
             EXPECT_TRUE(diaglist.size() > 0);

        }
        if(mediumrate_fire == true)
        {
            cmd.Option1 = LEVEL2;
            std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
            for(std::size_t i = 0; i < diaglist.size(); i++)
            {
                EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
            }
            EXPECT_TRUE(diaglist.size() > 0);

        }
        if(slowrate_fire == true)
        {
            cmd.Option1 = LEVEL1;
            std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
            for(std::size_t i = 0; i < diaglist.size(); i++)
            {
                EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
            }
             EXPECT_TRUE(diaglist.size() > 0);
        }
        current_time += dt;
        usleep((int)(dt*1000000.0));
    }
    EXPECT_TRUE(process->get_runtime() >= time_to_run);
    EXPECT_TRUE(audiotrigger_count > 0);

    DIR *dp;
    int audiofile_count = 0;
    struct dirent *ep;
    dp = opendir (process->get_audiostoragedirectory().c_str());

    if (dp != NULL)
    {
    	while (ep = readdir (dp))
    	{
    		audiofile_count++;
    	}
    	(void) closedir (dp);
    }
    else
    	perror ("Couldn't open the directory");
    audiofile_count = audiofile_count -2; //Subtract hidden files, : ".", ".."
    int expected_count = (int)(LENGTH_AUDIOFILES_TOKEEP/(double)AUDIORECORD_DURATION);
    if((expected_count >= (audiofile_count-1)) and
       (expected_count <= (audiofile_count +1)))
    {
    	EXPECT_TRUE(true);
    }
    else
    {
    	printf("File Count mismatch: %d %d\n",expected_count,audiofile_count);
    	EXPECT_TRUE(false);
    }


}
*/
TEST(Template,AudioStorage_Archive)
{
    AudioNodeProcess* process = initializeprocess(false);
    process = readyprocess(process);
    process->enable_archive(true);
    EXPECT_TRUE(process->set_audioarchivedirectory("/home/robot/logs/output/AUDIO/unit_test_archive"));

    char tempstr[512];
    sprintf(tempstr,"exec rm -r %s/*",process->get_audioarchivedirectory().c_str());
    system(tempstr);
    double time_to_run = 30.0;
    double dt = 0.001;
    double current_time = 0.0;
    bool fastrate_fire = false; //10 Hz
    bool mediumrate_fire = false; //1 Hz
    bool slowrate_fire = false; //0.1 Hz
    int audiotrigger_count = 0;
    while(current_time <= time_to_run)
    {
        icarus_rover_v2::diagnostic diag = process->update(current_time,dt);
        EXPECT_TRUE(diag.Level <= NOTICE);
        std::string command,filepath;
        if(process->get_audiotrigger(command,filepath))
        {
        	audiotrigger_count++;
        	printf("[Create] %s\n%s\n",command.c_str(),filepath.c_str());

        	//Simulate creating a new file
        	char tempstr[256];
        	sprintf(tempstr,"exec touch %s",filepath.c_str());
        	system(tempstr);
        }
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
        icarus_rover_v2::command cmd;
        cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
        if(fastrate_fire == true) 
        { 
            cmd.Option1 = LEVEL3;
            std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
            for(std::size_t i = 0; i < diaglist.size(); i++)
            {
                EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
            }
             EXPECT_TRUE(diaglist.size() > 0);
            
        }
        if(mediumrate_fire == true)
        {
            cmd.Option1 = LEVEL2;
            std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
            for(std::size_t i = 0; i < diaglist.size(); i++)
            {
                EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
            }
            EXPECT_TRUE(diaglist.size() > 0);
            
        }
        if(slowrate_fire == true)
        {
            cmd.Option1 = LEVEL1;
            std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
            for(std::size_t i = 0; i < diaglist.size(); i++)
            {
                EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
            }
             EXPECT_TRUE(diaglist.size() > 0);
        }
        current_time += dt;
        usleep((int)(dt*1000000.0));
    }
    EXPECT_TRUE(process->get_runtime() >= time_to_run);
    EXPECT_TRUE(audiotrigger_count > 0);

    DIR *dp;
    int audiofile_count = 0;
    struct dirent *ep;
    dp = opendir (process->get_audiostoragedirectory().c_str());

    if (dp != NULL)
    {
    	while (ep = readdir (dp))
    	{
    		audiofile_count++;
    	}
    	(void) closedir (dp);
    }
    else
    	perror ("Couldn't open the directory");
    audiofile_count = audiofile_count -2; //Subtract hidden files, : ".", ".."
    int expected_count = (int)(LENGTH_AUDIOFILES_TOKEEP/(double)AUDIORECORD_DURATION);
    if((expected_count >= (audiofile_count-1)) and
       (expected_count <= (audiofile_count +1)))
    {
    	EXPECT_TRUE(true);
    }
    else
    {
    	printf("File Count mismatch: %d %d\n",expected_count,audiofile_count);
    	EXPECT_TRUE(false);
    }

    int archivedfile_count = 0;
       dp = opendir (process->get_audioarchivedirectory().c_str());

       if (dp != NULL)
       {
       	while (ep = readdir (dp))
       	{
       		archivedfile_count++;
       	}
       	(void) closedir (dp);
       }
       else
       	perror ("Couldn't open the directory");
       printf("%d %d %d\n",archivedfile_count,process->get_numberaudiofiles_removed(),audiotrigger_count);
       archivedfile_count = archivedfile_count -2;
       EXPECT_TRUE(archivedfile_count == (process->get_numberaudiofiles_removed()));


}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

