#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/usermessage.h"
#include "../speaker_node_process.h"

std::string Node_Name = "/unittest_speaker_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Node_Name;

SpeakerNodeProcess initialize_process();

SpeakerNodeProcess initialize_process()
{
	SpeakerNodeProcess m_speakernodeprocess;
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
	m_speakernodeprocess.init(diagnostic,Host_Name);

	icarus_rover_v2::device newdevicemsg;
	newdevicemsg.DeviceName = Host_Name;
	newdevicemsg.Architecture = "x86_64";
	diagnostic = m_speakernodeprocess.new_devicemsg(newdevicemsg);
	return m_speakernodeprocess;
}
TEST(ProcessInitialization,NormalOperation)
{
	SpeakerNodeProcess m_speakernodeprocess = initialize_process();
	icarus_rover_v2::diagnostic diagnostic = m_speakernodeprocess.get_diagnostic();

	EXPECT_TRUE(diagnostic.Level < NOTICE);
	EXPECT_TRUE(diagnostic.Diagnostic_Message == NOERROR);
	EXPECT_TRUE(m_speakernodeprocess.is_finished_initializing() == true);
	EXPECT_TRUE(m_speakernodeprocess.readytospeak() == false);
}
TEST(ProcessOperation,ReceiveUserMessages)
{
	SpeakerNodeProcess m_speakernodeprocess = initialize_process();
	icarus_rover_v2::diagnostic diagnostic = m_speakernodeprocess.get_diagnostic();

	EXPECT_TRUE(diagnostic.Level < NOTICE);
	EXPECT_TRUE(diagnostic.Diagnostic_Message == NOERROR);
	EXPECT_TRUE(m_speakernodeprocess.is_finished_initializing() == true);
	EXPECT_TRUE(m_speakernodeprocess.readytospeak() == false);

	icarus_rover_v2::usermessage usermsg1,usermsg2,usermsg3,usermsg4,usermsg0;
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
	for(int i = 0; i < 10; i++)
	{
		diagnostic = m_speakernodeprocess.new_usermessage(usermsg1);
		EXPECT_TRUE(diagnostic.Level < NOTICE);
		EXPECT_TRUE(m_speakernodeprocess.get_usermessages(LEVEL1).size() == i+1);

		diagnostic = m_speakernodeprocess.new_usermessage(usermsg2);
		EXPECT_TRUE(diagnostic.Level < NOTICE);
		EXPECT_TRUE(m_speakernodeprocess.get_usermessages(LEVEL2).size() == i+1);

		diagnostic = m_speakernodeprocess.new_usermessage(usermsg3);
		EXPECT_TRUE(diagnostic.Level < NOTICE);
		EXPECT_TRUE(m_speakernodeprocess.get_usermessages(LEVEL3).size() == i+1);

		diagnostic = m_speakernodeprocess.new_usermessage(usermsg4);
		EXPECT_TRUE(diagnostic.Level < NOTICE);
		EXPECT_TRUE(m_speakernodeprocess.get_usermessages(LEVEL4).size() == i+1);

		diagnostic = m_speakernodeprocess.new_usermessage(usermsg0);
		EXPECT_TRUE(diagnostic.Level > NOTICE);
		EXPECT_TRUE(m_speakernodeprocess.get_usermessages(LEVEL4+1).size() == 0);
	}
}
TEST(ProcessOperation,SpeechPublish_Level1)
{
	std::vector<std::vector<unsigned char> > tx_buffers;
	SpeakerNodeProcess m_speakernodeprocess = initialize_process();
	icarus_rover_v2::diagnostic diagnostic = m_speakernodeprocess.get_diagnostic();

	EXPECT_TRUE(diagnostic.Level < NOTICE);
	EXPECT_TRUE(diagnostic.Diagnostic_Message == NOERROR);
	EXPECT_TRUE(m_speakernodeprocess.is_finished_initializing() == true);
	EXPECT_TRUE(m_speakernodeprocess.readytospeak() == false);
	EXPECT_TRUE(m_speakernodeprocess.isspeaking() == false);
	icarus_rover_v2::usermessage usermsg1,usermsg2,usermsg3,usermsg4,usermsg0;
	usermsg1.Level = LEVEL1;
	usermsg1.message = "Test_Level1";
	diagnostic = m_speakernodeprocess.new_usermessage(usermsg1);
	EXPECT_TRUE(diagnostic.Level < NOTICE);
	EXPECT_TRUE(m_speakernodeprocess.get_usermessages(LEVEL1).size() == 1);

	diagnostic = m_speakernodeprocess.update(0.01);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(m_speakernodeprocess.readytospeak() == true);
	EXPECT_TRUE(m_speakernodeprocess.isspeaking() == false);
	EXPECT_TRUE(m_speakernodeprocess.get_usermessages(LEVEL1).size() == 0);
	std::string speechout = m_speakernodeprocess.get_speechoutput();
	EXPECT_TRUE(speechout == usermsg1.message);
	diagnostic = m_speakernodeprocess.update(0.01);
	EXPECT_TRUE(m_speakernodeprocess.readytospeak() == false);
	EXPECT_TRUE(m_speakernodeprocess.isspeaking() == true);

	double time_to_finish = m_speakernodeprocess.get_timeleft_tofinishspeaking();
	double run_time = m_speakernodeprocess.get_runtime();
	while(run_time <= time_to_finish)
	{
		diagnostic = m_speakernodeprocess.update(0.01);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		run_time = m_speakernodeprocess.get_runtime();
	}
	diagnostic = m_speakernodeprocess.update(0.1);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(m_speakernodeprocess.isspeaking() == false);
}
TEST(ProcessOperation,SpeechPublish_Level1_Level2)
{
	std::string speechout;
	double run_time, time_to_finish;
	SpeakerNodeProcess m_speakernodeprocess = initialize_process();
	icarus_rover_v2::diagnostic diagnostic = m_speakernodeprocess.get_diagnostic();

	EXPECT_TRUE(diagnostic.Level < NOTICE);
	EXPECT_TRUE(diagnostic.Diagnostic_Message == NOERROR);
	EXPECT_EQ(m_speakernodeprocess.is_finished_initializing(),true);
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),false);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);
	icarus_rover_v2::usermessage usermsg1,usermsg2,usermsg3,usermsg4,usermsg0;
	usermsg2.Level = LEVEL2;
	usermsg2.message = "Test_Level2";
	usermsg1.Level = LEVEL1;
	usermsg1.message = "Test_Level1";
	diagnostic = m_speakernodeprocess.new_usermessage(usermsg2);
	diagnostic = m_speakernodeprocess.new_usermessage(usermsg1);
	EXPECT_TRUE(diagnostic.Level < NOTICE);
	EXPECT_EQ(m_speakernodeprocess.get_usermessages(LEVEL1).size(),1);
	EXPECT_EQ(m_speakernodeprocess.get_usermessages(LEVEL2).size(),1);
	diagnostic = m_speakernodeprocess.update(0.01); //Should speak Level2 Message first
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),true);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);
	speechout = m_speakernodeprocess.get_speechoutput();
	EXPECT_TRUE(speechout == usermsg2.message);
	diagnostic = m_speakernodeprocess.update(0.01);
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),false);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),true);

	time_to_finish = m_speakernodeprocess.get_timeleft_tofinishspeaking() + m_speakernodeprocess.get_runtime();
	run_time = m_speakernodeprocess.get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	while(run_time <= time_to_finish)
	{
		diagnostic = m_speakernodeprocess.update(0.01);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		run_time = m_speakernodeprocess.get_runtime();
	}
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);

	EXPECT_EQ(m_speakernodeprocess.get_usermessages(LEVEL1).size(),1);//Should speak Level1 Message next
	EXPECT_EQ(m_speakernodeprocess.ispaused(),true);
	while(m_speakernodeprocess.ispaused() == true)
	{
		diagnostic = m_speakernodeprocess.update(0.01);//Should be paused for a while
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),true);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);
	speechout = m_speakernodeprocess.get_speechoutput();
	EXPECT_TRUE(speechout == usermsg1.message);
	diagnostic = m_speakernodeprocess.update(0.01);
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),false);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),true);

	time_to_finish = m_speakernodeprocess.get_timeleft_tofinishspeaking() + m_speakernodeprocess.get_runtime();
	run_time = m_speakernodeprocess.get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	while(run_time <= time_to_finish)
	{
		diagnostic = m_speakernodeprocess.update(0.01);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		run_time = m_speakernodeprocess.get_runtime();
	}
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);
}
TEST(ProcessOperation,SpeechPublish_Level1_Level4)
{
	std::string speechout;
	double run_time, time_to_finish;
	SpeakerNodeProcess m_speakernodeprocess = initialize_process();
	icarus_rover_v2::diagnostic diagnostic = m_speakernodeprocess.get_diagnostic();

	EXPECT_TRUE(diagnostic.Level < NOTICE);
	EXPECT_TRUE(diagnostic.Diagnostic_Message == NOERROR);
	EXPECT_EQ(m_speakernodeprocess.is_finished_initializing(),true);
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),false);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);
	icarus_rover_v2::usermessage usermsg1,usermsg2,usermsg3,usermsg4,usermsg0;
	usermsg4.Level = LEVEL4;
	usermsg4.message = "Test_Level4";
	usermsg3.Level = LEVEL3;
	usermsg3.message = "Test_Level3";
	usermsg2.Level = LEVEL2;
	usermsg2.message = "Test_Level2";
	usermsg1.Level = LEVEL1;
	usermsg1.message = "Test_Level1";
	diagnostic = m_speakernodeprocess.new_usermessage(usermsg4);
	diagnostic = m_speakernodeprocess.new_usermessage(usermsg3);
	diagnostic = m_speakernodeprocess.new_usermessage(usermsg2);
	diagnostic = m_speakernodeprocess.new_usermessage(usermsg1);
	EXPECT_TRUE(diagnostic.Level < NOTICE);
	EXPECT_EQ(m_speakernodeprocess.get_usermessages(LEVEL1).size(),1);
	EXPECT_EQ(m_speakernodeprocess.get_usermessages(LEVEL2).size(),1);
	EXPECT_EQ(m_speakernodeprocess.get_usermessages(LEVEL3).size(),1);
	EXPECT_EQ(m_speakernodeprocess.get_usermessages(LEVEL4).size(),1);
	diagnostic = m_speakernodeprocess.update(0.01); //Should speak Level4 Message first
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),true);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);
	speechout = m_speakernodeprocess.get_speechoutput();
	EXPECT_TRUE(speechout == usermsg4.message);
	diagnostic = m_speakernodeprocess.update(0.01);
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),false);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),true);

	time_to_finish = m_speakernodeprocess.get_timeleft_tofinishspeaking() + m_speakernodeprocess.get_runtime();
	run_time = m_speakernodeprocess.get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	while(run_time <= time_to_finish)
	{
		diagnostic = m_speakernodeprocess.update(0.01);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		run_time = m_speakernodeprocess.get_runtime();
	}
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);

	EXPECT_EQ(m_speakernodeprocess.get_usermessages(LEVEL3).size(),1);//Should speak Level3 Message next
	EXPECT_EQ(m_speakernodeprocess.ispaused(),true);
	while(m_speakernodeprocess.ispaused() == true)
	{
		diagnostic = m_speakernodeprocess.update(0.01);//Should be paused for a while
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),true);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);
	speechout = m_speakernodeprocess.get_speechoutput();
	EXPECT_TRUE(speechout == usermsg3.message);
	diagnostic = m_speakernodeprocess.update(0.01);
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),false);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),true);

	time_to_finish = m_speakernodeprocess.get_timeleft_tofinishspeaking() + m_speakernodeprocess.get_runtime();
	run_time = m_speakernodeprocess.get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	while(run_time <= time_to_finish)
	{
		diagnostic = m_speakernodeprocess.update(0.01);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		run_time = m_speakernodeprocess.get_runtime();
	}
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);

	EXPECT_EQ(m_speakernodeprocess.get_usermessages(LEVEL2).size(),1);//Should speak Level2 Message next
	EXPECT_EQ(m_speakernodeprocess.ispaused(),true);
	while(m_speakernodeprocess.ispaused() == true)
	{
		diagnostic = m_speakernodeprocess.update(0.01);//Should be paused for a while
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),true);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);
	speechout = m_speakernodeprocess.get_speechoutput();
	EXPECT_TRUE(speechout == usermsg2.message);
	diagnostic = m_speakernodeprocess.update(0.01);
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),false);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),true);

	time_to_finish = m_speakernodeprocess.get_timeleft_tofinishspeaking() + m_speakernodeprocess.get_runtime();
	run_time = m_speakernodeprocess.get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	while(run_time <= time_to_finish)
	{
		diagnostic = m_speakernodeprocess.update(0.01);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		run_time = m_speakernodeprocess.get_runtime();
	}
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);

	EXPECT_EQ(m_speakernodeprocess.get_usermessages(LEVEL1).size(),1);//Should speak Level1 Message next
	EXPECT_EQ(m_speakernodeprocess.ispaused(),true);
	while(m_speakernodeprocess.ispaused() == true)
	{
		diagnostic = m_speakernodeprocess.update(0.01);//Should be paused for a while
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
	}
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),true);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);
	speechout = m_speakernodeprocess.get_speechoutput();
	EXPECT_TRUE(speechout == usermsg1.message);
	diagnostic = m_speakernodeprocess.update(0.01);
	EXPECT_EQ(m_speakernodeprocess.readytospeak(),false);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),true);

	time_to_finish = m_speakernodeprocess.get_timeleft_tofinishspeaking() + m_speakernodeprocess.get_runtime();
	run_time = m_speakernodeprocess.get_runtime();
	EXPECT_TRUE(run_time < time_to_finish);
	while(run_time <= time_to_finish)
	{
		diagnostic = m_speakernodeprocess.update(0.01);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		run_time = m_speakernodeprocess.get_runtime();
	}
	diagnostic = m_speakernodeprocess.update(0.1);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(m_speakernodeprocess.isspeaking(),false);
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
