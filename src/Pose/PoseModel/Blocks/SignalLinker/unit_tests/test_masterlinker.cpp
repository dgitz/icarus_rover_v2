#include <gtest/gtest.h>

#include "../MasterLinker.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "../../../../Development/DummyData/CreateDummyData.h"

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
bool isequal(double a, double b, double precision)
{
	double v = fabs(a-b);
	if(v < precision)
	{
		return true;
	}
	else
	{
		return false;
	}
}

MasterLinker *initialize_master_linker(std::string name, uint8_t input_signal_count, uint8_t output_signal_count)
{
	/*
	
	std::vector<eros::signal> input_signals;
	std::vector<eros::signal> output_signals;
	for (uint8_t i = 0; i < input_signal_count; ++i)
	{
		eros::signal sig;
		sig.name = "/IMU" + std::to_string(i + 1) + "/" + name;
		input_signals.push_back(sig);
	}
	for (uint8_t i = 0; i < output_signal_count; ++i)
	{
		eros::signal sig;
		sig.name = "/IMU" + std::to_string(i + 1) + "/" + name;
		output_signals.push_back(sig);
	}
	*/
	eros::diagnostic diag;
	diag.DeviceName = "unittest";
	diag.Node_Name = "unittest";
	diag.System = ROVER;
	diag.SubSystem = ROBOT_CONTROLLER;
	diag.Component = POSE_NODE;
	diag.Diagnostic_Type = DATA_STORAGE;
	MasterLinker *linker;
	linker = new MasterLinker;
	EXPECT_TRUE(linker->initialize_object(name, diag));
	//EXPECT_TRUE(linker->initialize_inputsignals(input_signals).Level <= NOTICE);
	//EXPECT_TRUE(linker->initialize_outputsignals(output_signals).Level <= NOTICE);
	return linker;
}
TEST(MasterLinker, Execution)
{
	eros::diagnostic diag;
	MasterLinker *linker = initialize_master_linker("TestLinker",0,0);
	CreateDummyData dummydata_obj;
	std::vector<std::vector<PostProcessedSignal> > dummydata = dummydata_obj.Create_ProcessedSignalVector();
	for(std::size_t t = 0; t < dummydata.at(0).size(); ++t)
	{
		std::vector<PostProcessedSignal> input_signals;
		for(std::size_t i = 0; i < dummydata.size(); ++i)
		{
			input_signals.push_back(dummydata.at(i).at(t));
		}
		diag = linker->new_input(input_signals);
		EXPECT_TRUE(diag.Level <= NOTICE);
	}
}
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
