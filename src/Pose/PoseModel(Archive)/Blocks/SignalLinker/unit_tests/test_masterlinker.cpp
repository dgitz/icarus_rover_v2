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
void print_linearaccelerations(MasterLinker* linker)
{
	printf("--- LINEAR ACC INPUTS ---\n");
	std::vector<InputSignal_3d> linear_acc_inputs = linker->get_linearaccelerations();
	
	for(std::size_t i = 0; i < linear_acc_inputs.size(); ++i)
	{
		printf("\t[%d/%d] Instance: %s X: %f Update Count: %ld Y: %f Update Count: %ld Z: %f Update Count: %ld\n",
			(int)i+1,(int)linear_acc_inputs.size(),linear_acc_inputs.at(i).instance_name.c_str(),
			linear_acc_inputs.at(i).x.value,
			linear_acc_inputs.at(i).x_update_count,
			linear_acc_inputs.at(i).y.value,
			linear_acc_inputs.at(i).y_update_count,
			linear_acc_inputs.at(i).z.value,
			linear_acc_inputs.at(i).z_update_count);
	}
}
void print_rotationrates(MasterLinker* linker)
{
	printf("--- ROTATION RATE INPUTS ---\n");
	std::vector<InputSignal_3d> rotation_rate_inputs = linker->get_rotationrates();
	
	for(std::size_t i = 0; i < rotation_rate_inputs.size(); ++i)
	{
		printf("\t[%d/%d] Instance: %s X: %f Update Count: %ld Y: %f Update Count: %ld Z: %f Update Count: %ld\n",
			(int)i+1,(int)rotation_rate_inputs.size(),rotation_rate_inputs.at(i).instance_name.c_str(),
			rotation_rate_inputs.at(i).x.value,
			rotation_rate_inputs.at(i).x_update_count,
			rotation_rate_inputs.at(i).y.value,
			rotation_rate_inputs.at(i).y_update_count,
			rotation_rate_inputs.at(i).z.value,
			rotation_rate_inputs.at(i).z_update_count);
	}
}
void print_orientations(MasterLinker* linker)
{
	printf("--- ORIENTATION INPUTS ---\n");
	std::vector<InputSignal_3d> orientations = linker->get_orientations();
	
	for(std::size_t i = 0; i < orientations.size(); ++i)
	{
		printf("\t[%d/%d] Instance: %s X: %f Update Count: %ld Y: %f Update Count: %ld Z: %f Update Count: %ld\n",
			(int)i+1,(int)orientations.size(),orientations.at(i).instance_name.c_str(),
			orientations.at(i).x.value,
			orientations.at(i).x_update_count,
			orientations.at(i).y.value,
			orientations.at(i).y_update_count,
			orientations.at(i).z.value,
			orientations.at(i).z_update_count);
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
		printf("t: %f\n",dummydata.at(0).at(t).signal.tov);
		print_linearaccelerations(linker);
		print_rotationrates(linker);
		print_orientations(linker);
	}
}
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
