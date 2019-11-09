#include <gtest/gtest.h>

#include "../LinearAccelerationLinker.h"
#include "ros/ros.h"
#include "ros/time.h"

void print_diagnostic(uint8_t level, eros::diagnostic diagnostic)
{
	if (diagnostic.Level >= level)
	{
		printf("Type: %d Message: %d Level: %d Device: %s Desc: %s\n", diagnostic.Diagnostic_Type, diagnostic.Diagnostic_Message,
			   diagnostic.Level, diagnostic.DeviceName.c_str(), diagnostic.Description.c_str());
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

LinearAccelerationLinker *initialize_linearacceleration_linker(std::string name, uint8_t input_signal_count, uint8_t output_signal_count)
{
	eros::diagnostic diag;
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
	LinearAccelerationLinker *linker;
	linker = new LinearAccelerationLinker;
	EXPECT_TRUE(linker->initialize_object(name, diag));
	EXPECT_TRUE(linker->initialize_inputsignals(input_signals).Level <= NOTICE);
	EXPECT_TRUE(linker->initialize_outputsignals(output_signals).Level <= NOTICE);
	return linker;
}
TEST(LinearAccelerationLinker, Execution)
{
	eros::diagnostic diag;
	uint8_t sensor_count = 2;
	{ //Initialize with 2 IMU's of data
		LinearAccelerationLinker *xacc_linker = initialize_linearacceleration_linker("X_Acc", sensor_count, sensor_count);
		{
			EXPECT_TRUE(xacc_linker->get_inputsignals().size() == xacc_linker->get_outputsignals().size());
			EXPECT_TRUE(xacc_linker->get_inputsignals().size() > 0);
		}
		LinearAccelerationLinker *yacc_linker = initialize_linearacceleration_linker("Y_Acc", sensor_count, sensor_count);
		{
			EXPECT_TRUE(yacc_linker->get_inputsignals().size() == yacc_linker->get_outputsignals().size());
			EXPECT_TRUE(yacc_linker->get_inputsignals().size() > 0);
		}
		LinearAccelerationLinker *zacc_linker = initialize_linearacceleration_linker("Z_Acc", sensor_count, sensor_count);
		{
			EXPECT_TRUE(zacc_linker->get_inputsignals().size() == zacc_linker->get_outputsignals().size());
			EXPECT_TRUE(zacc_linker->get_inputsignals().size() > 0);
		}
		double time_to_run = 1.0;
		double elap_time = 0.0;
		double dt = 0.1;
		xacc_linker->print_inputs();
		xacc_linker->print_outputs();
		while (elap_time < time_to_run)
		{
			std::vector<eros::signal> xacc_inputs;
			std::vector<eros::signal> yacc_inputs;
			std::vector<eros::signal> zacc_inputs;
			{// Test xacc linker inputs
				xacc_inputs = xacc_linker->get_inputsignals();
				for(std::size_t i = 0; i < xacc_inputs.size(); ++i)
				{
					xacc_inputs.at(i).value = 1.123 + (double)(i*12.0)+dt;
					diag = xacc_linker->new_inputsignal(xacc_inputs.at(i));
					EXPECT_TRUE(diag.Level <= NOTICE);
				}
				std::vector<eros::signal> input_signals = xacc_linker->get_inputsignals();
				for(std::size_t i = 0; i < input_signals.size(); ++i)
				{
					EXPECT_TRUE(isequal(xacc_inputs.at(i).value,input_signals.at(i).value,.00001));
				}
			}
			{// Test yacc linker inputs
				yacc_inputs = yacc_linker->get_inputsignals();
				for(std::size_t i = 0; i < yacc_inputs.size(); ++i)
				{
					yacc_inputs.at(i).value = 2.457 + (double)(i*17.0)+dt;
					diag = yacc_linker->new_inputsignal(yacc_inputs.at(i));
					EXPECT_TRUE(diag.Level <= NOTICE);
				}
				std::vector<eros::signal> input_signals = yacc_linker->get_inputsignals();
				for(std::size_t i = 0; i < yacc_inputs.size(); ++i)
				{
					EXPECT_TRUE(isequal(yacc_inputs.at(i).value,input_signals.at(i).value,.00001));
				}
			}
			{// Test zacc linker inputs
				zacc_inputs = zacc_linker->get_inputsignals();
				for(std::size_t i = 0; i < zacc_inputs.size(); ++i)
				{
					zacc_inputs.at(i).value = 17.345 + (double)(-i*12.0)+dt;
					diag = zacc_linker->new_inputsignal(zacc_inputs.at(i));
					EXPECT_TRUE(diag.Level <= NOTICE);
				}
				std::vector<eros::signal> input_signals = zacc_linker->get_inputsignals();
				for(std::size_t i = 0; i < input_signals.size(); ++i)
				{
					EXPECT_TRUE(isequal(zacc_inputs.at(i).value,input_signals.at(i).value,.00001));
				}
			}
			diag = xacc_linker->update(dt, elap_time);
			EXPECT_TRUE(diag.Level <= NOTICE);
			diag = yacc_linker->update(dt, elap_time);
			EXPECT_TRUE(diag.Level <= NOTICE);
			diag = zacc_linker->update(dt, elap_time);
			EXPECT_TRUE(diag.Level <= NOTICE);
			{// Test xacc linker outputs
				std::vector<eros::signal> outputs = xacc_linker->get_outputsignals();
				for(std::size_t i = 0; i < outputs.size(); ++i)
				{
					EXPECT_TRUE(isequal(outputs.at(i).value,xacc_inputs.at(i).value,.00001));
				}
			}
			{// Test yacc linker outputs
				std::vector<eros::signal> outputs = yacc_linker->get_outputsignals();
				for(std::size_t i = 0; i < outputs.size(); ++i)
				{
					EXPECT_TRUE(isequal(outputs.at(i).value,yacc_inputs.at(i).value,.00001));
				}
			}
			{// Test zacc linker outputs
				std::vector<eros::signal> outputs = zacc_linker->get_outputsignals();
				for(std::size_t i = 0; i < outputs.size(); ++i)
				{
					EXPECT_TRUE(isequal(outputs.at(i).value,zacc_inputs.at(i).value,.00001));
				}
			}	
			elap_time += dt;
		}
	}
}
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
