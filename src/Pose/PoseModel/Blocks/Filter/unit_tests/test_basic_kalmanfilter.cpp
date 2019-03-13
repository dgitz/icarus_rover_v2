#include <gtest/gtest.h>

#include "../BasicKalmanFilter.h"
#include "ros/ros.h"
#include "ros/time.h"
#include <iostream>
#include <fstream>
#include <string>
#include<boost/algorithm/string/split.hpp>
#include<boost/algorithm/string.hpp>
MatrixXf convert_str_to_matrix(int row_count,int column_count,std::string str)
{
	MatrixXf m(row_count,column_count);
	str.erase(str.begin());
	str.erase(str.end()-2);
	std::vector<std::string> rows;
	boost::split(rows,str,boost::is_any_of(";"),boost::token_compress_on);
	for(std::size_t r = 0; r < rows.size(); ++r)
	{
		std::vector<std::string> elements;
		boost::split(elements,rows.at(r),boost::is_any_of(" "));
		for(std::size_t c = 0; c < elements.size(); ++c)
		{
			m((int)r,(int)c) = std::atof(elements.at(c).c_str());
		}
	}
	return m;
}
VectorXf convert_str_to_vector(int length,std::string str)
{
	VectorXf v(length);
	str.erase(str.begin());
	str.erase(str.end()-1);
	std::vector<std::string> elements;
	boost::split(elements,str,boost::is_any_of(" ;"));
	for(std::size_t e = 0; e < elements.size(); ++e)
	{
		v((int)e) = std::atof(elements.at(e).c_str());
	}
	return v;
}
bool check_filterparameters(BasicKalmanFilter::FilterParameters kf, uint8_t input_count,uint8_t output_count)
{
	EXPECT_TRUE(kf.C.rows() == input_count);
	EXPECT_TRUE(kf.C.cols() == 1);
	EXPECT_TRUE(kf.G.rows() == 1);
	EXPECT_TRUE(kf.G.cols() == input_count);
	EXPECT_TRUE(kf.R.rows() == input_count);
	EXPECT_TRUE(kf.R.cols() == input_count);
	EXPECT_TRUE(kf.P.rows() == 1);
	EXPECT_TRUE(kf.P.cols() == 1);
	EXPECT_TRUE(kf.Q.rows() == 1);
	EXPECT_TRUE(kf.Q.cols() == 1);
	EXPECT_TRUE(kf.xhat.rows() == 1);
	EXPECT_TRUE(kf.xhat.cols() == 1);
	EXPECT_TRUE(kf.z.size() == input_count);
	EXPECT_TRUE(kf.I.rows() == 1);
	EXPECT_TRUE(kf.I.cols() == 1);
	return true;
}
double measure_time_diff(struct timeval t1,struct timeval t2)
{
	double a = t1.tv_sec + t1.tv_usec/1000000.0;
	double b = t2.tv_sec + t2.tv_usec/1000000.0;
	return b-a;
}
BasicKalmanFilter* initialize()
{
	BasicKalmanFilter *timecomp;
	return timecomp;
}

TEST(BasicKalmanFilter,Initialization)
{
}
TEST(BasicKalmanFilter,AutoGenUnitTestFiles)
{
	eros::diagnostic diagnostic;
	diagnostic.DeviceName = "Device1";
	diagnostic.Node_Name = "Node1";
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = POSE_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Filter Initializing";

	printf("Running Test 1\r\n");
	if(1)
	{
		std::vector<double> input_truth;
		std::vector<VectorXf> input_vector;
		std::vector<double> expected_P;
		std::vector<VectorXf> expected_G;
		std::vector<VectorXf> C;
		std::vector<VectorXf> w;
		std::vector<double> expected_Xhat;
		MatrixXf R;
		std::string filepath = "/home/robot/Dropbox/ICARUS/Scout/SIMULATION/Pose/UnitTests/KalmanFilter/KF_UnitTest1_Test.csv";
		std::ifstream myfile(filepath.c_str());
		std::string line;
		uint8_t input_count = 0;
		uint8_t output_count = 0;
		double Q = 0.0;

		bool input_param_section = true;
		bool input_data_section = false;
		bool output_param_section = false;
		bool output_data_section = false;
		int input_samples = 0;
		int output_samples = 0;
		int line_counter = 0;
		if (myfile.is_open())
		{
			while ( getline (myfile,line) )
			{
				line_counter++;

				std::vector<std::string> items;
				boost::split(items,line,boost::is_any_of(","),boost::token_compress_on);
				if(input_param_section == true)
				{
					if(items.at(0) == "Param 1:Input Count")
					{
						input_count = std::atoi(items.at(1).c_str());
					}
					else if(items.at(0) == "Param 2:Output Count")
					{
						output_count = std::atoi(items.at(1).c_str());
					}
					else if(items.at(0) == "Param 3:Q")
					{
						Q = std::atof(items.at(1).c_str());
					}
					else if(items.at(0) == "Param 4:[R]")
					{
						std::string tempstr = items.at(1);
						R = convert_str_to_matrix(input_count,input_count,tempstr);
						getline (myfile,line);
						line_counter++;
						getline (myfile,line);
						line_counter++;
						input_param_section = false;
						input_data_section = true;
					}
				}

				else if(input_data_section == true)
				{
					if(items.at(0).compare("Output") > 0)
					{
						getline (myfile,line);
						line_counter++;
						input_data_section = false;
						output_param_section = false;
						output_data_section = true;
					}
					else
					{
						int k = std::atoi(items.at(0).c_str());
						VectorXf wk;
						wk.resize(input_count);
						wk = convert_str_to_vector(input_count,items.at(1));
						input_truth.push_back(std::atof(items.at(2).c_str()));
						VectorXf input;
						input.resize(input_count);
						for( std::size_t i = 3; i < items.size()-1; ++i)
						{
							input((int)i-3) = std::atof(items.at(i).c_str());
						}
						input_vector.push_back(input);
						VectorXf Ck = convert_str_to_vector(input_count,items.at(items.size()-1));
						C.push_back(Ck);
						w.push_back(wk);
						input_samples++;
					}
				}
				else if(output_data_section == true)
				{
					int k = std::atoi(items.at(0).c_str());
					expected_P.push_back(std::atof(items.at(1).c_str()));
					VectorXf Gk = convert_str_to_vector(input_count,items.at(2));
					expected_G.push_back(Gk);
					expected_Xhat.push_back(std::atof(items.at(3).c_str()));
					output_samples++;

				}
			}
			myfile.close();
		}
		else
		{
			std::cout << "Unable to open file";
		}
		printf("Filter Parameters:\n");
		printf("Input: %d Output: %d Q=%4.2f\r\n",input_count,output_count,Q);
		std::cout << "R: " << std::endl << R << std::endl;
		EXPECT_TRUE(input_samples > 0);
		EXPECT_TRUE(input_samples == output_samples);
		BasicKalmanFilter kf;
		diagnostic = kf.initialize(diagnostic,
				input_count,
				output_count,
				expected_Xhat.at(0),
				R,
				C.at(0),
				Q);
		EXPECT_TRUE(check_filterparameters(kf.get_filter(),input_count,output_count));
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		struct timeval start,stop;
		gettimeofday(&start,NULL);
		for(int k = 1; k < input_samples; ++k)
		{
			double out = kf.update(input_vector.at(k),w.at(k));
			double error = fabs(out-expected_Xhat.at(k));
			//printf("[%d] out: %4.2f expected: %4.2f\n",k,out,expected_Xhat.at(k));
			BasicKalmanFilter::FilterParameters filter_param = kf.get_filter();
			EXPECT_TRUE(error < .0001);
		}
		gettimeofday(&stop,NULL);
		printf("Time per Filter Update: %f\n",measure_time_diff(start,stop)/(double)(input_samples));

	}
	printf("Running Test 2\r\n");
	if(1)
	{
		std::vector<double> input_truth;
		std::vector<VectorXf> input_vector;
		std::vector<double> expected_P;
		std::vector<VectorXf> expected_G;
		std::vector<VectorXf> C;
		std::vector<VectorXf> w;
		std::vector<double> expected_Xhat;
		MatrixXf R;
		std::string filepath = "/home/robot/Dropbox/ICARUS/Scout/SIMULATION/Pose/UnitTests/KalmanFilter/KF_UnitTest2_Test.csv";
		std::ifstream myfile(filepath.c_str());
		std::string line;
		uint8_t input_count = 0;
		uint8_t output_count = 0;
		double Q = 0.0;

		bool input_param_section = true;
		bool input_data_section = false;
		bool output_param_section = false;
		bool output_data_section = false;
		int input_samples = 0;
		int output_samples = 0;
		int line_counter = 0;
		if (myfile.is_open())
		{
			while ( getline (myfile,line) )
			{
				line_counter++;

				std::vector<std::string> items;
				boost::split(items,line,boost::is_any_of(","),boost::token_compress_on);
				if(input_param_section == true)
				{
					if(items.at(0) == "Param 1:Input Count")
					{
						input_count = std::atoi(items.at(1).c_str());
					}
					else if(items.at(0) == "Param 2:Output Count")
					{
						output_count = std::atoi(items.at(1).c_str());
					}
					else if(items.at(0) == "Param 3:Q")
					{
						Q = std::atof(items.at(1).c_str());
					}
					else if(items.at(0) == "Param 4:[R]")
					{
						std::string tempstr = items.at(1);
						R = convert_str_to_matrix(input_count,input_count,tempstr);
						getline (myfile,line);
						line_counter++;
						getline (myfile,line);
						line_counter++;
						input_param_section = false;
						input_data_section = true;
					}
				}

				else if(input_data_section == true)
				{
					if(items.at(0).compare("Output") > 0)
					{
						getline (myfile,line);
						line_counter++;
						input_data_section = false;
						output_param_section = false;
						output_data_section = true;
					}
					else
					{
						int k = std::atoi(items.at(0).c_str());
						VectorXf wk;
						wk.resize(input_count);
						wk = convert_str_to_vector(input_count,items.at(1));
						input_truth.push_back(std::atof(items.at(2).c_str()));
						VectorXf input;
						input.resize(input_count);
						for( std::size_t i = 3; i < items.size()-1; ++i)
						{
							input((int)i-3) = std::atof(items.at(i).c_str());
						}
						input_vector.push_back(input);
						VectorXf Ck = convert_str_to_vector(input_count,items.at(items.size()-1));
						C.push_back(Ck);
						w.push_back(wk);
						input_samples++;
					}
				}
				else if(output_data_section == true)
				{
					int k = std::atoi(items.at(0).c_str());
					expected_P.push_back(std::atof(items.at(1).c_str()));
					VectorXf Gk = convert_str_to_vector(input_count,items.at(2));
					expected_G.push_back(Gk);
					expected_Xhat.push_back(std::atof(items.at(3).c_str()));
					output_samples++;

				}
			}
			myfile.close();
		}
		else
		{
			std::cout << "Unable to open file";
		}
		printf("Filter Parameters:\n");
		printf("Input: %d Output: %d Q=%4.2f\r\n",input_count,output_count,Q);
		std::cout << "R: " << std::endl << R << std::endl;
		EXPECT_TRUE(input_samples > 0);
		EXPECT_TRUE(input_samples == output_samples);
		BasicKalmanFilter kf;
		diagnostic = kf.initialize(diagnostic,
				input_count,
				output_count,
				expected_Xhat.at(0),
				R,
				C.at(0),
				Q);
		EXPECT_TRUE(check_filterparameters(kf.get_filter(),input_count,output_count));
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		struct timeval start,stop;
		gettimeofday(&start,NULL);
		for(int k = 1; k < input_samples; ++k)
		{
			double out = kf.update(input_vector.at(k),w.at(k));
			double error = fabs(out-expected_Xhat.at(k));
			//printf("[%d] out: %4.2f expected: %4.2f\n",k,out,expected_Xhat.at(k));
			BasicKalmanFilter::FilterParameters filter_param = kf.get_filter();
			EXPECT_TRUE(error < .0001);
		}
		gettimeofday(&stop,NULL);
		printf("Time per Filter Update: %f\n",measure_time_diff(start,stop)/(double)(input_samples));

	}
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

