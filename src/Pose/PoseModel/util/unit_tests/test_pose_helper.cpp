#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../PoseHelper.h"



PoseHelper* initialize()
{
	PoseHelper *posehelper;
	return posehelper;
}
TEST(PoseHelper,Functions)
{
	PoseHelper *helper = initialize();
	{//Variance-Covariance Matrix: https://stattrek.com/matrix-algebra/covariance-matrix.aspx
		uint8_t sample_count = 5;
		MatrixXf expected_R(3,3);
		std::vector<double> x1 ={90.0,90.0,60.0,60.0,30.0};
		std::vector<double> x2 ={60.0,90.0,60.0,60.0,30.0};
		std::vector<double> x3 ={90.0,30.0,60.0,90.0,30.0};
		expected_R << 504.0,360.0,180.0,
					  360.0,360.0,0.0,
					  180.0,0.0,720.0;
		std::vector<std::vector<double> > inputs;
		inputs.push_back(x1);
		inputs.push_back(x2);
		inputs.push_back(x3);
		MatrixXf R = helper->compute_variance_covariance_matrix(inputs);
		EXPECT_TRUE(R.rows() == (uint8_t)inputs.size());
		EXPECT_TRUE(R.cols() == (uint8_t)inputs.size());
		for(uint8_t r = 0; r < (uint8_t)inputs.size(); ++r)
		{
			for(uint8_t c = 0; c < (uint8_t)inputs.size(); ++c)
			{
				double error = R(r,c)-expected_R(r,c);
				EXPECT_TRUE(fabs(error) < .00001);
			}
		}

	}


}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

