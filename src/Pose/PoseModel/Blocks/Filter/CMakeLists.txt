#Timing Blocks
add_library(basic_kalmanfilter BasicKalmanFilter.cpp)
catkin_add_gtest(test_basic_kalmanfilter unit_tests/test_basic_kalmanfilter.cpp)
target_link_libraries(test_basic_kalmanfilter ${catkin_LIBRARIES} ${Boost_LIBRARIES} basic_kalmanfilter)
