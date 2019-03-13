#SensorPostProcessing Blocks
add_library(imu_postprocess IMUPostProcess.cpp)

catkin_add_gtest(test_sensor_postprocess unit_tests/test_sensor_postprocess.cpp)
target_link_libraries(test_sensor_postprocess ${catkin_LIBRARIES} ${Boost_LIBRARIES} imu_postprocess)
