#SensorPostProcessing Blocks
add_library(base_postprocess BasePostProcess.cpp)
add_dependencies(base_postprocess eros_generate_messages_cpp)
add_library(imu_postprocess IMUPostProcess.cpp)
add_dependencies(imu_postprocess eros_generate_messages_cpp)
target_link_libraries(imu_postprocess ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} base_postprocess)
#add_library(basemachine_postprocess BaseMachinePostProcess.cpp)
#add_library(gps_postprocess GPSPostProcess.cpp)
#add_library(odometry_postprocess OdometryPostProcess.cpp)
#add_library(usercontrol_postprocess UserControlPostProcess.cpp)
#add_library(vision_postprocess VisionPostProcess.cpp)
catkin_add_gtest(test_sensor_postprocess unit_tests/test_sensor_postprocess.cpp)
target_link_libraries(test_sensor_postprocess ${catkin_LIBRARIES} ${Boost_LIBRARIES} 
    create_dummydata
    base_postprocess
    imu_postprocess)
#    basemachine_postprocess
#    gps_postprocess
#    odometry_postprocess
#    usercontrol_postprocess
#    vision_postprocess)
