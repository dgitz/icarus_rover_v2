#Pose Node
include_directories("Pose_AutoCode_grt_rtw/")
include_directories("R2019b/extern/include/")
include_directories("R2019b/simulink/include/")
add_library(pose_library 
    Pose_AutoCode_grt_rtw/Pose_AutoCode.cpp 
    Pose_AutoCode_grt_rtw/Pose_AutoCode_data.cpp 
    Pose_AutoCode_grt_rtw/rt_nonfinite.cpp
    Pose_AutoCode_grt_rtw/rtGetInf.cpp
    Pose_AutoCode_grt_rtw/rtGetNaN.cpp)
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/pose_node)
add_library(pose_node_process PoseNodeProcess.cpp)
add_dependencies(pose_node_process eros_generate_messages_cpp)
target_link_libraries(pose_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} pose_library)
add_executable(pose_node pose_node.cpp)
target_link_libraries(pose_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} pose_node_process logger resourcemonitor pose_library)
catkin_add_gtest(test_pose_node_process unit_tests/test_pose_node_process.cpp)
target_link_libraries(test_pose_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${TinyXML_LIBRARIES} pose_node_process logger)

#Calibration Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/calibration_node)
add_library(calibration_node_process CalibrationNodeProcess.cpp)
add_dependencies(calibration_node_process eros_generate_messages_cpp)
add_executable(calibration_node calibration_node.cpp)
target_link_libraries(calibration_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} calibration_node_process logger resourcemonitor)
catkin_add_gtest(test_calibration_node_process unit_tests/test_calibration_node_process.cpp)
target_link_libraries(test_calibration_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${TinyXML_LIBRARIES} calibration_node_process logger)
