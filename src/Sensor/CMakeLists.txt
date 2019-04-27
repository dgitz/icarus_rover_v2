#IMU Node
find_package(TinyXML REQUIRED)
file(GLOB IMU_SOURCES
    "Driver/110015/*.h"
    "Driver/110015/*.cpp"
)
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/imu_node)
add_library(imudriver ${IMU_SOURCES} Driver/IMUDriver.cpp)
add_library(imu_node_process IMUNodeProcess.cpp)
add_executable(imu_node imu_node.cpp)
target_link_libraries(imu_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} imudriver imu_node_process logger resourcemonitor)
catkin_add_gtest(test_imu_node_process unit_tests/test_imu_node_process.cpp)
target_link_libraries(test_imu_node_process imu_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${TinyXML_LIBRARIES}  logger)
