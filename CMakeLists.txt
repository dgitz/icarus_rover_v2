#Target: dgitzrosmaster
cmake_minimum_required(VERSION 2.8.3)
project(icarus_rover_v2)
set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
SET(CMAKE_CXX_FLAGS "-std=c++0x")
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  roslaunch
  message_generation
  sensor_msgs
  cv_bridge
  roslib
  tf
  eros
  rosbag
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(TinyXML REQUIRED)
find_package(OpenCV)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(icarus_rover_v2 thread signals system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES icarus_rover_v2
  CATKIN_DEPENDS roscpp rospy std_msgs roslib
  DEPENDS Boost message_runtime TinyXML
  
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ library



## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(icarus_rover_v2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable
# add_executable(icarus_rover_v2_node src/icarus_rover_v2_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(icarus_rover_v2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(icarus_rover_v2_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS icarus_rover_v2 icarus_rover_v2_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############
#roslaunch_add_file_check(/home/dgitz/catkin_ws/src/icarus_rover_v2/launch)
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_icarus_rover_v2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
include_directories(include ${catkin_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS})
#Libraries-General
add_library(logger util/logger.cpp)
add_library(resourcemonitor util/resourcemonitor.cpp)

#Libraries-Communications
add_library(udpmessage util/udpmessage.cpp)
add_library(serialmessage util/serialmessage.cpp)
add_library(spimessage util/spimessage.cpp)
add_library(i2cmessage util/i2cmessage.cpp)

#Libraries-Drivers
add_library(lcddriver src/Diagnostics/Driver/LCDDriver.cpp)
#add_library(terminalhatdriver src/Board/Driver/TerminalHatDriver.cpp) #Only compatible for armv7l
#add_library(gpiohatdriver src/Board/Driver/GPIOHatDriver.cpp) #Only compatible for armv7l
#add_library(servohatdriver src/Board/Driver/ServoHatDriver.cpp) #Only compatible for armv7l
add_library(imudriver src/Sensor/Driver/IMUDriver.cpp)



#Sample_Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/sample_node)
add_library(sample_node_process src_templates/SampleNodeProcess.cpp)
add_executable(sample_node src_templates/sample_node.cpp)
target_link_libraries(sample_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor sample_node_process)
catkin_add_gtest(test_sample_node_process src_templates/unit_tests/test_sample_node_process.cpp)
target_link_libraries(test_sample_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} sample_node_process)



#TimeMaster Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/timemaster_node)
add_library(timemaster_node_process src/Timing/TimeMasterNodeProcess.cpp)
add_executable(timemaster_node src/Timing/timemaster_node.cpp)
target_link_libraries(timemaster_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor timemaster_node_process)
catkin_add_gtest(test_timemaster_node_process src/Timing/unit_tests/test_timemaster_node_process.cpp)
target_link_libraries(test_timemaster_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} timemaster_node_process logger serialmessage)

#TimeSlave Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/timeslave_node)
add_library(timeslave_node_process src/Timing/TimeSlaveNodeProcess.cpp)
add_executable(timeslave_node src/Timing/timeslave_node.cpp)
target_link_libraries(timeslave_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor timeslave_node_process)
catkin_add_gtest(test_timeslave_node_process src/Timing/unit_tests/test_timeslave_node_process.cpp)
target_link_libraries(test_timeslave_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} timeslave_node_process logger serialmessage)

#Network Transceiver Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/networktransceiver_node)
add_library(networktransceiver_node_process src/Communication/NetworkTransceiverNodeProcess.cpp)
add_executable(networktransceiver_node src/Communication/networktransceiver_node.cpp)
target_link_libraries(networktransceiver_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger udpmessage resourcemonitor networktransceiver_node_process)
catkin_add_gtest(test_networktransceiver_node_process src/Communication/unit_tests/test_networktransceiver_node_process.cpp)
target_link_libraries(test_networktransceiver_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} networktransceiver_node_process logger serialmessage)

#HatController Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/hatcontroller_node)
add_library(hatcontroller_node_process src/Board/HatControllerNodeProcess.cpp)
#add_executable(hatcontroller_node src/Board/hatcontroller_node.cpp) #Only compatible for armv7l
#target_link_libraries(hatcontroller_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor wiringPi i2cmessage gpiohatdriver servohatdriver terminalhatdriver hatcontroller_node_process) #Only compatible for armv7l
catkin_add_gtest(test_hatcontroller_node_process src/Board/unit_tests/test_hatcontroller_node_process.cpp)
target_link_libraries(test_hatcontroller_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} hatcontroller_node_process logger)

#Audio Node
#file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/audio_node)
#add_library(audio_node_process src/User_Control/AudioNodeProcess.cpp)
#add_executable(audio_node src/User_Control/audio_node.cpp) 
#target_link_libraries(audio_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor audio_node_process)
#catkin_add_gtest(test_audio_node_process src/User_Control/unit_tests/test_audio_node_process.cpp)
#target_link_libraries(test_audio_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} audio_node_process)

#Topic Remapper Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/topicremapper_node)
add_library(topicremapper_node_process src/Communication/TopicRemapperNodeProcess.cpp)
add_executable(topicremapper_node src/Communication/topicremapper_node.cpp)
target_link_libraries(topicremapper_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} logger resourcemonitor topicremapper_node_process)
catkin_add_gtest(test_topicremapper_node_process src/Communication/unit_tests/test_topicremapper_node_process.cpp)
target_link_libraries(test_topicremapper_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} topicremapper_node_process logger)

#Board Controller Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/boardcontroller_node)
add_library(boardcontroller_node_process src/Board/BoardControllerNodeProcess.cpp)
add_executable(boardcontroller_node src/Board/boardcontroller_node.cpp)
target_link_libraries(boardcontroller_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} spimessage boardcontroller_node_process logger resourcemonitor)
catkin_add_gtest(test_boardcontroller_node_process src/Board/unit_tests/test_boardcontroller_node_process.cpp)
target_link_libraries(test_boardcontroller_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${TinyXML_LIBRARIES} spimessage boardcontroller_node_process logger)

#IMU Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/imu_node)
add_library(imu_node_process src/Sensor/IMUNodeProcess.cpp)
add_executable(imu_node src/Sensor/imu_node.cpp)
target_link_libraries(imu_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} imu_node_process imudriver logger resourcemonitor)
catkin_add_gtest(test_imu_node_process src/Sensor/unit_tests/test_imu_node_process.cpp)
target_link_libraries(test_imu_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${TinyXML_LIBRARIES} imu_node_process logger)

#Add CMakeLists for Pose
add_subdirectory(src/Pose)

#Add CMakeLists for Control
add_subdirectory(src/Control)

#Add CMakeLists for Diagnostics
add_subdirectory(src/Diagnostics)