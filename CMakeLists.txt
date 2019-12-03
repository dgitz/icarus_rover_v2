#Target: dgitzrosmaster
cmake_minimum_required(VERSION 2.8.3)
project(icarus_rover_v2)
set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
SET(CMAKE_CXX_FLAGS "-std=c++0x")
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)
FIND_PACKAGE (Threads REQUIRED)
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
  rosbag
)

## Setup FuzzyLite Library
set(FL_SYSTEM_LIBRARY FALSE) #Finds the library installed in the system
set(FL_LIBRARY_DIR)

if (NOT FL_SYSTEM_LIBRARY)
    #it is possible to find the FuzzyLiteLibrary locally as follows,
    #assuming the submodule of fuzzylite is added to the ${PROJECT_SOURCE_DIR}:
    set(FL_HOME /home/robot/other_packages/fuzzylite-6.0/fuzzylite/)
    set(FL_INCLUDE_DIR ${FL_HOME})
    if (FL_DEBUG)
        set(FL_LIBRARY_DIR ${FL_HOME}/debug/bin)
    else()
        set(FL_LIBRARY_DIR ${FL_HOME}/release/bin)
    endif()
    message("Finding FuzzyLiteLibrary locally at ${FL_LIBRARY_DIR}")

    include_directories(${FL_INCLUDE_DIR})
endif()

set(FL_POSTFIX)
if (FL_STATIC)
    set(FL_POSTFIX "-static")
endif()
if (FL_DEBUG)
    set(FL_POSTFIX "${FL_POSTFIX}-debug")
endif()

set(FL_LIBRARY_NAME  fuzzylite${FL_POSTFIX})
find_library (FuzzyLiteLibrary ${FL_LIBRARY_NAME} HINTS ${FL_LIBRARY_DIR})

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
add_library(resourcemonitor util/resourcemonitor.cpp)
add_dependencies(resourcemonitor ${catkin_EXPORTED_TARGETS})
add_library(controlgroup util/controlgroup.cpp)


#Libraries-Communications
add_library(udpmessage util/udpmessage.cpp)
add_library(serialmessage util/serialmessage.cpp)
add_library(spimessage util/spimessage.cpp)
add_library(i2cmessage util/i2cmessage.cpp)

#Add CMakeLists for Templates
add_subdirectory(src_templates)

#Add CMakeLists for Board
add_subdirectory(src/Board)

#Add CMakeLists for Communication
add_subdirectory(src/Communication)

#Add CMakeLists for Control
add_subdirectory(src/Control)

#Add CMakeLists for Diagnostics
add_subdirectory(src/Diagnostics)

#Add CMakeLists for Pose
add_subdirectory(src/Pose)

#Add CMakeLists for Sensors
add_subdirectory(src/Sensor) 

#Add CMakeLists for Timing
add_subdirectory(src/Timing)

#Add CMakeLists for User Control
add_subdirectory(src/User_Control)
