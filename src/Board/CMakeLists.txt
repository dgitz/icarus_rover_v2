if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)
    add_library(servohatdriver Driver/ServoHatDriver.cpp)
    add_library(gpiohatdriver Driver/GPIOHatDriver.cpp)
    add_library(terminalhatdriver Driver/TerminalHatDriver.cpp)
endif ()
#Board Controller Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/boardcontroller_node)
add_library(boardcontroller_node_process BoardControllerNodeProcess.cpp)
add_dependencies(boardcontroller_node_process eros_generate_messages_cpp)
target_link_libraries(boardcontroller_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
add_executable(boardcontroller_node boardcontroller_node.cpp)
target_link_libraries(boardcontroller_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} spimessage boardcontroller_node_process logger resourcemonitor)
catkin_add_gtest(test_boardcontroller_node_process unit_tests/test_boardcontroller_node_process.cpp)
target_link_libraries(test_boardcontroller_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${TinyXML_LIBRARIES} spimessage boardcontroller_node_process logger)
#add_executable(test_arduinoboard unit_tests/test_arduinoboard.cpp)
#target_link_libraries(test_arduinoboard spimessage)
add_dependencies(boardcontroller_node_process ${catkin_EXPORTED_TARGETS})
add_dependencies(boardcontroller_node ${catkin_EXPORTED_TARGETS})

#HatController Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/hatcontroller_node)
add_library(hatcontroller_node_process HatControllerNodeProcess.cpp)
add_dependencies(hatcontroller_node_process eros_generate_messages_cpp)
target_link_libraries(hatcontroller_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)
    add_executable(hatcontroller_node hatcontroller_node.cpp) #Only compatible for armv7l
    target_link_libraries(hatcontroller_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor wiringPi i2cmessage gpiohatdriver servohatdriver terminalhatdriver hatcontroller_node_process) #Only compatible for armv7l
    
    #Unit Test Programs
    add_executable(test_servohat unit_tests/test_servohat.cpp)
    target_link_libraries(test_servohat servohatdriver wiringPi)
    add_executable(test_terminalhat unit_tests/test_terminalhat.cpp)
    target_link_libraries(test_terminalhat terminalhatdriver wiringPi)
    add_executable(test_gpiohat unit_tests/test_gpiohat.cpp)
    target_link_libraries(test_gpiohat gpiohatdriver wiringPi i2cmessage)
endif ()
catkin_add_gtest(test_hatcontroller_node_process unit_tests/test_hatcontroller_node_process.cpp)
target_link_libraries(test_hatcontroller_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} hatcontroller_node_process logger)
