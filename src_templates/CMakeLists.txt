add_library(sampledriver Driver/SampleDriver.cpp)

#Sample_Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/sample_node)
add_library(sample_node_process SampleNodeProcess.cpp)
target_link_libraries(sample_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${FuzzyLiteLibrary} sampledriver)
add_executable(sample_node sample_node.cpp)
target_link_libraries(sample_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} ${FuzzyLiteLibrary} logger resourcemonitor sample_node_process)
catkin_add_gtest(test_sample_node_process unit_tests/test_sample_node_process.cpp)
target_link_libraries(test_sample_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${FuzzyLiteLibrary} sample_node_process)

if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l) 
    message("[Sample Node] Specific build instructions for Raspberry Pi.")
endif ()
if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL x86_64) 
    message("[Sample Node] Specific build instructions for x86/x64.")
endif ()