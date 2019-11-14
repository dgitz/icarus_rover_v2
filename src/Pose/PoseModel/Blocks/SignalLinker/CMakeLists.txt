#SignalLinker Blocks

add_library(master_linker MasterLinker.cpp SignalSplitter.cpp 
    BaseLinker.cpp 
    LinearAccelerationLinker.cpp
    RotationRateLinker.cpp
    OrientationLinker.cpp)
target_link_libraries(master_linker ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES})
catkin_add_gtest(test_masterlinker unit_tests/test_masterlinker.cpp)
target_link_libraries(test_masterlinker ${catkin_LIBRARIES} ${Boost_LIBRARIES} create_dummydata master_linker)
