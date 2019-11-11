#SignalLinker Blocks
add_library(master_linker MasterLinker.cpp SignalSplitter.cpp)

catkin_add_gtest(test_masterlinker unit_tests/test_masterlinker.cpp)
target_link_libraries(test_masterlinker ${catkin_LIBRARIES} ${Boost_LIBRARIES} create_dummydata master_linker)
