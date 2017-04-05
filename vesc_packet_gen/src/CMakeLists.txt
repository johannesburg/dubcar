cmake_minimum_required(VERSION 2.6)
 
# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})



 
# Link runTests with what we want to test and the GTest and pthread library
add_executable(runTests ${packet_gen} bldc_interface.c bldc_interface.h buffer.c buffer.h crc.c crc.h datatypes.h packet.h test_packet_gen.cpp)
target_link_libraries(runTests ${GTEST_LIBRARIES} pthread)
