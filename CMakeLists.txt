cmake_minimum_required(VERSION 3.0)
project (mavlink_udp)
include_directories(include
lib)

set(mavlink_src
src/mavlink_udp.cpp
src/rc_mocap_tracking.cpp
include/rc/mavlink_udp.h
include/rc/DataStreamClient.h) 

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(rc_mocap_tracking ${mavlink_src})
target_link_libraries(rc_mocap_tracking Ws2_32.lib  ${PROJECT_SOURCE_DIR}/lib/ViconDataStreamSDK_CPP.lib )

