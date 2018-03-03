cmake_minimum_required(VERSION 3.0)
project (mavlink_udp)
include_directories(include
lib)

set(mavlink_src
src/mavlink_udp.cpp
src/rc_test_mavlink_heartbeat.cpp
include/rc/mavlink_udp.h
include/rc/DataStreamClient.h
) 
#set(SOURCE ${PROJECT_SOURCE_DIR}/src)
#message(${SOURCE})

add_executable(rc_test_mavlink_heartbeat ${mavlink_src})
target_link_libraries(rc_test_mavlink_heartbeat Ws2_32.lib  ${PROJECT_SOURCE_DIR}/lib/ViconDataStreamSDK_CPP.lib )

