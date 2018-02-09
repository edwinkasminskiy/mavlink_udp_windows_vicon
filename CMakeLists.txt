cmake_minimum_required(VERSION 3.0)
project (mavlink_udp)
include_directories(include)
set(mavlink_src
src/mavlink_udp.c
src/rc_test_mavlink_heartbeat.c
include/rc/mavlink_udp.h
) 

add_executable(rc_test_mavlink_heartbeat ${mavlink_src})
target_link_libraries(rc_test_mavlink_heartbeat Ws2_32.lib)
