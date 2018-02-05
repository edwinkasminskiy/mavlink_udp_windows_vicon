/**
 * @file rc_test_mavlink_heartbeat
 *
 * @brief      Basic Mavlink UDP heartbeat tester.
 *
 *             Sends a heartbeat packets every second and prints to the screen
 *             when one has been received along with the system identifier of
 *             the device that sent the received heartbeat. Optionally specify
 *             the destination IP address to send to with the -a option,
 *             otherwise messages will be sent to 127.0.0.1 (localhost).
 *             Optionally specify the UDP port which will be used for listening
 *             and sending with the -p option, otherwise RC_MAV_DEFAULT_UDP_PORT
 *             (14551) will be used. Optionally specify the system ID which will
 *             be specified in each sent heartbeat packet to be read by the
 *             listener, otherwise a system ID of 1 will be used.
 *
 * @author     James Strawson & Henry Gaudet
 *
 * @date       1/24/2018
 */

#include <ctype.h> // for isprint()
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <signal.h> // to SIGINT signal handler
#include "../include/rc/mavlink_udp.h"
#include "../include/rc/mavlink_udp_helpers.h"
#include "../include/rc/DataStreamClient.h"

#define LOCALHOST_IP	"127.0.0.1"
#define DEFAULT_SYS_ID	1

const char* dest_ip;
uint8_t my_sys_id;
uint16_t port;
int running;


// interrupt handler to catch ctrl-c
void signal_handler(int dummy)
{
	running=0;
	return;
}

int main(int argc, char * argv[])
{
	// set default options before checking options
	dest_ip=LOCALHOST_IP;
	my_sys_id=DEFAULT_SYS_ID;
	port=RC_MAV_DEFAULT_UDP_PORT;

	// parse arguments
	/*
		if(parse_args(argc,argv)){
		fprintf(stderr,"failed to parse arguments\n");
		return -1;
	}*/


	printf("run with -h option to see usage and other options\n");
	// inform the user what settings are being used
	printf("\n");
	printf("Initializing with the following settings:\n");
	printf("dest ip addr: %s\n", dest_ip);
	printf("my system id: %d\n", my_sys_id);
	printf("UDP port: %d\n", port);
	printf("\n");

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, signal_handler);

	// initialize the UDP port and listening thread with the rc_mav lib
	if (rc_mav_init(my_sys_id, dest_ip, port) < 0){
		return -1;
	}
	//Set up client to read data from Vicon
	Client MyClient;

	running=1;
	while(running){
		//Sleep(1000);

		//Use Vicon SDK to get position data

		 class Output_GetSegmentStaticRotationQuaternion
		 {
		 public:
		 Result::Enum Result;
		 double Rotation[ 4 ];
		 };
		
		 Output_GetSegmentStaticRotationQuaternion
		 GetSegmentStaticRotationQuaternion(
		 const String & SubjectName,
		 const String & SegmentName ) const
		ViconDataStreamSDK::CPP::Client MyClient;
		MyClient.Connect("localhost");
		MyClient.GetFrame();
		Output_GetSegmentStaticRotationQuaternion Output =
			MyClient.GetSegmentStaticRotationQuaternion("Alice", "Pelvis");



		if(rc_mav_send_heartbeat_abbreviated()){
			fprintf(stderr,"failed to send heartbeat\n");
		}
		else{
			printf("sent heartbeat\n");
		}
		if (rc_mav_get_att_pos_mocap(quat, x_pos, y_pos, z_pos)) {
			fprintf(stderr, "failed to send position data\n");
		}
		else {
			printf("quaternion: \n"
				"x coordinate: \n"
				"y coordinate: \n"
				"z coordinate: \n");
		}
	}

	// stop listening thread and close UDP port
	printf("closing UDP port\n");
	rc_mav_cleanup();
}