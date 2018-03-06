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

#define output_stream std::cout 
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
	using namespace ViconDataStreamSDK::CPP;
	Client MyClient;
	MyClient.Connect("localhost:801");

	MyClient.SetAxisMapping(Direction::Forward,
		Direction::Left,
		Direction::Up); // Z-up
	MyClient.EnableMarkerData();
	MyClient.EnableSegmentData();
	Output_GetSubjectCount OutputGSC;
	OutputGSC = MyClient.GetSubjectCount();
	Output_GetSubjectName OutputGSN;
	unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
	output_stream << "Subjects (" << SubjectCount << "):" << std::endl;
	for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
	{
		output_stream << "  Subject #" << SubjectIndex << std::endl;

		// Get the subject name
		std::string SubjectName = MyClient.GetSubjectName(SubjectIndex).SubjectName;
		output_stream << "    Name: " << SubjectName << std::endl;

		// Get the root segment
		std::string RootSegment = MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
		output_stream << "    Root Segment: " << RootSegment << std::endl;

		// Count the number of segments
		unsigned int SegmentCount = MyClient.GetSegmentCount(SubjectName).SegmentCount;
		output_stream << "    Segments (" << SegmentCount << "):" << std::endl;
		for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
		{
			output_stream << "      Segment #" << SegmentIndex << std::endl;

			// Get the segment name
			std::string SegmentName = MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
			output_stream << "        Name: " << SegmentName << std::endl;

			// Get the segment parent
			std::string SegmentParentName = MyClient.GetSegmentParentName(SubjectName, SegmentName).SegmentName;
			output_stream << "        Parent: " << SegmentParentName << std::endl;

			// Get the segment's children
			unsigned int ChildCount = MyClient.GetSegmentChildCount(SubjectName, SegmentName).SegmentCount;
			output_stream << "     Children (" << ChildCount << "):" << std::endl;
			for (unsigned int ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
			{
				std::string ChildName = MyClient.GetSegmentChildName(SubjectName, SegmentName, ChildIndex).SegmentName;
				output_stream << "       " << ChildName << std::endl;
			}
		}
	}
	/*
	std::cout << "Current Subjects:" << std::endl;

	for (int i = 0; i < OutputGSC.SubjectCount; ++i) {
	OutputGSN = MyClient.GetSubjectName(i);
	std::cout <<	OutputGSN.SubjectName << std::endl;
	};
	*/

	std::cout << "Enter the index of the object you want to track:";
	int n;
	std::cin >> n;
	OutputGSN = MyClient.GetSubjectName(n);
	#define SubjectName OutputGSN.SubjectName
	std::cout << "Tracked object:" << SubjectName << std::endl;
	/*
	Output_GetSubjectName Drone1;
	Drone1 = MyClient.GetSubjectName(0);
	Drone1.SubjectName = "Drone 1";
	*/
	Output_GetSubjectRootSegmentName OutputGSRS;
	OutputGSRS = MyClient.GetSubjectRootSegmentName(SubjectName);
	#define SegmentName OutputGSRS.SegmentName
	running=1;
	while(running){
		Sleep(1000);

		//Use Vicon SDK to get position data
		std::cout << "Waiting for new frame...";
		while (MyClient.GetFrame().Result != Result::Success)
		{
			// Sleep a little so that we don't lumber the CPU with a busy poll
			Sleep(200);
			std::cout << ".";
		}
		Output_GetFrameNumber Frames_Since_Boot;
		Frames_Since_Boot = MyClient.GetFrameNumber();
		Output_GetSegmentStaticRotationQuaternion static_quat = MyClient.GetSegmentStaticRotationQuaternion(SubjectName, SegmentName);

		float q[4];
		q[0] = static_quat.Rotation[0];
		q[1] = static_quat.Rotation[1];
		q[2] = static_quat.Rotation[2];
		q[3] = static_quat.Rotation[3];
		Output_GetSegmentStaticTranslation static_translation =	MyClient.GetSegmentStaticTranslation(SubjectName, SegmentName);

		if(rc_mav_send_heartbeat_abbreviated()){
			fprintf(stderr,"failed to send heartbeat\n");
		}
		else{
			printf("sent heartbeat\n");
		}
		if (rc_mav_send_att_pos_mocap(q, static_translation.Translation[0], static_translation.Translation[1], static_translation.Translation[2])) {
			fprintf(stderr, "failed to send position data\n");
		}
		else {
			std ::cout << "Static Rotation Quaternion: (" << q[0] << ", "
				<< q[1] << ", "
				<< q[2] << ", "
				<< q[3] << ")" << std::endl
				<< "Static Translation: (" << static_translation.Translation[0] << ","
				<< static_translation.Translation[1] << ","
				<< static_translation.Translation[2] << ")" << std::endl
				<< "Frames since boot: " << Frames_Since_Boot.FrameNumber << std::endl;

		}
	}

	// stop listening thread and close UDP port
	printf("closing UDP port\n");
	rc_mav_cleanup();

	//sdfgsdfg
}