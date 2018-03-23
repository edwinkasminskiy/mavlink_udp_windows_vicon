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
#include <string>
#include <sstream>
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
	running = 0;
	return;
}

int main(int argc, char * argv[])
{
	//Set up client to read data from Vicon
	using namespace ViconDataStreamSDK::CPP;
	Output_GetSegmentGlobalRotationQuaternion global_quat;
	float q[4];
	Client MyClient;
	bool ok;
	int ret;
	const char* dest_ip;
	Output_GetSegmentGlobalTranslation global_translation;
	Output_GetFrameNumber Frames_Since_Boot;
	std::string ip_string;
	std::string drone_name;
	// set default options before checking options
	my_sys_id = DEFAULT_SYS_ID;
	port = RC_MAV_DEFAULT_UDP_PORT;
	dest_ip = "127.0.0.1";

	// initialize the UDP port and listening thread with the rc_mav lib
	if (rc_mav_init(my_sys_id, dest_ip, port) < 0)
	{
		return -1;

	}
	printf("run with -h option to see usage and other options\n");
	// inform the user what settings are being used
	printf("\n");
	printf("Initializing with the following settings:\n");
	//printf("dest ip addr: %s\n", dest_ip);
	printf("my system id: %d\n", my_sys_id);
	printf("UDP port: %d\n", port);
	printf("\n");

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, signal_handler);


	ok = (MyClient.Connect("localhost:801").Result == Result::Success);

	while(!ok)
	{
		std::cout << "Warning - connect failed..." << std::endl;
	}

	MyClient.SetAxisMapping(Direction::Forward,
		Direction::Right,
		Direction::Down); // NED
	MyClient.EnableSegmentData();
	MyClient.EnableMarkerData();
	MyClient.EnableUnlabeledMarkerData();
	MyClient.EnableMarkerRayData();
	MyClient.EnableDeviceData();
	MyClient.EnableDebugData();
	Output_GetSubjectCount OutputGSC;
	OutputGSC = MyClient.GetSubjectCount();
	Output_GetSubjectName OutputGSN;
	unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;


	while (SubjectCount < 1)
	{
		if (MyClient.GetFrame().Result != Result::Success)
		{
			output_stream << "Waiting for new frame..." << std::endl;
			//continue;
		}
		SubjectCount = MyClient.GetSubjectCount().SubjectCount;
	}
	output_stream << "Subjects (" << SubjectCount << "):" << std::endl;



	for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
	{
		output_stream << "  Subject #" << SubjectIndex + 1 << std::endl;

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

	running = 1;

	output_stream << "Starting data stream" << std::endl;
	while (running)
	{
		//Use Vicon SDK to get position data
		//std::cout << "Waiting for new frame...";
		if (MyClient.GetFrame().Result != Result::Success)
		{
			std::cout << "No new frame received" << std::endl;
			continue;
		}

		
		Frames_Since_Boot = MyClient.GetFrameNumber();

		//For every subject, get the name and parse the name and IP address
		for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
		{
			//output_stream << "  Subject #" << SubjectIndex + 1 << std::endl;

			// Get the subject name
			std::string SubjectName = MyClient.GetSubjectName(SubjectIndex).SubjectName;
			
			//output_stream << "    Object: " << SubjectName << std::endl;
			std::istringstream iss(SubjectName);
			while (iss.good())
			{
				getline(iss, ip_string, '@');
			}
			dest_ip = ip_string.c_str();
			drone_name = SubjectName;

			//output_stream << "    Object Name: " << drone_name << std::endl;
			//output_stream << "    IP address: " << dest_ip << std::endl;
			// Get the root segment
			std::string RootSegment = MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
			//output_stream << "    Root Segment: " << RootSegment << std::endl;


			if (rc_mav_set_dest_ip(dest_ip)) {
				std::cout << "ERROR setting dest ip\n";
				continue;
			}

			global_quat = MyClient.GetSegmentGlobalRotationQuaternion(drone_name, RootSegment);
			q[0] = global_quat.Rotation[0];
			q[1] = global_quat.Rotation[1];
			q[2] = global_quat.Rotation[2];
			q[3] = global_quat.Rotation[3];

			global_translation = MyClient.GetSegmentGlobalTranslation(drone_name, RootSegment);

			ret = rc_mav_send_att_pos_mocap(q, global_translation.Translation[0], global_translation.Translation[1], global_translation.Translation[2]);
			if(ret == -1){
				fprintf(stderr, "failed to send position data\n");
				continue;
			}
			else{
				printf("\rquat: %4.2f  %4.2f  %4.2f translation (XYZ): %6.4fm %6.4fm %6.4fm", q[0], q[1], q[2], q[3], global_translation.Translation[0], global_translation.Translation[1], global_translation.Translation[2]);
				fflush(stdout);
			}
		
		}// end for loop through subjects

	} // end while(running)


// stop listening thread and close UDP port
printf("closing UDP port\n");
rc_mav_cleanup();

return 0;
}
