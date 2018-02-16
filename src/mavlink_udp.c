/**
 * @file mavlink_udp.c
 *
 * @brief      C interface for communicating with mavlink over UDP in
 *             Linux/Windows
 *
 *             Uses common mavlink v2 packets generated from official mavlink
 *             source (https://github.com/mavlink/mavlink). Also see
 *             mavlink_udp_helpers.h for additional helper functions for most
 *             commonly used packets.
 *
 * @author     James Strawson & Henry Gaudet
 *
 * @date       1/24/2018
 */

#define _GNU_SOURCE	// for pthread_timedjoin_np
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>	// for specific integer types
//#include <sys/time.h>
//#include <arpa/inet.h>	// Sockets & networking
#include <sys/types.h>
//#include <sys/socket.h>
#include <WinSock2.h>
#include <string.h>
#include "../include/rc/mavlink_udp.h"


#define BUFFER_LENGTH		512 // common networking buffer size
#define MAX_UNIQUE_MSG_TYPES	256
#define LOCALHOST_IP "127.0.0.1"


// connection stuff
static int init_flag=0;
static int sock_fd;
static int current_port;
static struct sockaddr_in my_address ;
static struct sockaddr_in dest_address;
static uint8_t system_id;




// private local function declarations;
static uint64_t __nanos_since_boot();
static int __address_init(struct sockaddr_in* address, const char* dest_ip, uint16_t port);


////////////////////////////////////////////////////////////////////////////////
// LOCAL FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////


static uint64_t __micros_since_boot()
{
	FILETIME ft;
	GetSystemTimeAsFileTime(&ft);
	unsigned long long tt = ft.dwHighDateTime;
	tt <<= 32;
	tt |= ft.dwLowDateTime;
	tt /= 10;
	tt -= 11644473600000000ULL;
	return tt;
}


// configures sockaddr_in struct for UDP port
int __address_init(struct sockaddr_in* address, const char* dest_ip, uint16_t port)
{
	// sanity check
	if(address == NULL || port < 1){
		fprintf(stderr, "ERROR: in __address_init: received NULL address struct\n");
		return -1;
	}
	memset((char*) address, 0, sizeof address);
	address->sin_family = AF_INET;
	// convert port from host to network byte order
	address->sin_port = htons(port);
	address->sin_addr.s_addr = ((long)dest_ip==0) ? htonl(INADDR_ANY) : inet_addr(dest_ip);
	return 0;
}




////////////////////////////////////////////////////////////////////////////////
// DEFINITIONS FOR mavlink_udp.h
////////////////////////////////////////////////////////////////////////////////

int rc_mav_init(uint8_t sysid, const char* dest_ip, uint16_t port)
{

	// sanity checks
	if(init_flag!=0){
		fprintf(stderr, "WARNING, trying to init mavlink connection when it's already initialized!\n");
		return -1;
	}

	if(dest_ip==NULL){
		fprintf(stderr, "ERROR: in rc_mav_init received NULL dest_ip string\n");
		return -1;
	}

	// save port globally for other functions to use
	current_port = port;

	

	// open socket for UDP packets

	printf("\nInitialising Winsock...");
	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Failed. Error Code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	printf("Initialised.\n");

	sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(sock_fd == INVALID_SOCKET){
		fprintf(stderr,"ERROR: in rc_mav_init: socket failed");
		return -1;
	}

	// fill out rest of sockaddr_in struct
	if (__address_init(&my_address, 0, current_port) != 0){
		fprintf(stderr, "ERROR: in rc_mav_init: couldn't set local address\n");
		return -1;
	}

	// bind address to listening port
	if(bind(sock_fd, (struct sockaddr *) &my_address, sizeof my_address) < 0){
		fprintf(stderr,"ERROR: in rc_mav_init, bind failed\n");
		return -1;
	}

	// set destination address
	if(__address_init(&dest_address, dest_ip, current_port) != 0){
		fprintf(stderr, "ERROR: in rc_mav_init: couldn't set destination address");
		return -1;
	}

	// signal initialization finished
	init_flag=1;
	system_id=sysid;


	return 0;
}

int rc_mav_set_dest_ip(const char* dest_ip)
{
	return __address_init(&dest_address,dest_ip,current_port);
}


int rc_mav_set_system_id(uint8_t sys_id)
{
	system_id = system_id;
	return 0;
}


int rc_mav_cleanup()
{
	//close(sock_fd);
	init_flag=0;
	return 0;
}


int rc_mav_send_msg(mavlink_message_t msg)
{
	if(init_flag == 0){
		fprintf(stderr, "ERROR: in rc_mav_send_msg, socket not initialized\n");
		return -1;
	}
	uint8_t buf[BUFFER_LENGTH];
	memset(buf, 0, BUFFER_LENGTH);
	int msg_len = mavlink_msg_to_send_buffer(buf, &msg);
	if(msg_len < 0){
		fprintf(stderr, "ERROR: in rc_mav_send_msg, unable to pack message for sending\n");
		return -1;
	}
	int bytes_sent = sendto(sock_fd, buf, msg_len, 0, (struct sockaddr *) &dest_address,
							sizeof dest_address);
	if(bytes_sent != msg_len){
		perror("ERROR: in rc_mav_send_msg: failed to write to UDP socket\n");
		return -1;
	}
	return 0;
}


////////////////////////////////////////////////////////////////////////////////
// DEFINITIONS FOR mavlink_udp_helpers.h
////////////////////////////////////////////////////////////////////////////////

int rc_mav_send_heartbeat_abbreviated()
{
	// sanity check
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(system_id, MAV_COMP_ID_ALL, &msg, 0, 0, 0, 0, 0);
	if(rc_mav_send_msg(msg)){
		fprintf(stderr, "ERROR: in rc_mav_send_heartbeat_abbreviated, failed to send\n");
		return -1;
	}
	return 0;
}


int rc_mav_send_heartbeat(uint32_t custom_mode, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t system_status)
{
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(system_id, MAV_COMP_ID_ALL, &msg, type, autopilot, base_mode, custom_mode, system_status);
	return rc_mav_send_msg(msg);
}




int rc_mav_send_att_pos_mocap(float q[4], float x, float y, float z)
{
	mavlink_message_t msg;
	uint64_t time_usec = __micros_since_boot();
	mavlink_msg_att_pos_mocap_pack(system_id, MAV_COMP_ID_ALL, &msg, time_usec, q, x, y, z);
	return rc_mav_send_msg(msg);
}

