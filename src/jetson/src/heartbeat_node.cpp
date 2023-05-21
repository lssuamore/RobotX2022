#include "ros/ros.h"
#include "std_msgs/String.h"
#include "jetson/task_info.h"	
#include "sensor_msgs/NavSatFix.h"
//#include "heartbeat_sock.h"
/*  Author: Matthew Bowen
    Class:  CSCI 201
    Created:9-2-21
    Updated:9-2-21
*/

//#include "heartbeat_sock.h"
/* 
 * Class Name: heartbeat_sock
 * Purpose: open socket at specified ip and port, 
 *          build and send a formatted string through that socket,
 *          openned with a '$', closed with a "*<CR><LF><XORChecksum>"
 * Matthew Bowen <mbowen2@lssu.edu>
 * 10/17/22
 */

#include "jetson/state_msg.h"					// message type used to recieve state of operation from mission_control

#include "jetson/usv_pose_msg.h"				// message that holds usv position as a geometry_msgs/Point and heading in radians as a Float64
#include "jetson/zed2i_msg.h"				// message that comes from the zed2i, it gets string of object, order of objects and position of object from the camera
#include "sensor_msgs/NavSatFix.h"	
#include "jetson/task_info.h"	

#include <arpa/inet.h> // inet_addr()
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h> // bzero()
#include <sys/socket.h>
#include <unistd.h> // read(), write(), close()
#define MAX 256
#define SA struct sockaddr
#define PORT 8080
#define IP "127.0.0.1"
#define MSG "RXHRB,111221,161229,21.31198,N,157.88972,W,ROBOT,2,2"


/*Array type stack*/
class heartbeat_sock
{
private:
    char ip[MAX];
    char port[MAX];
    int sockfd, connfd;
public:
    //open socket  ip (v4 format), port
	heartbeat_sock(const char [], const int);
    //closes socket
	~heartbeat_sock();
    //send NMEA string packet; sockfd, message to send
    int send(const char *);
};


/* 
 * Method Name: heartbeat_sock constructor
 * Purpose: open socket at specified ip and port
 * Matthew Bowen <mbowen2@lssu.edu>
 * 10/17/22
 */
heartbeat_sock::heartbeat_sock(const char ip[MAX], const int port)
{
    struct sockaddr_in servaddr, cli;

	// socket create and verification
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) 
	{
		printf("socket creation failed...\n");
		return;
	}
	else
		printf("Socket successfully created..\n");
	bzero(&servaddr, sizeof(servaddr));

	// assign IP, PORT
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(ip);
	servaddr.sin_port = htons(port);

	// connect the client socket to server socket
	if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) 
	{
		printf("connection with the server failed...\n");
		return;
	}
	else
		printf("connected to the server..\n");

	// function for chat
	//func(sockfd);

    // function sends single heartbeat message
	//beatSend(sockfd,MSG);
}

/* 
 * Method Name: heartbeat_sock destructor
 * Purpose: Close socket
 * Matthew Bowen <mbowen2@lssu.edu>
 * 10/17/22
 */
heartbeat_sock::~heartbeat_sock()
{
    // close the socket
	close(sockfd);
	close(connfd);
}

/* 
 * Method Name: send
 * Purpose: send a formatted string to the RobotX Judges Server to convey system status
 * Function: recieve buffer with all information to send, comma separated,
 *              calculate XOR checksum, prepend '$', append terminating '*' <XOR checksum> "<CR><LF>",
 *              and send to the specified socket
 * Matthew Bowen <mbowen2@lssu.edu>
 * 10/17/22
 */
int heartbeat_sock::send(const char *buff)
{
    char packet[MAX];
    int buffLen = strnlen(buff,MAX);
    unsigned char checksum = 0;
    char formatstring[MAX];

    //calculate checksum, skipping '$'
    for(int i=0; i<buffLen; i++)
        checksum ^= buff[i];
    //prepend '$'
    strcpy(packet, "$");
    //put buffer info into the packet
    strncat(packet, buff, MAX);
    //append '*' terminator, checksum, and <CR><LF> terminator
    sprintf(formatstring, "*%02X\r\n", checksum);
    strncat(packet,formatstring, MAX);

	/*TESTING*/
	//print entire packet
	//packet[sizeof(packet)] = '\0';
	//printf("PACKET: %s\n", packet);

    //write the packet to the socket
    write(sockfd, packet, strnlen(packet,MAX));
    return 0;
}
//Global Variables
char teamid[] = "LSSUS";
int status = 1;
int UAVstat = 1;
int task;
float lat,lon,alt;

// THIS FUNCTION: Updates the state of "propulsion_system" given by "mission_control"
// ACCEPTS: propulsion_system state_msg from "Task_Num"
// RETURNS: (VOID)
// =============================================================================
void Task_Num_update(const jetson::task_info::ConstPtr& msg)
{
	
		task = msg->data.data;				//Gets task number
		if (task != 0)
		{
				ROS_INFO("%d",task);
		}
	
	
} // END OF Task_Num_update()

void gps_processor(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	lat = gps_msg->latitude; //sets latitude from gps
	lon = gps_msg->longitude; //sets longitude from gps
	alt = gps_msg->altitude; //sets altitude rom gps
	
	ROS_INFO("%f, %f, %f",lat, lon, alt);
} // END OF gps_processor()
  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "heartbeat");

  ros::NodeHandle nh1, nh2,nh3;
  
  char msg[MAX];
  char timeDateStr[14];
  char latLon[24];
  char temp[64];
  time_t rawtime;
  struct tm * timeinfo;

 heartbeat_sock sock(IP,PORT);

	ros::Subscriber Task_Num_sub = nh1.subscribe("task_state", 1, Task_Num_update);		//Subscribes to task_info
	ros::Subscriber gpspos_sub = nh2.subscribe("/gps/fix", 10, gps_processor);		// subscribes to GPS position from indoor gps

  //send heartbeat at 1 Hz
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
   
    //compile the message to send
    //heartbeat message
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    
    switch(task)
    {
      case 1:
        //Task Identifier
        strcat(msg, "RXHRB,");
        //Local Date and Time
        strftime(timeDateStr,8,"%d%m%y,%H%M%S,",timeinfo);
        strcat(msg,timeDateStr);
        //Latitude and Longitude
        sprintf(latLon, "%.5f,%c,%.5f,%c,", lat, lat>0 ? 'N' : 'S', lon, lon>0 ? 'E' : 'W');
        strcat(msg, latLon);
        //Team Identifier
        strcat(msg,teamid);
        //System Status
        sprintf(temp, "%d,", status);
        strcat(msg,temp);
        //UAV Status
        sprintf(temp, "%d", UAVstat);
    };
	ROS_INFO("Heartbeat : Working");

    //send the heartbeat
    sock.send(msg);
	 //update all subscriptions
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
