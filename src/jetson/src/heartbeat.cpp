//Heartbeat Code Includes and Constants
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/Odometry.h"
//#include "jetson/state_msg.h"					// message type used to communicate state for rudimentary codes
#include "jetson/AMS_state.h"				// message that says the AMS state
#include "std_msgs/UInt64.h"
#include "jetson/task_info.h"				// message that comes from mission control
#include "jetson/Acoustics_msg.h"		//Meassage published by the computer from Acoustics
#include "jetson/NED_objects.h"  // message type that has an array of pointstamped
#include "jetson/zed2i_msg.h"				// message that comes from the zed2i, it gets string of object, order of objects and position of object from the camera
#include "jetson/Detect_Dock_Fling.h"		//Message for task 6 and task 7
#include "sensor_msgs/NavSatFix.h"		// message type of lat and long coordinates given by the GPS

#include <arpa/inet.h> // inet_addr()
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h> // bzero()
#include <sys/socket.h>
#include <unistd.h> // read(), write(), close()
#include <ctime>
#define MAX 1024
#define SA struct sockaddr
#define PORT 12345
#define IP "127.0.1.1"
//#define MSG "RXHRB,111221,161229,21.31198,N,157.88972,W,ROBOT,2,2"

//Global Variables
char ip[MAX];
int port;
char teamid[6] = "LSQUT";
int status = 1;
char UAVStatusStr[2] = "3";
int task = 1;
float lat = 0;
float lon = 0;
float alt = 0;
//Task 2
char enterGateStr[2]; //1,2,3
char exitGateStr[2]; //1,2,3
//Task 3
char channelClearStr[2] = "1"; //in progress = 1, complete = 2
//Task 4
char wildlifeCountStr[2] = "3"; //1,2,3
char wildlifeStr1[2]; //platypus = P, crocodile = C, turtle = T
char wildlifeStr2[2];
char wildlifeStr3[2];
//Task 5
char lightPatternStr[64]; //Color Pattern
//Task 6
char dockColorStr[2]; //R,G,B
char dockingStatusStr[2]; //Docking = 1, Complete = 2
//Task 7
char targetColorStr[2];  //R,G,B
char flingerStatusStr[2];  //Scanning = 1, Flinging = 2
//Task 8
char itemStatusStr[2];
//Task 9
char object1LocationStr[] = "R,21.31198,N,157.88972,W,";
char object2LocationStr[] = "N,21.32198,N,157.89972,W";

int Task = 0;
//Task1
std::string AMS_state_str;
int AMS_state = 0; 
std::string UAV_state_str;
int UAV_state = 0; 
char Task1_str[2];

// Task 2
char Task2_str[0];
int Entry_gate= 0; 
char Exit_gate_str[0];
int Exit_gate = 0; 

//Task 3
std::string State;  // string array to hold the state of the task
char Task3_str[1];

//Task 4
std::string Animal[3];  // string array to hold the names of the animals
char Animal_str[4]; // String to send animals for heartbeat
int animal_qty = 0;

//Task 5 
std::string Object[9];  // string array to hold the names of the objects from the zed2i
float x_object_NED[9], y_object_NED[9];  // arrays to hold animal locations
char Cod_Seq[100];
int cnt = 0;
int black = 0;

//Task 6
std::string Color_Dock;  
std::string Status_Dock; 
char Task6_str[2];

//Task 7
std::string Color_Fling;  
std::string Status_Fling; 
char Task7_str[2];

//Task 8
std::string Item_state_str;
std::string UAV_state2_str;
char UAV_Task8[2];


// THIS FUNCTION: Subscribes to the task topic
// ACCEPTS: jetson/task_info.msg
// RETURNS: (VOID)
// =============================================================================
void Task_Control (const jetson::task_info::ConstPtr& msg)									//Task name subscriber
{
	task = msg->name.data;
}//END OF //Task_Control()

//======================TASK 1==================================================
// THIS FUNCTION: Gets the AMS Status and writes char variable 
// ACCEPTS: jetson/AMS_state.msg
// RETURNS: (VOID)
// =============================================================================
void AMS_status (const std_msgs::Int32::ConstPtr& msg_Status)
{
	status = msg_Status->data;
	//UAV_state_str = msg->UAV;
	
	/* if (AMS_state_str =="Remote Operated")
	{
		Task1_str[0] = 1 + '0';
		status = 1;
	}
	else if (AMS_state_str  == "Autonomous")
	{
		Task1_str[0] = 2 + '0';
		status = 2;
	}
	else if (AMS_state_str =="Killed")
	{
		Task1_str[0] = 3 + '0';
		status = 3;
	} */
	//if (UAV_state_str == "Stowed")
		//UAVStatusStr[0] = 1 + '0';
//	else if (UAV_state_str == "Deployed")
	//	UAVStatusStr[0] = 2+ '0';
	//else if (UAV_state_str == "Faulted")
		//UAVStatusStr[0] = 3  + '0';
	
} // END OF AMS_status()

//======================TASK 2==================================================
// THIS FUNCTION: Gets the Acoustics gate and writes char variable 
// ACCEPTS: jetson/Acoustics_msg.msg
// RETURNS: (VOID)
// =============================================================================
void Acoustics_get_gate (const jetson::Acoustics_msg::ConstPtr& msg)
{
	Entry_gate = msg->Entrance.data;
	enterGateStr[0] = Entry_gate + '0';
	Exit_gate = msg->Exit.data;
	exitGateStr[0] = Exit_gate + '0';
} // END OFAcoustics_get_gate()
//======================TASK 3==================================================
// THIS FUNCTION: Gets Task 3 if it's completed or in progress 
// ACCEPTS: jetson/task_info.msg
// RETURNS: (VOID)
// =============================================================================
void task_state (const jetson::task_info::ConstPtr& msg)
{
	State = msg->state;
	
	if (State=="In Progress")
		channelClearStr[0] = 1 + '0';
	else if (State =="Completed")
		channelClearStr[0] = 2 + '0';
} // END OF task_state()

//======================TASK 4==================================================
// THIS FUNCTION: Gets what animals detected and the order, also get the locations in respect the USV
// ACCEPTS: jetson/NED_objects.msg
// RETURNS: (VOID)
// =============================================================================
void CC_animals_ned_update(const jetson::NED_objects::ConstPtr& object)
{
	
		int i = 0;
		animal_qty = object->quantity;
		
		wildlifeCountStr[0] = animal_qty + '0';
		for (int i = 0; i <object->quantity; i++)
		{
		   Animal[i] = object->objects[i].header.frame_id;  // Getting array of animal names
		   if (Animal[0] == "crocodile")
		   {
		   wildlifeStr1[0]= 'C';
		   }
		   else if (Animal[0] == "turtle")
		   {
			   wildlifeStr2[0] = 'T';
		   }
		   else if (Animal[0] == "platypus")
		   {
			  wildlifeStr3[0] = 'P';
		   }
		  
		}
		
}  // END OF CC_animals_ned_update()

//======================TASK 5==================================================
// THIS FUNCTION: Gets the color of the light bouy
// ACCEPTS: jetson/zed2i_msg.msg
// RETURNS: (VOID)
// =============================================================================
void object_rec(const jetson::zed2i_msg::ConstPtr& msg)
{
	int i = 0;
		   Object[i] = msg->objects[i].header.frame_id;  // Getting array of animal name. Based on on array location, the object is closer or farther from USV
		   x_object_NED[i] = msg->objects[i].point.x;  // Getting x position of animals 
		   y_object_NED[i] = msg->objects[i].point.y;  // Getting y position of animals 
	//std::cout<<Object[0];
	
	if (Object[0] == "Blue Light")
	{
		lightPatternStr[cnt] = 'B';
		black = 0;
		cnt++;
	}
	else if (Object[0] == "Red Light")
	{
		lightPatternStr[cnt] = 'R';
		black = 0;
		cnt++;
	}
	else if (Object[0] == "Green Light")
	{
		lightPatternStr[cnt] = 'G';
		black = 0;
		cnt++;
	}
	else if (Object[0] == "Black Light")
	{
		black++;
	}
	
	
		
} // END OF object_rec()
//======================TASK 6==================================================
// THIS FUNCTION: Gets what dock we are docking and the status
// ACCEPTS: jetson/Detect_Dock_Fling.msg
// RETURNS: (VOID)
// =============================================================================
void Dock_detect(const jetson::Detect_Dock_Fling::ConstPtr& msg)
{
	
	Color_Dock= msg->Color;
	Status_Dock= msg->Status;

	if (Color_Dock == "red")
	{
		dockColorStr[0] = 'R';
	}
	else if (Color_Dock== "blue")
		dockColorStr[0] = 'B';
	else if (Color_Dock == "green")
		dockColorStr[0] = 'G';
	if (Status_Dock == "Docking")
		dockingStatusStr[0] = '1';
	else if (Status_Dock== "Complete")
		dockingStatusStr[0] = '2';
	
} // END OF Dock()

//======================TASK 7==================================================
// THIS FUNCTION: Gets what dock we are flinging and the status
// ACCEPTS: jetson/Detect_Dock_Fling.msg
// RETURNS: (VOID)
// =============================================================================
void Dock_Fling(const jetson::Detect_Dock_Fling::ConstPtr& msg)
{
	
	Color_Fling= msg->Color;
	Status_Fling= msg->Status;

	if (Color_Fling == "red")
		targetColorStr[0] = 'R';
	else if (Color_Fling== "blue")
		targetColorStr[0] = 'B';
	else if (Color_Fling== "green")
		targetColorStr[0] = 'G';
	if (Status_Fling == "Scanning")
		flingerStatusStr[0] = '1';
	else if (Status_Fling== "Flinging")
		flingerStatusStr[0] = '2';
	
} // END OF Dock_Fling()
//======================TASK 8==================================================
// THIS FUNCTION: Gets the UAV status and if it picked up the can or not
// ACCEPTS: jetson/AMS_state.msg
// RETURNS: (VOID)
// =============================================================================
void AMS2_status (const jetson::AMS_state::ConstPtr& msg)
{
	Item_state_str = msg->Item;
	UAV_state2_str = msg->UAV;

	if (UAV_state2_str == "Stowed")
		UAVStatusStr[0] = 1 + '0';
	else if (UAV_state2_str == "Deployed")
		UAVStatusStr[0] = 2 + '0';
	else if (UAV_state2_str == "Faulted")
		UAVStatusStr[0] = 3 + '0';
	
		if (Item_state_str == "Not Picked Up")
		itemStatusStr[0] = 1 + '0';
	else if (Item_state_str == "Picked Up")
		itemStatusStr[0] = 2 + '0';
	else if (Item_state_str == "Delivered")
		itemStatusStr[0] = 3 + '0';
} // END OF AMS_status()


/*open a tcp socket and send an NMEA string*/
class heartbeat_sock
{
private:
    int sockfd;
    struct sockaddr_in servaddr, cli;
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
    int buffLen = (int)strnlen(buff,MAX);
    unsigned char checksum = 0;
    char formatstring[MAX];

    //calculate checksum, skipping '$'
    for(int i=0; i<buffLen; i++)
    {
        printf("%c",buff[i]);
        checksum ^= buff[i];
    }
    printf("\n\n");
    //prepend '$'
    strcpy(packet, "$");
    //put buffer info into the packet
    strncat(packet, buff, MAX);
    //append '*' terminator, checksum, and <CR><LF> terminator
    sprintf(formatstring, "*%02X\r\n", checksum);
    strncat(packet,formatstring, MAX);

    printf("%s",packet);

    //TESTING
    //print entire packet
    //packet[sizeof(packet)] = '\0';
    //printf("PACKET: %s\n", packet);

    // socket create and verification
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1)
    {
      printf("socket creation failed...\n");
      return 1;
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
      return 1;
    }
    else
      printf("connected to the server..\n");


    //write the packet to the socket
    write(sockfd, packet, strnlen(packet,MAX));

    close(sockfd);
    return 0;
}



// THIS FUNCTION: Updates the state of "propulsion_system" given by "mission_control"
// ACCEPTS: propulsion_system state_msg from "Task_Num"
// RETURNS: (VOID)
// =============================================================================
/*void Task_Num_update(const jetson::task_info::ConstPtr& msg)
{
	
		task = msg->data.data;				//Gets task number
		if (task != 0)
		{
				ROS_INFO("%d",task);
		}
	
	
} // END OF Task_Num_update()
*/
void gps_processor(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	lat = gps_msg->latitude; //sets latitude from gps
	lon = gps_msg->longitude; //sets longitude from gps
	alt = gps_msg->altitude; //sets altitude rom gps
	
	ROS_INFO("%f, %f, %f",lat, lon, alt);
} // END OF gps_processor()





//send heartbeat after each ros::spinonce
void heartbeatSend(heartbeat_sock sock)
{
  char packet[MAX];
  char timeDateStr[14];
  char latLonStr[32];
  char temp[64];
  time_t rawtime;
  struct tm * timeinfo;


	//compile the message to send
    //heartbeat message
    time(&rawtime);
    timeinfo = localtime(&rawtime);

    switch(task)
    {
      case 1: // Task 1 - heartbeat
        //Message ID (Task Identifier)
        strcpy(packet, "RXHRB,");
        //Local Date and Time
        strftime(timeDateStr,15,"%d%m%y,%H%M%S,",timeinfo);
        strcat(packet,timeDateStr);
        //Latitude and Longitude
        sprintf(latLonStr, "%.5f,%c,%.5f,%c,", lat, lat>0 ? 'N' : 'S', lon, lon>0 ? 'E' : 'W');
        strcat(packet, latLonStr);
        //Team Identifier
        strcat(packet,teamid);
        strcat(packet,",");
        //System Status
        sprintf(temp, "%i,", status);
        strcat(packet,temp);
        //UAV Status
        //sprintf(temp, "%i", UAVstat);
	      strcat(packet,UAVStatusStr);
        break;
    case 2: //Task 2 - Entrance and Exit Gates
        //Message ID (Task Identifier)
        strcpy(packet, "RXGAT,");

        //Local Date and Time
        strftime(timeDateStr,15,"%d%m%y,%H%M%S,",timeinfo);
        strcat(packet,timeDateStr);
        //Team Identifier
        strcat(packet,teamid);
        strcat(packet, ",");

        //Entrance Gate
        strcat(packet, enterGateStr);
        strcat(packet, ",");
        //Exit Gate
        strcat(packet, exitGateStr);
        break;
    case 3: // Task 3 - Follow the Path
        //Message ID (Task Identifier)
        strcpy(packet, "RXPTH,");

        //Local Date and Time
        strftime(timeDateStr,15,"%d%m%y,%H%M%S,",timeinfo);
        strcat(packet,timeDateStr);
        //Team Identifier
        strcat(packet,teamid);
        strcat(packet, ",");

        //Completion Status
        strcat(packet, channelClearStr);
        break;
    case 4: // Task 4 - Wildlife Encounter
        //Message ID (Task Identifier)
        strcpy(packet, "RXENC,");

        //Local Date and Time
        strftime(timeDateStr,15,"%d%m%y,%H%M%S,",timeinfo);
        strcat(packet,timeDateStr);
        //Team Identifier
        strcat(packet,teamid);
        strcat(packet, ",");

        //Number of 'wildlife' detected
        strcat(packet,wildlifeCountStr);
        strcat(packet,",");
        //Which 'wildlife' are detected
        strcat(packet,wildlifeStr1);
        strcat(packet,",");
        strcat(packet,wildlifeStr2);
        strcat(packet,",");
        strcat(packet,wildlifeStr3);
        break;
    case 5: // Task 5 - Scan the Code
        //Message ID (Task Identifier)
        strcpy(packet, "RXCOD,");

        //Local Date and Time
        strftime(timeDateStr,15,"%d%m%y,%H%M%S,",timeinfo);
        strcat(packet,timeDateStr);
        //Team Identifier
        strcat(packet,teamid);
        strcat(packet, ",");

        //Light Pattern
        strcat(packet, lightPatternStr);
        break;
    case 6: // Task 6 - Detect and Dock
        //Message ID (Task Identifier)
        strcpy(packet, "RXDOK,");

        //Local Date and Time
        strftime(timeDateStr,15,"%d%m%y,%H%M%S,",timeinfo);
        strcat(packet,timeDateStr);
        //Team Identifier
        strcat(packet,teamid);
      	strcat(packet, ",");

        //Dock Color
        strcat(packet, dockColorStr);
        strcat(packet,",");

        //AMS Status
        strcat(packet,dockingStatusStr);
        break;
    case 7: // Task 7 - Find and Fling
        //Message ID (Task Identifier)
        strcpy(packet, "RSFLG,");

        //Local Date and Time
        strftime(timeDateStr,15,"%d%m%y,%H%M%S,",timeinfo);
        strcat(packet,timeDateStr);
        //Team Identifier
        strcat(packet,teamid);
      	strcat(packet, ",");

        //Target Color
        strcat(packet,targetColorStr);
        strcat(packet,",");
        //Docking Status
        strcat(packet,flingerStatusStr);
        break;
    case 8: // Task 8 - UAV Replenishment
        //Message ID (Task Identifier)
        strcpy(packet, "RXUAV,");

        //Local Date and Time
        strftime(timeDateStr,15,"%d%m%y,%H%M%S,",timeinfo);
        strcat(packet,timeDateStr);
        //Team Identifier
        strcat(packet,teamid);
      	strcat(packet, ",");

        //UAV Status
        strcat(packet,UAVStatusStr);
        strcat(packet,",");
        //Item Status
        strcat(packet,itemStatusStr);
        break;
    case 9: // Task 9 - UAV Search and Report
        //Message ID (Task Identifier)
        strcpy(packet, "RXSAR,");

        //Local Date and Time
        strftime(timeDateStr,15,"%d%m%y,%H%M%S,",timeinfo);
        strcat(packet,timeDateStr);
        
        //Object locations
        strcat(packet,object1LocationStr);
        strcat(packet,object2LocationStr);
        //Team Identifier
        strcat(packet,teamid);
      	strcat(packet, ",");
        //UAV Status
        strcat(packet,UAVStatusStr);
    };
    //ros::ROS_INFO("Heartbeat : Working");

    //send the heartbeat
    sock.send(packet);
}

int main(int argc, char **argv)
{
  // names the program for visual purposes
	ros::init(argc, argv, "heartbeat");

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	// NodeHandles
	ros::NodeHandle n, nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8, nh9;

  // Subscribers 
 	
	ros::Subscriber gpspos_sub = nh2.subscribe("/gps/fix", 10, gps_processor);		// subscribes to GPS position from indoor gps
	ros::Subscriber AMS_status_sub = n.subscribe("AMS_status", 1, AMS_status);					//Obtains what is the current AMS state 
	ros::Subscriber Task_info_sub = n.subscribe("task_info", 1,Task_Control);							//Obtains current AMS task
	ros::Subscriber Acoustics_get_sub = n.subscribe("Acoustics_gate", 1, Acoustics_get_gate);						//Obtains what is the current AMS state 
	ros::Subscriber Task3_info = n.subscribe("task_info", 1, task_state);						//Obtains if task 3 is in progress or completed
	ros::Subscriber CC_animals_ned_sub = n.subscribe("CC_animals_ned", 1,CC_animals_ned_update );						//Obtains what is the current AMS state 
	ros::Subscriber obj_rec_sub = n.subscribe("obj_rec", 1, object_rec);						//Obtains what object the camera recognise, pose of the object and order of recognition
	ros::Subscriber Detect_Dock_Fling_sub = n.subscribe("Det_Dock", 1, Dock_detect);						//Obtains what object the camera recognise, pose of the object and order of recognition
	ros::Subscriber Detect_Fling_sub = n.subscribe("Det_Dock", 1, Dock_Fling);						//Obtains what object the camera recognise, pose of the object and order of recognition
	//ros::Subscriber AMS2_status_sub = n.subscribe("AMS_status", 1, AMS2_status);						//Obtains what is the current AMS state 

	// Initialize global variables
  strcpy(ip,IP);//----------------------------!!!!!!!!---TEMPORARY---!!!!!!!!------------------------------------------
  port = PORT;
  //current_time = ros::Time::now();						// sets current time to the time it is now
	//last_time = current_time;							// sets last time to the time it is now

  //Initialize Socket
	heartbeat_sock sock(ip,port);

	//sets the frequency for which the program sleeps at 1Hz(specified by task constraints)
	ros::Rate loop_rate(1);

	// ros::ok() will stop when the user inputs Ctrl+C
	while(ros::ok())
	{
		//MISSION_CONTROL_inspector();		// check that entire system is initialized before starting calculations
		ros::spinOnce();							// update subscribers

    //Send Heartbeat
    heartbeatSend(sock);

		loop_rate.sleep();							// sleep for set loop_rate
		//last_time = current_time;						// update last_time
	} // while(ros::ok())

  return 0;
}
