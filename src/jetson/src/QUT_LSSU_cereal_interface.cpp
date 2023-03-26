//  Filename:  QUT_cereal.cpp
//  Creation Date:  11/07/2022
//  Last Revision Date:  11/07/2022
//  Author(s) [email]:  Brad Hacker [bhacker@lssu.edu]
//  Organization/Institution:  Lake Superior State University RobotX Team AMORE
// 
// ...........................QUT_cereal.cpp.......................
//  This code subscribes to the propulsion_system code topics to concatinate the serial string and send the
//  
//
//  Inputs and Outputs of the QUT_cereal.cpp file
//				Inputs: "thruster_int_left","thruster_int_right","angle_int_left","angle_int_right" - truly PM,SM,PB,SB
//
//				Outputs: "AMS_status" - state feedback from safety management system
//
//...............................................................................................Included Libraries and Message 



#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "stdio.h"
#include "std_msgs/Bool.h"        // message type used for communicating initialization status to mission_control
#include "jetson/state.h"         // message type used to recieve state of operation from mission_control
#include "std_msgs/Float32.h"     // message type of thruster commands, and type for control efforts
#include "std_msgs/Int32.h"       // thruster commands

#include <cmath>
#include <deque>
#include <termios.h>
#include <fcntl.h>
#include "jetson/motorStatus_.h"  // message type that holds thrust x, thrust y, and moment z
#include "boost/thread.hpp"


#define MAX_SERIAL_BUFFER_SIZE 200    // Max serial buffer size



// Globals
int   motorSerialEnabled = 1;
int   serialDeviceMotorHandle;
int   motorSendCounter = 0;
int   motorArmCounter=0;


// thuster outputs
float PM;  // used to get the port (left) main thruster output
float PB;  // used to get the port (left) bow thruster output
float SM;  // used to get the starboard (right) main thruster output
float SB;  // used to get the starboard (right) bow thruster output


jetson::motorStatus_ ms;  // "AMS_status" message



// Function defines
int    process_motor_controller_message( char *p );
int    process_motor_controller_message( char *p );
void   motorSerialReaderFunc();
void   serialSendFunc();
int    sendMotorCommands();
int    sendStatusPacket();
int    set_serial_device_params( int fd, int baud );

ros::Publisher motorStatus_pub;  // "AMS_status" publisher



//.....................................................................................................................Functions.......................................................................................................................
// THIS FUNCTION: Callback for "thruster_int_left" topic
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void thruster_int_left_callback(std_msgs::Float32 port_main_thrust)
{
    PM = port_main_thrust.data;
}  // END OF thruster_int_left_callback()



// THIS FUNCTION: Subscribes to the "thruster_int_right" topic
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void thruster_int_right_callback(std_msgs::Float32 stbd_main_thrust)
{
    SM = stbd_main_thrust.data;
}  // END OF thruster_int_right_callback()



// THIS FUNCTION: Subscribes to the "angle_int_left" topic
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void angle_int_left_callback(std_msgs::Float32 port_bow_thrust)
{
    PB = port_bow_thrust.data;
}  // END OF angle_int_left_callback()



// THIS FUNCTION: Subscribes to the "angle_int_right" topic
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void angle_int_right_callback(std_msgs::Float32 stbd_bow_thrust)
{
    SB = stbd_bow_thrust.data;
}  // END OF angle_int_right_update()



// Main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "QUT_cereal");  // names the program for visual purposes

    // NodeHandles
    ros::NodeHandle nh1;

    // Subscribers from propulsion_system
    ros::Subscriber thruster_int_left_sub = nh1.subscribe("thruster_int_left", 1, thruster_int_left_callback);
    ros::Subscriber thruster_int_right_sub = nh1.subscribe("thruster_int_right", 1, thruster_int_right_callback);
    ros::Subscriber angle_int_left_sub = nh1.subscribe("angle_int_left", 1, angle_int_left_callback);
    ros::Subscriber angle_int_right_sub = nh1.subscribe("angle_int_right", 1, angle_int_right_callback);


    // Publishers for motor commands
    motorStatus_pub = nh1.advertise<jetson::motorStatus_>("AMS_status", 1);  // state feedback from safety management system
	

    //std::string serialPortMotorName = "/dev/motors";
    std::string serialPortMotorName = "/dev/ttyUSB1";
	int serialBaudRateMotors = 115200;

    // Open the serial port
    if ((serialDeviceMotorHandle = open(serialPortMotorName.c_str(), O_RDWR | O_NOCTTY)) < 0) {
		ROS_ERROR("FAILED TO OPEN MOTOR SERIAL PORT: %s", serialPortMotorName.c_str());
        return 1;
    } else {
        if (set_serial_device_params(serialDeviceMotorHandle, serialBaudRateMotors) == 1)
    	return 1;
    }
    ROS_INFO("Motor Serial Port Successfully configured");

    // Start the serial reader and writer threads
	boost::thread motorSerialReaderThread(motorSerialReaderFunc);
	boost::thread serialSendThread(serialSendFunc);

	std::cout << "main: starting motor interface thread" << std::endl;
	motorSerialReaderThread.join();
    serialSendThread.join();
    
    return 0;
}  // END OF main()




/**
 * Serial reader and processing thread for incoming packets from the motor controller
 */
void motorSerialReaderFunc() {
    unsigned char 	sync_cnt = 0;
    unsigned char 	packetLen = 0;
    int			endPacket = 0;
    char			pk[ MAX_SERIAL_BUFFER_SIZE ];

    ROS_INFO( "motor_serial_reader_function: running" );

    while ( ros::ok() ) {
        int			val;
        unsigned char	rxByte;

        val = read( serialDeviceMotorHandle, &rxByte, 1 );

        if ( val > 0 ) { // New character received

            if ( 1 ) printf( "%c", rxByte );

            if ( rxByte == '$' ) {
                sync_cnt = 0;
                packetLen = 0;
            }
            pk[ packetLen ] = rxByte;
            packetLen++;

            if ( packetLen >= ( MAX_SERIAL_BUFFER_SIZE - 2 ) ) {
                sync_cnt = 0;
                packetLen = 0;
            }

            if ( (rxByte == '\n') && (sync_cnt == 1) ) {
                if ( 0 ) {
                    for ( int i=0; i<packetLen; i++ )
                        if ( 1 ) printf( "%c", pk[i] );
                }
                if ( 1 ) {
                    process_motor_controller_message( pk );
                    sync_cnt = 0;
                    if ( 0 ) ROS_INFO( "VALID NMEA PACKET RECEIVED" );
                } else {
                    if ( 1 ) ROS_INFO( "ERROR: INVALID NMEA PACKET RECEIVED" );
                }
            } else {
                sync_cnt = 0;
            }

            if ( rxByte == '\r' ) {
                sync_cnt = 1;
            }
        }
    }
}




// Send the motor command and arming packets to the serial port with correct checksum
int send_motor_message( int* cmd )
{
    int  i = 0;
    int  crc = 0;
    char tmpMsg[ 40 ];
    char tmpMsgMC[ 40 ];
    bool motorSerialEnabled = true;
	 
	int serialBaudRateMotors = 115200;
	bool verboseDEMANDS = true;
	int MOTOR_MAX_MAIN_PERCENTAGE = 100;
	enum { MOTOR_PORT_MAIN = 0, MOTOR_STBD_MAIN = 1, MOTOR_PORT_BOW=2, MOTOR_STBD_BOW=3};

    if ( !motorSerialEnabled ) {
        ROS_WARN( "WRITING MOTOR DEMANDS DISABLED");
        return 0;
    }
	
    // Wait 1.5 seconds on startup so the motor board bootloader
    // doesn't get smashed too early with serial data
    if ( motorSendCounter++ < (int)( 10 * 1.5 ) ) {
          if ( verboseDEMANDS ) ROS_INFO( "WAITING FOR MOTOR CONTROLLER BOOTLOADER" );
          return 1;
    }

    // Periodically send the arming packet to the micro. This can be a lot smarter,
    // but for now just send about every two seconds.
    if ( motorArmCounter++ >= (int)( 10 * 2.0) ) {
        sprintf( tmpMsg, "$MA,1,%d,", (int)( MOTOR_MAX_MAIN_PERCENTAGE ) );
        motorArmCounter = 0;

        // Calculate the checksum
        i = 1;
        crc = 0;
        while ( tmpMsg[ i ] != '\0' ) {
            crc^=tmpMsg[ i++ ];
        }
        // Append the checksum to the end of the packet
	    sprintf( tmpMsg, "$MA,1,%d,*%02X\r\n",  (int)( MOTOR_MAX_MAIN_PERCENTAGE ), crc );

        i = 0;
        while( tmpMsg[i] != '\0' ) {
            write( serialDeviceMotorHandle, &tmpMsg[ i++ ], 1 );
        }
        if ( verboseDEMANDS ) ROS_INFO( "ARMING THE MOTOR CONTROLLER" );
    }

    // Prepare the packet. Note it currently does not include a checksum
    sprintf( tmpMsgMC, "$MC,%d,%d,%d,%d", cmd[MOTOR_PORT_MAIN], cmd[MOTOR_STBD_MAIN], cmd[MOTOR_PORT_BOW], cmd[MOTOR_STBD_BOW] );
    // Calculate the checksum
    i = 1;
    crc = 0;
    while ( tmpMsgMC[ i ] != '\0' ) {
        crc^=tmpMsgMC[ i++ ];
    }
    // Append the checksum to the end of the packet
    sprintf( tmpMsgMC, "$MC,%d,%d,%d,%d*%02X\r\n", cmd[MOTOR_PORT_MAIN], cmd[MOTOR_STBD_MAIN], cmd[MOTOR_PORT_BOW], cmd[MOTOR_STBD_BOW], crc );
  
    // Send the packet
    i = 0;
    while( tmpMsgMC[ i ] != '\0' ) {
        write( serialDeviceMotorHandle, &tmpMsgMC[ i++ ], 1 );
    }
    if ( verboseDEMANDS ) ROS_INFO( "WRITING MOTOR DEMANDS: %s", tmpMsgMC );
    return 0;
}




// Serial motor packet sending thread, runs at 10 Hz
void serialSendFunc() {
    int cmd[4];
    ros::Rate loop_rate(10);
    while(ros::ok())  // ros::ok() will be false when the user inputs Ctrl+C
    {
        // int cmd[4] = {PM, SM, PB, SB};
        cmd[0] = (int)(floor(PM));
        cmd[1] = (int)(floor(SM));
        cmd[2] = (int)(floor(PB));
        cmd[3] = (int)(floor(SB));
        send_motor_message(cmd);
        ros::spinOnce();            // update subscribers
        loop_rate.sleep();          // sleep to accomplish set loop_rate
	}
}




/**
 * Parses the serial packet received from the motor controller
 */
int process_motor_controller_message( char *p ) {
    int	i, tmp1, tmp2, tmp3, tmp4;
    char	msgHdr[7];
    int	motorStatus = 0;
    int	motorMode = 0;
    int motor_control_mode;
    jetson::motorStatus_ ms;

    // Determine the NMEA header format
    for ( i=0; i<6; i++ ) {
        if ( p[i] == ',' ) {
            break;
        }
        msgHdr[i] = p[i];
    }
    msgHdr[i] = '\0';

    if ( strcmp( msgHdr, "$MS" ) == 0 ) {
        sscanf( p, "$MS,%d,%d,%d,%d,%d,%d,", &tmp1, &tmp2, &tmp3, &tmp4, &motorStatus, &motorMode );
        ms.seqno++;
        ms.motorLeftAct = tmp1;
        ms.motorRightAct = tmp2;
        ms.motorStatus = motorStatus;
        ms.motorMode = motorMode;
        if (motorMode == 2)
        {
        motor_control_mode = 0;
        }
        else if (motorMode == 1)
        {
            motor_control_mode = 2;
        }
        else if (motorMode == 0)
        {
            motor_control_mode = 1;
        }
		
        // Publish the motor status message
        motorStatus_pub.publish( ms );
        if ( 1 ) ROS_INFO( "VALID $MS MESSAGE RECEIVED." );
        if ( 1 ) ROS_INFO( "%d  %d  %d  %d  %d  %d", tmp1, tmp2, tmp3, tmp4, motorStatus, motorMode);
        return 1;
    }
    return 0;
}




/**
 * Initialise the serial ports for communicating to the motor microcontroller
 * and radio.
 */
int set_serial_device_params( int fd, int baud )
{
    struct	termios	options;

    fcntl( fd, F_SETFL, 0 );
    tcgetattr( fd, &options );
    options.c_cflag |= (CLOCAL | CREAD);
    // Set the options for the serial port
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~(CSIZE);
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON | ECHO | IEXTEN | ECHONL | ECHOE | ISIG);

    options.c_iflag &= ~(IXON);
    options.c_iflag &= ~(IXOFF);
    options.c_iflag &= ~(ICRNL);
    options.c_iflag &= ~(INLCR);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 5;

    switch ( baud ) {
        case 9600:
            cfsetispeed( &options, B9600 );
            cfsetospeed( &options, B9600 );
            break;
        case 19200:
            cfsetispeed( &options, B19200 );
            cfsetospeed( &options, B19200 );
            break;
        case 38400:
            cfsetispeed( &options, B38400 );
            cfsetospeed( &options, B38400 );
            break;
        case 57600:
            cfsetispeed( &options, B57600 );
            cfsetospeed( &options, B57600 );
            break;
        case 115200:
            cfsetispeed( &options, B115200 );
            cfsetospeed( &options, B115200 );
            break;
        default:
            ROS_INFO( "ERROR: INVALID BAUD RATE" );
            return -1;
    }

    ROS_INFO( "SERIAL BAUD RATE SET TO: %d", baud );

    if ( tcsetattr( fd, TCSANOW, &options ) < 0 ) {
        ROS_INFO( "FAILED TO SET SERIAL DEVICE OPTIONS" );
        close( fd );
        return 1;
    } else {
        ROS_INFO( "SUCCESSFULLY SET SERIAL DEVICE OPTIONS" );
    }

    // Flush the serial port input and output buffers
    tcflush( fd, TCIOFLUSH );
    return 0;
}


