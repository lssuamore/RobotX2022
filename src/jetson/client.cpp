#include <arpa/inet.h> // inet_addr()
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h> // bzero()
#include <sys/socket.h>
#include <unistd.h> // read(), write(), close()
#define MAX 256
#define PORT 8080
#define IP "127.0.0.1"
#define SA struct sockaddr
#define MSG "RXHRB,111221,161229,21.31198,N,157.88972,W,ROBOT,2,2"

/* 
 * Method Name: beatSend
 * Purpose: send a formatted NMEA string to the RobotX Judges Server
 * Function: recieve buffer with all information to send, comma separated,
 *              calculate XOR checksum, prepend '$', append terminating '*' <XOR checksum> "<CR><LF>",
 *              and send to the specified socket
 * Matthew Bowen <mbowen2@lssu.edu>
 * 10/15/22
 */
int beatSend(int sockfd, const char *buff)
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

/*void func(int sockfd)
{
	char buff[MAX];
	int n;
	for (;;) {
		bzero(buff, sizeof(buff));
		printf("Enter the string : ");
		n = 0;
		while ((buff[n++] = getchar()) != '\n');
		write(sockfd, buff, sizeof(buff));
		bzero(buff, sizeof(buff));
		read(sockfd, buff, sizeof(buff));
		printf("From Server : %s", buff);
		if ((strncmp(buff, "exit", 4)) == 0) {
			printf("Client Exit...\n");
			break;
		}
	}
}
*/
int main()
{
	int sockfd, connfd;
	struct sockaddr_in servaddr, cli;

	// socket create and verification
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		printf("socket creation failed...\n");
		exit(0);
	}
	else
		printf("Socket successfully created..\n");
	bzero(&servaddr, sizeof(servaddr));

	// assign IP, PORT
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(IP);
	servaddr.sin_port = htons(PORT);

	// connect the client socket to server socket
	if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr))
		!= 0) {
		printf("connection with the server failed...\n");
		exit(0);
	}
	else
		printf("connected to the server..\n");

	// function for chat
	//func(sockfd);

    // function sends single heartbeat message
	beatSend(sockfd,MSG);
	
	// close the socket
	close(sockfd);
	close(connfd);
}
