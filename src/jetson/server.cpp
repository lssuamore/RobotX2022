

#include <stdio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

#define MAX 256
#define PORT 8080
#define SA struct sockaddr

static volatile char keepRunning = 1;
int sockfd, connfd, len;

//handle ctrl+c interrupt
void intHandler(int dummy)
{
	keepRunning = 0;
	// After chatting close the socket
	close(sockfd);
	close(connfd);
}

/* 
 * Method Name: beatRecv
 * Purpose: Recieve and verify a formatted NMEA string simulating the RobotX Judges Server
 * Function: recieve a packet formatted as an NMEA string, 
 * 				verify that it starts with a '$' and ends with a "*<CR><LF><XOR_CHECKSUM>"
 * Matthew Bowen <mbowen2@lssu.edu>
 * 10/15/22
 */
int beatRecv(int sockfd, char *buff)
{
    char packet[MAX];
    unsigned char checksum = 0;
    char formatstring[MAX];
    
	bzero(buff,MAX);
	bzero(packet,MAX);

    //read heartbeat from the client
    //read one byte at a time
	char curByte = 0;
	int buffLen = 0;
	//read the packet
	while (curByte != '\n' && buffLen++ < 256)
	{
		if(read(sockfd,(void*)&curByte,1) < 0)
		{
			perror("Socket Read Error: ");
			break;
		}
		//append the characters one by one
		sprintf(formatstring,"%c\0", curByte);
		strcat(packet,formatstring);
	}
	
    //check for propper open and close to message
    if(packet[0]!='$')
        return 1;

	//check if '*' was found
    if(strstr(packet,"*") == nullptr || buffLen > 256)
        return -1;

	
    //copy informational contents of the packet to the buffer
    strncpy(buff, &packet[1], buffLen-6);
    buff[buffLen] = '\0';
	
/*    //check the checksum
    //calculate
    for(int i=0; i<buffLen-6; i++)
	{
		printf("%c",buff[i]);
        checksum ^= buff[i];
	}
	printf("\n\n");

    int inCS;
	char textCS[3];
	strncpy(textCS, packet+buffLen+0, 2);
	textCS[2]='\0';
    sscanf(textCS, "%X", &inCS);

    char expectedCS[3];
	sprintf(expectedCS, "%2X", checksum);

	//compare
    //if (inCS != checksum)
	if(strncmp(textCS,expectedCS,2))
	{
		printf("Expected Checksum: %2X\n \nRecieved Checksum: %s", checksum, textCS);
        return 2;
	}
	*/
    return 0;
}

/*// Function designed for chat between client and server.
void func(int connfd)
{
	char buff[MAX];
	int n;
	// infinite loop for chat
	for (;;) {
		bzero(buff, MAX);

		// read the message from client and copy it in buffer
		read(connfd, buff, sizeof(buff));
		// print buffer which contains the client contents
		printf("From client: %s\t To client : ", buff);
		bzero(buff, MAX);
		n = 0;
		// copy server message in the buffer
		while ((buff[n++] = getchar()) != '\n')
			;

		// and send that buffer to client
		write(connfd, buff, sizeof(buff));

		// if msg contains "Exit" then server exit and chat ended.
		if (strncmp("exit", buff, 4) == 0) {
			printf("Server Exit...\n");
			break;
		}
	}
}
*/

// Driver function
int main()
{
	struct sockaddr_in servaddr, cli;

	signal(SIGINT, intHandler);

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
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servaddr.sin_port = htons(PORT);

	// Binding newly created socket to given IP and verification
	if ((bind(sockfd, (SA*)&servaddr, sizeof(servaddr))) != 0) {
		printf("socket bind failed...\n");
		close(sockfd);
		exit(0);
	}
	else
		printf("Socket successfully binded..\n");


    
	// Function for chatting between client and server
	//func(connfd);

    char buff[256];
	//socket read
	while(keepRunning)
	{
		// Now server is ready to listen and verification
		if ((listen(sockfd, 5)) != 0) {
			printf("Listen failed...\n");
			close(sockfd);
			exit(0);
		}
		else
			printf("Server listening..\n");
		len = sizeof(cli);

		// Accept the data packet from client and verification
		connfd = accept(sockfd, (SA*)&cli, (socklen_t *)&len);
		if (connfd < 0) {
			printf("server accept failed...\n");
			close(sockfd);
			exit(0);
		}
		else
			printf("server accept the client...\n");

		//Check the packet for requirements
		switch (beatRecv(connfd, buff))
		{
		case 1:
			printf("Recieved: Missing '$'\n");
			break;
		case -1:
			printf("Recieved: Missing '*'\n");
			break;
		case 2:
			printf("Recieved: Bad Checksum\n");
			break;
		default:
			printf("Recieved: Correct Format\n");
		}
		printf("Buffer Length: %d\n",strnlen(buff,256));
		printf("Message: %s\n\n", buff);
		close(connfd);
	}
	return 0;
}
