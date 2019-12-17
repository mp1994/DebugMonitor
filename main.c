/*********    Debug Serial Monitor    *********/
/**********************************************/

// ./monitor <PORT> <BAUDRATE> <FILENAME>
// Default: /dev/ttyUSB0 57600 monitorFile.csv

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>

#define BUFFER_SIZE 1024

char buffer[BUFFER_SIZE];
char readFlag = 1;
char filename[100] = "";
	
int8_t initPort(int *fd, int BR);
void   INThandler(int);

FILE *foutput;

int main(int argc, char *argv[]) {

	int baudrate;
	char port[25];
	time_t tempo = time(NULL);
	
	/** PARSE USER INPUT **/
	/* argv[0] = ./monitor */
	if ( argc > 1 ) {
		
		strcpy(port, argv[1]);
		baudrate = atoi(argv[2]);
		strcpy(filename, argv[3]);

	} 
	else {
	
		/* Default options */
		baudrate = 115200;
		strcpy(port, "/dev/ttyUSB0");
		if ( strlen(filename) == 0 ) {
			struct tm tm = *localtime(&tempo);
			sprintf(filename, "LOGFILE_%d%d%d_%d%d%d.csv", tm.tm_year, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		}
		
	}
	
	/** OPEN SERIAL **/
	int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	usleep(500);
	if ( fd < 0 ) {
		printf("[!] Unable to open connection with %s.\n", port);
		printf("Exiting...\n");
		return fd;
	}
	if ( initPort(&fd, baudrate) )
		printf("Connection established with: %s \n", port);
	else {
		printf("Unable to connect.\n\n");
		return -1;
	}
	
	/* Variable to Store the Number of Bytes Read from the Serial */
	int n = 0; 
	
	/* Init the Logfile */
	foutput = fopen(filename, "w+");
	if ( !foutput ) {
		printf("Unable to open file.\n");
		return -2;
	}
	fprintf(foutput, "Log file\n");
	
	/* Flush the Buffer */
	n = read(fd, buffer, sizeof(buffer));
	memset(buffer, 0, BUFFER_SIZE);
	usleep(10);

	/* Catch CTRL+C to Close the Connection Before Quitting */
	signal(SIGINT, INThandler);
		
	while( readFlag ) {
	
		/* Get Data */
		n = read(fd, buffer, sizeof(buffer));	
		/* Log */	
		if ( n > 1 ) {
			fprintf(foutput, "%s", buffer);
			printf("%s", buffer);	
		}
		/* Flush the Buffer */
		memset(buffer, 0, BUFFER_SIZE);
		
	}	
	
	printf("\nClosing connection... Goodbye!\n");
	close(fd);
	fclose(foutput);
	
	return 0;

}

void INThandler(int sig) {
	readFlag = 0;
}

int8_t initPort(int *fd, int BR) {
	
	struct termios options;
     tcgetattr(*fd,&options);

	// Set Baud Rate of communication
     switch(BR) {
     	case 9600:   cfsetispeed(&options, B9600);
     	             cfsetospeed(&options, B9600);
          		   break;
          case 19200:  cfsetispeed(&options, B19200);
     	             cfsetospeed(&options, B19200);
          		   break;
          case 38400:  cfsetispeed(&options, B38400);
          	        cfsetospeed(&options, B38400);
      	     	   break;
          case 57600:  cfsetspeed(&options, B57600);
               	   break;
          case 115200: cfsetspeed(&options, B115200);
          		   break;

          default: cfsetispeed(&options, B115200);
                   cfsetospeed(&options, B115200);
                   break;
	}

	/* Set some UART options */
	// Enable receiver (CREAD), ignore modem control lines (CLOCAL)
     options.c_cflag |= (CLOCAL | CREAD); 
     // No parity
     options.c_cflag &= ~PARENB;
     // Only 1 stop bit (8n1)
     options.c_cflag &= ~CSTOPB;
     // Set 8-bit words
     options.c_cflag &= ~CSIZE;
     options.c_cflag |= CS8;
     
     // Enable POLLING-READ
     options.c_cc[VTIME] = 0;
     options.c_cc[VMIN]  = 0;
     
     // Flush unread, unwritten data
     tcflush(*fd, TCIOFLUSH);
     
     // Apply settings
     tcsetattr(*fd,TCSANOW,&options);
     
     return 1;

}
