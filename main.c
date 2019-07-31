/***   Debug Serial Monitor   ***/
/********************************/

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

#define BUFFER_SIZE 4096

char buffer[BUFFER_SIZE];
char readFlag = 1;
char filename[100] = "monitorFile.csv";
	
int8_t initPort(int *fd, int BR);
void INThandler(int);

FILE *foutput;

int main(int argc, char *argv[]) {

	int baudrate;
	char port[25];

	if ( argc > 1 ) {
	
		strcpy(port, argv[1]);
		baudrate = atoi(argv[2]);
		strcpy(filename, argv[3]);
	
	
	} else {
	
		// Default options
		baudrate = 57600;
		strcpy(port, "/dev/ttyUSB0");
		
	}
	
	// OPEN SERIAL
	int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	usleep(500);
	if ( !fd ) {
		printf("Unable to open connection.\n");
		return fd;
	}
	if ( initPort(&fd, baudrate) )
		printf("Connection established with: %s \n", port);
	else {
		printf("Unable to connect.\n\n");
		return -1;
	}
		
	int n = 0; 
	
	// Init file 
	foutput = fopen(filename, "w+");
	if ( !foutput ) {
		printf("Unable to open file.\n");
		return -2;
	}
	fprintf(foutput, "Log file\n");
	
	// Flush the buffer due to the RESET of the AVR when opening the serial communication
	n = read(fd, buffer, sizeof(buffer));
	memset(buffer, 0, BUFFER_SIZE);
	usleep(10);

	// Catch CTRL+C and close the connection before quitting
	signal(SIGINT, INThandler);
	
	int i = 1;
	
	while( readFlag ) {
	
		n = read(fd, buffer, sizeof(buffer));
		usleep(50);
		
		if ( n > 1 ) {
			fprintf(foutput, "%s", buffer);
			printf("%s", buffer);	
		}
		
		// Log data to file 
		//fprintf(foutput, "%d : %s\n", i, buffer);
		//i++;
		
		memset(buffer, 0, BUFFER_SIZE); // Flush buffer
		
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

     switch(BR) {
     	case 9600: cfsetispeed(&options, B9600);
     	           cfsetospeed(&options, B9600);
          	break;
          case 19200: cfsetispeed(&options, B19200);
     	            cfsetospeed(&options, B19200);
          	break;
          case 38400: cfsetispeed(&options, B38400);
          	       cfsetospeed(&options, B38400);
      	     break;
          case 57600: cfsetspeed(&options, B57600);
               break;

          default: cfsetispeed(&options, B57600);
                   cfsetospeed(&options, B57600);
                   break;
	}

     options.c_cflag |= (CLOCAL | CREAD);
     options.c_cflag &= ~PARENB;
     options.c_cflag &= ~CSTOPB;
     options.c_cflag &= ~CSIZE;
     options.c_cflag |= CS8;
     tcsetattr(*fd,TCSANOW,&options);
     
     return 1;

}
