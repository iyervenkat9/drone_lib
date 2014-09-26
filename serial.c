#include "navdata.h"
#include "serial.h"
#include "stdint.h"
#include <time.h>

void serial_init() {	
	fd_set mask, smask;
	serial_fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY | O_SYNC);

	if (serial_fd) {
		if (fcntl(serial_fd, F_SETFL, 0) < 0) {
	    	printf("could not set fcntl");
    		return;
  		}

		if (tcgetattr(serial_fd, &options) < 0) {
		    printf("could not get options");
		    return;
		}
		/*   fprintf(stderr, "serial options set\n"); */
		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);
		/* Enable the receiver and set local mode */
		options.c_cflag |= (CLOCAL | CREAD);
		/* Mask the character size bits and turn off (odd) parity */
		options.c_cflag &= ~(CSIZE|PARENB|PARODD);
		/* Select 8 data bits */
		options.c_cflag |= CS8;

		/* Raw input */
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		/* Raw output */
		options.c_oflag &= ~OPOST;
		/* Should block */
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;
	

		if (tcsetattr(serial_fd, TCSANOW, &options) < 0) {
			printf("could not set options");
			return;
		}
		printf("All's well, clearing serial buffer for data\n");
		memset(serial_str, 0x00, SERIAL_BUFFER_SIZE);
	}	
}

void serial_read() {
	int l;
	int num_bytes = 1, ptr = 0, serial_ptr = 0, 
	cnt_read = 0, partial_read = 0, serial_len = 0;
	char escape_char = 0;
	struct timeval tv1, tv2;
	printf("Reading serial buffer\n");
	memset(serial_buffer, 0x00, SERIAL_BUFFER_SIZE);
	while (1) {			
		gettimeofday(&tv1, NULL);
		num_bytes = read(serial_fd, serial_buffer + serial_ptr,
						 SERIAL_BUFFER_SIZE);		
		gettimeofday(&tv2, NULL);
		if (num_bytes > 0) {
			int i;
			printf("cnt_read:%d, time = %f, num_bytes = %d\n", 
					cnt_read, 
					(double) (tv2.tv_usec - tv1.tv_usec)/1000000 
					+ (double) (tv2.tv_sec - tv1.tv_sec),
					num_bytes);
			cnt_read = 0;
			serial_ptr += num_bytes;
			
			if (serial_ptr >= 3) {
				if (serial_buffer[0] == 0x55 && 
					serial_buffer[1] == 0x55) {
					serial_len = serial_buffer[2];
					if (serial_ptr >= serial_len + 3) {
						// Handle input string
						handle_serial_str();
						memset(serial_buffer, 0x00, 
							   SERIAL_BUFFER_SIZE);	
						serial_ptr = 0;				
					}
				}
			}
		}
		else {
			cnt_read++;
			if (cnt_read % 1024 == 0)
				printf("0 read time = %f\n", 
					  (double) (tv2.tv_usec - tv1.tv_usec));
		}
	}
}

void handle_serial_str() {
	int i = 0, bad_data = 1;
		
	time_t timer;
	struct tm* tm_info;
	nav_gps_heading_t avg_data;
	char tmp_gps[30];

	bad_data = 0;
	time(&timer);
	tm_info = localtime(&timer);

	strftime(log_string, 27, "%Y:%m:%d:%H:%M:%S,", tm_info);
	avg_data = get_avg_heading();
	memset(tmp_gps, 0x00, 30);
	sprintf(tmp_gps, "(%4.6f, %4.6f),", 
		avg_data.gps_lat, avg_data.gps_lon);
	strncat(log_string, tmp_gps, strlen(tmp_gps));	
	strncat(log_string, serial_buffer + 3, (strlen(serial_buffer)-3));			
		
	
	printf("Log: %s", log_string);
	
	//if (bad_data) {
		//printf("len %d, bad data %s", strlen(serial_str), serial_str);	
	//}
}
