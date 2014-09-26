#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "actuation.h"
#include "navdata.h"
#include "serial.h"
#include <pthread.h>

#define YAW_MAX_DEGREE 80

void * nav_thread(void *arg) {
	printf("Inside thread\n");
	nav_read();
	return NULL;
}


void * fill_nav_buf(void *arg) {
	nav_read_buf();
	return NULL;
}

void * serial_read_thread(void *arg) {
	serial_init();
	serial_read();
	return NULL;
}


void main() {
	char watchdog[] = "AT*COMWDG=1\r";
	char ctrl[] = "AT*CTRL=1,5,0\r";
	int l, cnt = 0, calibrate_mag = 0;
	int num_bytes, ptr = 0, ret_val= 0;
	nav_gps_heading_t  loop_nav;

	pthread_t pt, serial_thread;

	register_actuation();
	printf("Initialized AT port\n");

	nav_port_init();
	printf("Initialized nav port\n");
	ret_val = pthread_create(&pt, NULL, nav_thread, NULL);
    #if ON_PC
		printf("gcc\n");
    #else
		printf("arm-gcc\n");
    #endif
    

	if (ret_val) 
		printf("Thread creation problem\n");
	else
	{
		printf("Created thread\n");
		sleep(6);
		calibrate_yaw();
		//test_leds();
		//printf("test leds\n");
		pthread_create(&serial_thread, NULL, serial_read_thread, NULL);
		//sleep(10);
		//takeoff();
		//printf("takeoff\n");
		//sleep(6);		
		////calibrate_magneto();
		////sleep(6);
		////set_drone_heading(0);
		////travel_distance(0, 7.0);
		
		//go_forward(1,10);
		//sleep(10);
		
		//*go_forward(1,15);
		//sleep(10);
		//go_forward(1,15);
		//sleep(10);
		//*/
		//sleep(60);
		//land();
		//printf("landed\n");
		
		while (1) sleep(10);
	}
	close_actuation();
	pthread_exit(NULL);
 }
 
 
