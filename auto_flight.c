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
	int l, cnt = 0, calibrate_mag = 1;
	int num_bytes, ptr = 0;
	nav_gps_heading_t  loop_nav;

	pthread_t pt, serial_thread;

    register_actuation();
    set_outdoor(0);
    nav_port_init();
    int ret_val = pthread_create(&pt, NULL, nav_thread, NULL);
        
	if (ret_val) 
		printf("Thread creation problem\n");
	else
	{
		printf("Created thread\n");
		pthread_create(&serial_thread, NULL, serial_read_thread, NULL);
		flat_trim();
		sleep(3);
		
		takeoff();
		sleep(15);
		move_up(1,10);
		
		if (calibrate_mag) {
			calibrate_magneto();
			sleep(2);
			calibrate_yaw();
			//sleep(2);
			set_drone_heading(0);
		} else {			
			calibrate_magneto();
			sleep(2);
			calibrate_yaw();
			sleep(2);
			uint8_t waypoint_ptr = 0;
			while (waypoint_ptr < num_waypoints) {
				navigate_next(waypoint_ptr);
				waypoint_ptr++;
			}
		}
/*		
		sleep(2);					
		loop_nav = get_avg_heading();
		printf("before turn, avg gps_lat: %4.6f, gps_lon: %4.6f, heading: %4.3f,\
					wind_speed: %4.3f, wind_angle: %4.3f\n",  
					loop_nav.gps_lat, loop_nav.gps_lon, loop_nav.heading,
					loop_nav.wind_speed, loop_nav.wind_angle);										
		loop_nav = get_avg_heading();
			printf("after turn, avg gps_lat: %4.6f, gps_lon: %4.6f, heading: %4.3f,\
					wind_speed: %4.3f, wind_angle: %4.3f\n",  
					loop_nav.gps_lat, loop_nav.gps_lon, loop_nav.heading,
					loop_nav.wind_speed, loop_nav.wind_angle);		
		int loop_cnt = 0;
		while (loop_cnt < 20) {
			loop_nav = get_avg_heading();
			printf("avg gps_lat: %4.6f, gps_lon: %4.6f, heading: %4.3f,\
					wind_speed: %4.3f, wind_angle: %4.3f\n",  
					loop_nav.gps_lat, loop_nav.gps_lon, loop_nav.heading,
					loop_nav.wind_speed, loop_nav.wind_angle);
			sleep(1);
			loop_cnt++;
		}
*/		
		land();
	}
	close_actuation();
	pthread_exit(NULL);
 }
 
 
