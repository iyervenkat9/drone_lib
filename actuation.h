#include <stdio.h>
#include <string.h>
#include "convert.h"
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include "convert.h"
#include <errno.h>
#include <stdint.h>


#ifndef ACTUATION_H
#define ACTUATION_H
#define PORT_NO 5556


int sockfd;
int at_seq;
char at_seq_str[6];
struct sockaddr_in sock_info;
pthread_mutex_t at_mutex;
/* Initialize AT port for actuation */
void register_actuation();
void close_actuation();


/* Actuation commands for yaw, pitch, roll 
 * and vertical movements
 */
void vertical_down(float speed);
void vertical_up(float speed);
void rotate_left(float l_angle);
void rotate_right(float r_angle);
void tilt_left(float r_tilt);
void tilt_right(float r_tilt);
void tilt_forward(float r_tilt);
void tilt_backward(float r_tilt);

/* Actuation commands for continuous drone movements
 * over a longer time scale
 */ 
void go_forward(float r_tilt, int ntimes);
void clockwise(float r_angle, int ntimes);
void anti_clockwise(float l_angle, int ntimes);
void move_up(float speed, int ntimes);
void move_down(float speed, int ntimes);
void roll_left(float r_tilt, int ntimes);
void roll_right(float r_tilt, int ntimes);
void go_backward(float r_tilt, int ntimes);

/* AT commands for Takeoff and Landing */
void takeoff();
void land();

/* Calibrate Magnetometer reading */
void calibrate_magneto();
void flat_trim();
void test_leds();

/* Specify whether the user wants the 
 * full navdata or only a demo
 */
void set_navdata_options(char demo);

void set_control_yaw(float yaw_radians);
void set_outdoor(uint8_t flag);

#endif
