#include "navdata.h"
#include "actuation.h"
#include <pthread.h>

char nav_init[] = {0x01, 0x00, 0x00, 0x00};

static uint16_t minval(uint16_t a, uint16_t b) {
    if (a < b) return a;
    else       return b;
}

void enable_navdata_print(uint8_t tagp) {
    navdata_tag_mask |= ((uint32_t) 1) << tagp;
}

void disable_navdata_print(uint8_t tagp) {
    navdata_tag_mask ^= ((uint32_t) 1) << tagp;
}

void print_nav_data(uint16_t tagp, uint16_t sizep, uint8_t *n) {
    switch(tagp) {
        case ARDRONE_NAVDATA_DEMO_TAG:
                ;nav_demo d;
                memcpy(&d, (uint8_t *) n, minval(sizep, sizeof(nav_demo)));
                // printf("battery: %u, thetha: %4.3f, phi: %4.3f, psi: %4.3f\n",
                //         d.vbat_flying_percentage, d.theta, d.phi, d.psi);
        break;
        case ARDRONE_NAVDATA_TIME_TAG:
        break;
        case ARDRONE_NAVDATA_RAW_MEASURES_TAG:
        break;
        case ARDRONE_NAVDATA_PHYS_MEASURES_TAG:
        break;
        case ARDRONE_NAVDATA_GYROS_OFFSETS_TAG:
        break;
        case ARDRONE_NAVDATA_EULER_ANGLES_TAG:
        break;
        case ARDRONE_NAVDATA_REFERENCES_TAG:
        break;
        case ARDRONE_NAVDATA_TRIMS_TAG:
        break;
        case ARDRONE_NAVDATA_RC_REFERENCES_TAG:
        break;
        case ARDRONE_NAVDATA_PWM_TAG:
        break;
        case ARDRONE_NAVDATA_ALTITUDE_TAG:
                ; altitude alt;
                memcpy(&alt, (uint8_t *) n, minval(sizep, sizeof(altitude)));
                // printf("obs_alt: %4.3f\n", alt.obs_alt);
        break;
        case ARDRONE_NAVDATA_VISION_RAW_TAG:
        break;
        case ARDRONE_NAVDATA_VISION_OF_TAG:
        break;
        case ARDRONE_NAVDATA_VISION_TAG:
        break;
        case ARDRONE_NAVDATA_VISION_PERF_TAG:
        break;
        case ARDRONE_NAVDATA_TRACKERS_SEND_TAG:
        break;
        case ARDRONE_NAVDATA_VISION_DETECT_TAG:
        break;
        case ARDRONE_NAVDATA_WATCHDOG_TAG:
        break;
        case ARDRONE_NAVDATA_ADC_DATA_FRAME_TAG:
        break;
        case ARDRONE_NAVDATA_VIDEO_STREAM_TAG:
        break;
        case ARDRONE_NAVDATA_GAME_TAG:
        break;
        case ARDRONE_NAVDATA_PRESSURE_RAW_TAG:
        break;
        case ARDRONE_NAVDATA_MAGNETO_TAG:
                ;magneto *mag = (magneto *) n;
                // printf("magneto_x: %d, magneto_y: %d, magneto_z: %d, angle: %f, calib: %d\n", 
                        // mag->mx, mag->my, mag->mz, atan2(1.0*(mag->my),mag->mx)*180.0/3.1416, mag->magneto_calibration_ok);
//                printf("magneto_hd: %4.3f, magneto_gy: %4.3f, magneto_fu: %4.3f\n", 
  //                    mag->heading_unwrapped, mag->heading_gyro_unwrapped, mag->heading_fusion_unwrapped);
                gps_heading_info[struct_ptr].heading = mag->heading_unwrapped;
        break;
        case ARDRONE_NAVDATA_WIND_TAG:
				;wind *wind_data = (wind *) n;
				gps_heading_info[struct_ptr].wind_angle = wind_data->wind_angle;
				gps_heading_info[struct_ptr].wind_speed = wind_data->wind_speed;
        break;
        case ARDRONE_NAVDATA_KALMAN_PRESSURE_TAG:
        break;
        case ARDRONE_NAVDATA_HDVIDEO_STREAM_TAG:
        break;
        case ARDRONE_NAVDATA_WIFI_TAG:
        break;  
        case ARDRONE_NAVDATA_GPS_TAG:
            ;gps *gps_data = (gps *) n;
            //printf("gps lat: %f, gps lon: %f, gps elevation %f\n", gps_data->lat, gps_data->lon, gps_data->elevation);
            gps_heading_info[struct_ptr].gps_lat = gps_data->lat;
            gps_heading_info[struct_ptr].gps_lon = gps_data->lon;
        break;
        case ARDRONE_NAVDATA_CKS_TAG:
        break;
        default:
        break;
    }
}


void nav_port_init() {
	navsock = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);
	memset(&navsock_info, 0, sizeof(navsock_info));

	navsock_info.sin_family = AF_INET;
#if ON_PC	
	navsock_info.sin_port = htons(NAV_PORT);
#else
	navsock_info.sin_port = htons(LOCAL_NAV_PORT);
#endif
	navsock_info.sin_addr.s_addr = htonl(INADDR_ANY);

	if(bind(navsock, (struct sockaddr*) &navsock_info, 
			sizeof(navsock_info)) == -1)
	{
		printf("could not bind navsocket\n");
		close(navsock);
		return;
	}
	printf("bind worked\n");
	memset(&navdata_info, 0x00, sizeof(navdata_info));
	navdata_info.sin_family = AF_INET;
	navdata_info.sin_port = htons(NAV_PORT);
	navdata_info.sin_addr.s_addr = inet_addr("192.168.1.3");

	int nav_datalen = sizeof(navdata_info);
	printf("Initializing nav port\n");
	// Initialize the Navigation port, send some commands
	int num_bytes = sendto(navsock, nav_init, sizeof(nav_init), 0, 
		(struct sockaddr *) &navdata_info, sizeof(navdata_info));

	if (num_bytes == -1) 
		printf("can't initialize nav port - 1\n");
	else	
	{	
		set_navdata_options(0);
	}

	enable_navdata_print(ARDRONE_NAVDATA_DEMO_TAG);
    enable_navdata_print(ARDRONE_NAVDATA_GPS_TAG);
    enable_navdata_print(ARDRONE_NAVDATA_MAGNETO_TAG);
    
    /* Read Navigation waypoint data from file */
    char filebuffer[50], filechar;
#if ON_PC
	int fp = open("./waypoints.txt", O_RDONLY);
#else
	int fp = open("/data/video/waypoints.txt", O_RDONLY);
#endif    
	int retval = read(fp, &filechar, 1);
	float gps_lat, gps_lon;
	int i = 0, ptr = 0;
	while (retval)
	{
		if (filechar == 0x0a || filechar == 0x0d)
		{
			filebuffer[ptr++] = filechar;
			retval = sscanf(filebuffer, "%f,%f\n", &gps_lat, &gps_lon);
			if (retval > 0) {
				gps_points[i].gps_lat = gps_lat;
				gps_points[i].gps_lon = gps_lon;
				i++;
				memset(filebuffer, 0x00, sizeof(filebuffer));
				ptr = 0;
			}
		}
		else
			filebuffer[ptr++] = filechar;
		
		retval = read(fp, &filechar, 1);
	}
	
	close(fp);
	num_waypoints = i;
	wptr = 0;
	for (; i < MAX_WAYPOINTS; i++) {
		gps_points[i].gps_lat = -1.0;
		gps_points[i].gps_lon = -1.0;
	}	
	close(fp);

    pthread_mutex_init(&gps_heading_mutex, NULL);
}


void nav_read() {
	int num_bytes = sendto(navsock, nav_init, sizeof(nav_init), 0, 
		(struct sockaddr *) &navdata_info, sizeof(navdata_info));

	int l = sizeof(navdata_info);
	uint16_t cnt_size, sizep, tagp; 
    struct_ptr = 0; nav_resolution_ptr = 0;
	while (1) {		
		memset(navdata_buffer, 0x00, NAV_BUFFER_SIZE);
		num_bytes = recvfrom(navsock, navdata_buffer, NAV_BUFFER_SIZE, 0, 
			(struct sockaddr *) &navfrom, &l);
		/* Added in a filter to sample nav data 
		 * every 100 ms approximately
		 */
		nav_resolution_ptr = (nav_resolution_ptr + 1) % NAV_DATA_RESOLUTION;
		cnt_size = 0;
		if ((num_bytes > 0) && !nav_resolution_ptr) {
            uint8_t *ndata = (uint8_t *) (navdata_buffer);
            cnt_size += 8;
            ndata =  ((uint8_t *) ndata) + 0x10;

            tagp = *((uint8_t *) ndata) + (*(((uint8_t *) ndata) + 1) << 8);
            sizep = *(((uint8_t *) ndata) + 2) + (*(((uint8_t *) ndata) + 3) << 8);
            while (cnt_size < sizeof(navdata_buffer) && (tagp != 0xffff)) {        
                print_nav_data(tagp, sizep, (uint8_t *) ndata);
                ndata = ((uint8_t *) ndata) + sizep;
                cnt_size += sizep;
                tagp = *((uint8_t *) ndata) + (*(((uint8_t *) ndata) + 1) << 8);
                sizep = *(((uint8_t *) ndata) + 2) + (*(((uint8_t *) ndata) + 3) << 8);
            }
			
				
		
           fflush(stdout);
           num_bytes = sendto(navsock, nav_init, sizeof(nav_init), 0, 
            (struct sockaddr *) &navdata_info, sizeof(navdata_info));
           struct_ptr = (struct_ptr + 1) % GPS_STRUCT_MAX_ELEMENTS;
           //if (!struct_ptr)
           //{
				//nav_gps_heading_t avg_gps = get_avg_heading();
				
				//printf("avg gps lat = %4.6f, avg gps lon = %4.6f\n",
						//avg_gps.gps_lat, avg_gps.gps_lon);
		   //}
       }
	}
}

void nav_read_buf() {
	int num_bytes = sendto(navsock, nav_init, sizeof(nav_init), 0, 
		(struct sockaddr *) &navdata_info, sizeof(navdata_info));

	int l = sizeof(navdata_info);
	uint16_t cnt_size, sizep, tagp; 
	memset(gps_heading_info, 0x00, sizeof(nav_gps_heading_t)*GPS_STRUCT_MAX_ELEMENTS);
    struct_ptr = 0;
	while (struct_ptr < GPS_STRUCT_MAX_ELEMENTS) {		
		memset(navdata_buffer, 0x00, NAV_BUFFER_SIZE);
		num_bytes = recvfrom(navsock, navdata_buffer, NAV_BUFFER_SIZE, 0, 
			(struct sockaddr *) &navfrom, &l);
		cnt_size = 0;
		if (num_bytes > 0) {
            uint8_t *ndata = (uint8_t *) (navdata_buffer);
            cnt_size += 8;
            ndata =  ((uint8_t *) ndata) + 0x10;

            tagp = *((uint8_t *) ndata) + (*(((uint8_t *) ndata) + 1) << 8);
            sizep = *(((uint8_t *) ndata) + 2) + (*(((uint8_t *) ndata) + 3) << 8);
            while (cnt_size < sizeof(navdata_buffer) && (tagp != 0xffff)) {        
                //printf("tag: %d, size: %d\n", tagp, sizep);
                print_nav_data(tagp, sizep, (uint8_t *) ndata);
                ndata = ((uint8_t *) ndata) + sizep;
                cnt_size += sizep;
                tagp = *((uint8_t *) ndata) + (*(((uint8_t *) ndata) + 1) << 8);
                sizep = *(((uint8_t *) ndata) + 2) + (*(((uint8_t *) ndata) + 3) << 8);
            }

           fflush(stdout);
           num_bytes = sendto(navsock, nav_init, sizeof(nav_init), 0, 
            (struct sockaddr *) &navdata_info, sizeof(navdata_info));
           struct_ptr = struct_ptr + 1;
       }
	}
}

nav_gps_heading_t get_avg_heading() {
    nav_gps_heading_t avg_gps_hd;

    float avg_heading = 0;
    float avg_lon = 0;
    float avg_lat = 0;
    float avg_wind_speed = 0, avg_wind_angle = 0;
    uint8_t i;

    for (i = 0; i < GPS_STRUCT_MAX_ELEMENTS; i++) {
        avg_heading = avg_heading + gps_heading_info[i].heading;
        avg_lon = avg_lon + gps_heading_info[i].gps_lon;
        avg_lat = avg_lat + gps_heading_info[i].gps_lat;
        avg_wind_speed = avg_wind_speed + gps_heading_info[i].wind_speed;
        avg_wind_angle = avg_wind_angle + gps_heading_info[i].wind_angle;
	}
        

    avg_heading = avg_heading*1.0 / GPS_STRUCT_MAX_ELEMENTS;     
    avg_lon = avg_lon*1.0 / GPS_STRUCT_MAX_ELEMENTS;
    avg_lat = avg_lat*1.0 / GPS_STRUCT_MAX_ELEMENTS;
    avg_wind_speed = avg_wind_speed*1.0/GPS_STRUCT_MAX_ELEMENTS;
    avg_wind_angle = avg_wind_angle*1.0/GPS_STRUCT_MAX_ELEMENTS;

    avg_gps_hd.heading = avg_heading;
    avg_gps_hd.gps_lon = avg_lon;
    avg_gps_hd.gps_lat = avg_lat;
    avg_gps_hd.wind_speed = avg_wind_speed;
    avg_gps_hd.wind_angle = avg_wind_angle;
    return avg_gps_hd;
}


void navigate_next(uint8_t waypoint_ptr) {
	int count = 0;
	float dist = 1000.0;
	wptr = waypoint_ptr;
	dist = get_distance(gps_points[waypoint_ptr].gps_lat, 
						gps_points[waypoint_ptr].gps_lon);
	printf("Initial distance = %f\n", dist);
	// 2 times - 1 m -- roughly
	while (dist > 5.0 && count<7) {
		count++;		
		float ref = get_bearing(waypoint_ptr);
		set_drone_heading(ref);
		int ntimes = dist*2;
		go_forward(1,ntimes);
		sleep(2);
		dist = get_distance(gps_points[waypoint_ptr].gps_lat,
							gps_points[waypoint_ptr].gps_lon);
		printf("distance = %f, bearing = %f\n", dist, ref);		
	}
}

void travel_distance(float ref, float req_distance) {
	int count, ntimes;
	nav_gps_heading_t avg_heading;
	float diff_dist, dist;
	count = 0;
	set_drone_heading(ref);
	avg_heading = get_avg_heading();
	
	dist = 0;
	diff_dist = req_distance - dist;					
	if (dist > req_distance)
		printf("Initial distance = %f\n", dist);
	// 2 times - 1 m -- roughly
	while (diff_dist >= 5.0 && count<7) {
		count++;		
				
		if (diff_dist >= 0) {
			ntimes = diff_dist*2;
			if (ntimes > 20)
				ntimes = 1;
			go_forward(1,ntimes);
		}	
		else {
			ntimes = -diff_dist*2;
			if (ntimes > 20)
				ntimes = 1;
			go_backward(1, ntimes);
		}	
		sleep(2);
		dist = get_distance(avg_heading.gps_lat, 
						avg_heading.gps_lon);
		diff_dist = req_distance - dist;
		printf("distance = %f, ntimes = %d\n", dist, ntimes);		
	}
}



void set_drone_heading(float ref) {
	// ref: 0 = North, 90 = East, -90 = West, +-180 = South
	float err_angle, intended_heading = ref;
	nav_gps_heading_t avg_data;	
	float avg_heading;

	int count = 0;
	int nr_times, clockwise_correction = 0;
	avg_data = get_avg_heading();
	avg_heading = avg_data.heading;
	
	//avg_heading = -40.102;

	err_angle = intended_heading - avg_heading;
	/* Get the sign of the error and the absolute value */
	if (err_angle < 0)			
		err_angle = -err_angle;
	else
		clockwise_correction = 1;


    while (count < 3 && err_angle >= 3)
    {
    	count++;
    	
    	if (clockwise_correction == 1)
			/* Check whether the drone needs to rotate more than once.
			 * If so, determine nr_times from err_angle. 
			 * If the drone is within 'yaw_calibration_p' degrees from 
			 * the true value, then determine the fraction of full-scale
			 * rotation
			 */ 
			if (err_angle > yaw_calibration_p)			
			{			
				nr_times =  err_angle / yaw_calibration_p;			
				if (nr_times > 10)
					nr_times = 1;
				err_angle = intended_heading - avg_heading;	
				printf("setpoint %4.3f: avg heading %4.3f: diff: %4.3f, nrtimes: %d\n",
						intended_heading, avg_heading, err_angle, nr_times);
						
				clockwise(1.0, nr_times);				
			}		
			else					
			{
				float frac_angle = err_angle / yaw_calibration_p;				
				err_angle = intended_heading - avg_heading;	
				printf("setpoint %4.3f: avg heading %4.3f: diff: %4.3f, frac_angle: %4.3f\n",
						intended_heading, avg_heading, err_angle, frac_angle);
				clockwise(frac_angle, 1);
			}
		else
			/* Check whether the drone needs to rotate more than once.
			 * If so, determine nr_times from err_angle. 
			 * If the drone is within 'yaw_calibration_n' degrees from 
			 * the true value, then determine the fraction of full-scale
			 * rotation
			 */
			if (err_angle > yaw_calibration_n)			//adjust nr_times
			{			
				nr_times =  err_angle/yaw_calibration_n;
				if (nr_times > 10)
					nr_times = 1;
				//printf("err_angle = %f, yaw_calibration_n = %f\n", 
				//		err_angle, yaw_calibration_n);
				err_angle = intended_heading - avg_heading;	//tells which direction to go
				printf("setpoint %4.3f: avg heading %4.3f: diff: %4.3f, nrtimes: %d\n",
						intended_heading, avg_heading, err_angle, nr_times);
						
						
				anti_clockwise(1.0, nr_times);
			}		
			else						//adjust rotation angle
			{
				float frac_angle = err_angle / yaw_calibration_n;
		
				err_angle = intended_heading - avg_heading;	//tells which direction to go
				printf("setpoint %4.3f: avg heading %4.3f: diff: %4.3f, frac_angle: %4.3f\n",
						intended_heading, avg_heading, err_angle, frac_angle);
				
				anti_clockwise(frac_angle, 1);
			}

		/* Get the heading value 
		 * for the next round of control.
		 * But sleep for a while to filter out 
		 * the noisy sensor reads 
		 * in the transient phase
		 */
		sleep(2);
		avg_data = get_avg_heading();
		avg_heading = avg_data.heading;
		
		/* Adjust reference to account for wind */
		// intended_heading = get_bearing(wptr); 

		err_angle = intended_heading - avg_heading;
		if (err_angle < 0)			
		{
			err_angle = -err_angle;
			clockwise_correction = 0;
		}
		else
			clockwise_correction = 1;
	}        
    printf("avg heading after %f\n", avg_heading);
}

void calibrate_yaw() {
#if FIXED_YAW
	yaw_calibration_n = 25;
	yaw_calibration_p = 25;
#else
	uint8_t cnt_loop = 0;
	nav_gps_heading_t avg_data;
	float heading[7], diff_heading[6], davg_heading = 0;
	avg_data = get_avg_heading();
	heading[cnt_loop] = avg_data.heading;
	
	while (cnt_loop < 3) {
		printf("clock %d\n", cnt_loop+1);
		clockwise(1,1);
		cnt_loop++;
		sleep(2);
		avg_data = get_avg_heading();
		heading[cnt_loop] = avg_data.heading;		
		diff_heading[cnt_loop - 1] = heading[cnt_loop] - 
									 heading[cnt_loop - 1];
		if (diff_heading[cnt_loop - 1] < 0)
			diff_heading[cnt_loop - 1] = -diff_heading[cnt_loop - 1];				
	}
	
	while (cnt_loop < 6) {
		anti_clockwise(1,1);
		cnt_loop++;
		sleep(2);
		avg_data = get_avg_heading();
		heading[cnt_loop] = avg_data.heading;		
		diff_heading[cnt_loop - 1] = heading[cnt_loop] - 
									 heading[cnt_loop - 1];
		if (diff_heading[cnt_loop - 1] < 0)
			diff_heading[cnt_loop - 1] = -diff_heading[cnt_loop - 1];				
	}
	
	printf("Difference heading clockwise: ");
	davg_heading = 0;
	for (cnt_loop = 0; cnt_loop < 3; cnt_loop++) {
		printf("%4.3f ", diff_heading[cnt_loop]);
		davg_heading = davg_heading + diff_heading[cnt_loop];
	}	
	
	yaw_calibration_p = davg_heading / 3;
	 
	
	printf("Difference heading anticlockwise: ");
	davg_heading = 0;
	for (cnt_loop = 3; cnt_loop < 6; cnt_loop++) {
		printf("%4.3f ", diff_heading[cnt_loop]);
		davg_heading = davg_heading + diff_heading[cnt_loop];
	}
	
	yaw_calibration_n = davg_heading / 3; 
#endif		
	printf("yaw_calibration_p: %f\n", yaw_calibration_p);
	printf("yaw_calibration_n: %f\n", yaw_calibration_n);	
}

float get_bearing(uint8_t waypoint_ptr)//return array with dist and bearing here instead maybe
{
	float dest_lat = gps_points[waypoint_ptr].gps_lat*M_PI/180;
	float dest_lon = gps_points[waypoint_ptr].gps_lon*M_PI/180;  
	//printf("r_dest_lat: %f, r_dest_lon: %f, M_Pi: %f\n", dest_lat, dest_lon, M_PI);

	nav_gps_heading_t avg_data;	
	float avg_heading;	
	float avg_lon;
	float avg_lat;
	
	avg_data = get_avg_heading();
	avg_heading = avg_data.heading;
	avg_lon = avg_data.gps_lon;
	avg_lat = avg_data.gps_lat;
   	//printf("start_avg_lat: %f, start_avg_lon: %f\n", avg_lat, avg_lon);
   	//convert to radians
   	avg_lon = (avg_lon*M_PI)/180;
   	avg_lat = (avg_lat*M_PI)/180;
   	//printf("radians_avg_lat: %f, radians_avg_lon: %f\n", avg_lat, avg_lon);

   	float delta_lon = dest_lon-avg_lon;
  	//printf("delta lon: %f\n", delta_lon);
  	float x = cos(avg_lat)*sin(dest_lat) - sin(avg_lat)*cos(dest_lat)*cos(delta_lon);
 	float y = sin(delta_lon)*cos(dest_lat);
 	//printf("x: %f, y: %f\n", x, y);
 	
 	float bearing = atan2(y,x)*180.0/M_PI;
	printf("bearing %4.6f\n", bearing);
 	return bearing;
	
}

float get_distance(float gps_lat, float gps_lon)
{
	float dest_lat = gps_lat*M_PI/180;
	float dest_lon = gps_lon*M_PI/180; 

	//printf("r_dest_lat: %f, r_dest_lon: %f, M_Pi: %f\n", dest_lat, dest_lon, M_PI);

	nav_gps_heading_t avg_data;	
	float avg_heading;
	float avg_lon;
	float avg_lat;
	avg_data = get_avg_heading();
	avg_heading = avg_data.heading;
	avg_lon = avg_data.gps_lon;
	avg_lat = avg_data.gps_lat;
	
	//convert to radians
   	avg_lon = (avg_lon*M_PI)/180;
   	avg_lat = (avg_lat*M_PI)/180;
   	//printf("radians_avg_lat: %f, radians_avg_lon: %f\n", avg_lat, avg_lon);

  	float delta_lon = dest_lon-avg_lon;
   	//printf("start_avg_lat: %f, start_avg_lon: %f\n", avg_lat, avg_lon);
   	float R = 6371*1000;
 	float d = acos(sin(avg_lat)*sin(dest_lat)+ cos(avg_lat)*cos(dest_lat)*cos(delta_lon))*R;
 	printf("distance: %f\n", d);
	
	return d;
}


void clockwise(float r_angle, int ntimes) {	
	float psi_old = psi_val, angle_measure = 0;
	printf("psi_old = %f, psi_val = %f\n", psi_old, psi_val);
	while (ntimes > 0) {
		//rotate_right(r_angle);
		usleep(50000);
		if (psi_val >= psi_old)		
			angle_measure = angle_measure + (psi_val - psi_old) / 1000.0;
		else
			angle_measure = angle_measure + 360 + (psi_val - psi_old) / 1000.0;
			
		psi_old = psi_val;
		ntimes--;
	}	
	
	printf("angle_measure clockwise = %f\n", angle_measure);
}

void clockwise_turn(float r_angle, float setpoint) {	
	float psi_old = psi_val, angle_measure = 0;
	int ntimes = 100;
	while ((angle_measure < setpoint) && (ntimes > 0)) {
		rotate_right(r_angle);
		usleep(50000);
		if (psi_val >= psi_old)		
			angle_measure = angle_measure + (psi_val - psi_old) / 1000.0;
		else
			angle_measure = angle_measure + 360 + (psi_val - psi_old) / 1000.0;
		psi_old = psi_val;
		ntimes--;
	}	
	
	printf("angle_measure clockwise = %f, ntimes = %d\n", 
			angle_measure, 100-ntimes);
}

void anti_clockwise_turn(float l_angle, float setpoint) {
	float psi_old = psi_val, angle_measure = 0;
	int ntimes = 100;
	while ((angle_measure < setpoint) && (ntimes > 0)) {
		rotate_left(l_angle);
		usleep(50000);
		if (psi_old >= psi_val)
			angle_measure = angle_measure + (psi_old - psi_val) / 1000.0;
		else
			angle_measure = angle_measure + 360 + (psi_old - psi_val) / 1000.0;
		psi_old = psi_val;
		ntimes--;
	}
	
	printf("angle_measure counter-clockwise = %f, ntimes = %d\n",
			angle_measure, ntimes);		
}


void anti_clockwise(float l_angle, int ntimes) {
	float psi_old = psi_val, angle_measure = 0;
	while (ntimes > 0) {
		usleep(50000);
		angle_measure = angle_measure + (psi_val - psi_old) / 1000.0;
		psi_old = psi_val;
		ntimes--;
	}
	
	printf("angle_measure counter-clockwise = %f\n", angle_measure);		
}


void go_forward(float r_tilt, int ntimes) {
	//x_distance = 0; 
	lock_on = 1;
	while (ntimes > 0) {
		tilt_forward(r_tilt);
		usleep(50000);					
		ntimes--;
	}
	lock_on = 0;
	usleep(500000);
}

void forward_distance(float r_tilt, float req_distance) {
	int maxiter = 5, ntimes;
	// Set to a random value
	float distance_per_command = 0.07, alpha_coeff = 0.7, prev_dist;
	x_distance = 0;
	
	while ((x_distance < req_distance) && (maxiter > 0)) {
		prev_dist = x_distance;
		ntimes = (2*((req_distance - x_distance) * alpha_coeff
				 / distance_per_command) + 1) / 2;
		if ((ntimes > 30) || (ntimes < 0)) {
			ntimes = 5;
			go_forward(r_tilt, ntimes);
		}
		else
			go_forward(r_tilt, ntimes);
		sleep(1);
		distance_per_command = (x_distance - prev_dist) / ntimes;
		maxiter--;
	}
	printf("done %f m\n", x_distance);
}

void go_backward(float r_tilt, int ntimes) {
	while (ntimes > 0) {
		tilt_forward(r_tilt);
		usleep(50000);
		ntimes--;
	}
}
