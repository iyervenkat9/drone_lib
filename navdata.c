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
                demo_ptr = (nav_demo *) n;
                inertial_state.psi_val = demo_ptr->psi;                
                inertial_state.vx_val = demo_ptr->vx / 1000.0;
                inertial_state.vy_val = demo_ptr->vy / 1000.0;
                inertial_state.vz_val = demo_ptr->vz / 1000.0;
                inertial_state.x_distance = inertial_state.x_distance 
                                            + inertial_state.vx_val
                                            * 0.005;
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
                altitude_ptr = (altitude *) n;        
                
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
        break;
        case ARDRONE_NAVDATA_WIND_TAG:
       break;
        case ARDRONE_NAVDATA_KALMAN_PRESSURE_TAG:
        break;
        case ARDRONE_NAVDATA_HDVIDEO_STREAM_TAG:
        break;
        case ARDRONE_NAVDATA_WIFI_TAG:
        break;  
        case ARDRONE_NAVDATA_GPS_TAG:
            gps_ptr = (gps *) n;
            //printf("gps lat: %f, gps lon: %f, gps elevation %f\n", gps_data->lat, gps_data->lon, gps_data->elevation);
            gps_data[struct_ptr].gps_lat = gps_ptr->lat;
            gps_data[struct_ptr].gps_lon = gps_ptr->lon;
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
       }
    }
}

void update_gps_state() {
    nav_gps_heading_t avg_gps_hd;

    gps_state.gps_lon = 0;
    gps_state.gps_lat = 0;
    uint8_t i;

    for (i = 0; i < GPS_STRUCT_MAX_ELEMENTS; i++) {
        gps_state.gps_lon = gps_state.gps_lon + gps_data[i].gps_lon;
        gps_state.gps_lat = gps_state.gps_lat + gps_data[i].gps_lat;
    }
    gps_state.gps_lon = gps_state.gps_lon*1.0 / GPS_STRUCT_MAX_ELEMENTS;
    gps_state.gps_lat = gps_state.gps_lat*1.0 / GPS_STRUCT_MAX_ELEMENTS;
}


void navigate_next(uint8_t waypoint_ptr) {
    int count = 0;
    float dist = 1000.0, gps_bearing;
    wptr = waypoint_ptr;
    dist = get_distance(gps_points[waypoint_ptr].gps_lat, 
                        gps_points[waypoint_ptr].gps_lon);
    printf("Initial distance = %f\n", dist);
    
    while (dist > 5.0 && count < 7) {
        count++;        
        gps_bearing = get_bearing(waypoint_ptr);        
        set_drone_heading(gps_bearing);
        
        forward_distance(0.3, dist);
        
        update_gps_state();
        dist = get_distance(gps_points[waypoint_ptr].gps_lat,
                            gps_points[waypoint_ptr].gps_lon);
    }
}

void set_drone_heading(float bearing_angle) {
    uint8_t case_turn;
    float angle_tilt = 0.5;
    
    if ((inertial_state.psi_val > 0) && (bearing_angle > 0))
        case_turn = 0;
    else if ((inertial_state.psi_val <= 0) && (bearing_angle > 0))
        case_turn = 1;
    else if ((inertial_state.psi_val > 0) && (bearing_angle <= 0))
        case_turn = 2;
    else 
        case_turn = 3;
        
    switch (case_turn) {
        case 0:
                if (inertial_state.psi_val < bearing_angle)
                    clockwise_turn(angle_tilt, 
                        bearing_angle - inertial_state.psi_val);
                else
                    anti_clockwise_turn(angle_tilt, 
                        inertial_state.psi_val - bearing_angle);                                  
        break;
        
        case 1:
                if (bearing_angle - inertial_state.psi_val < 180)
                    clockwise_turn(angle_tilt, 
                        bearing_angle - inertial_state.psi_val);
                else
                    anti_clockwise_turn(angle_tilt, 360 +
                        inertial_state.psi_val - bearing_angle);                                  
        break;
        
        case 2:
                if (bearing_angle + inertial_state.psi_val < 180)
                    anti_clockwise_turn(angle_tilt, 
                        bearing_angle - inertial_state.psi_val);
                else
                    clockwise_turn(angle_tilt, 360 -
                        inertial_state.psi_val - bearing_angle);                                  
        break;
        
        case 3:
                 if (inertial_state.psi_val < bearing_angle)
                    clockwise_turn(angle_tilt, 
                        bearing_angle - inertial_state.psi_val);
                else
                    anti_clockwise_turn(angle_tilt, 
                        inertial_state.psi_val - bearing_angle);
        break;
        
        default:
                    printf("Bad value %d\n", case_turn);
    }
}

float get_bearing(uint8_t waypoint_ptr)
{
    float dest_lat, dest_lon;
    float delta_lon, 
          work_x, work_y, 
          avg_lat, avg_lon;
    
    dest_lat = gps_points[waypoint_ptr].gps_lat * M_PI / 180,
    dest_lon = gps_points[waypoint_ptr].gps_lon * M_PI / 180;  

    update_gps_state();
    avg_lat = gps_state.gps_lat * M_PI / 180;
    avg_lon = gps_state.gps_lon * M_PI / 180;

       delta_lon = dest_lon - avg_lon;
      work_x = cos(avg_lat) * sin(dest_lat) - 
             sin(avg_lat) * cos(dest_lat) * cos(delta_lon);
     work_y = sin(delta_lon) * cos(dest_lat);
     
     return atan2(work_y, work_x) * 180.0 / M_PI;
}

float get_distance(float gps_lat, float gps_lon)
{
    float dest_lat = gps_lat*M_PI/180,
          dest_lon = gps_lon*M_PI/180,
          avg_lat, avg_lon;
          
    /** R - Earth's radius in Km
     */
    float R = 6371*1000, delta_lon;

    update_gps_state();
    avg_lat = gps_state.gps_lat * M_PI / 180;
    avg_lon = gps_state.gps_lon * M_PI / 180;
    
    
      delta_lon = dest_lon-avg_lon;
       return  acos( sin(avg_lat) * sin(dest_lat) + 
                  cos(avg_lat) * cos(dest_lat) * cos(delta_lon) ) * R;
}

void clockwise_turn(float r_angle, float setpoint) {    
    float psi_old = inertial_state.psi_val;
    float angle_measure = 0;
    int ntimes = 100;

    while ((angle_measure < setpoint) && (ntimes > 0)) {
        rotate_right(r_angle);
        usleep(50000);
        if (inertial_state.psi_val >= psi_old)        
            angle_measure = angle_measure + 
                            (inertial_state.psi_val - psi_old) / 1000.0;
        else
            angle_measure = angle_measure + 360 + 
                            (inertial_state.psi_val - psi_old) / 1000.0;
        psi_old = inertial_state.psi_val;
        ntimes--;
    }    
    
    printf("angle_measure clockwise = %f, ntimes = %d\n", 
            angle_measure, 100-ntimes);
}

void anti_clockwise_turn(float l_angle, float setpoint) {
    float psi_old = inertial_state.psi_val;
    float angle_measure = 0;
    int ntimes = 100;
    
    while ((angle_measure < setpoint) && (ntimes > 0)) {
        rotate_left(l_angle);
        usleep(50000);
        if (psi_old >= inertial_state.psi_val)
            angle_measure = angle_measure + 
                            (psi_old - inertial_state.psi_val) / 1000.0;
        else
            angle_measure = angle_measure + 360 +
                            (psi_old - inertial_state.psi_val) / 1000.0;
        psi_old = inertial_state.psi_val;
        ntimes--;
    }
    
    printf("angle_measure counter-clockwise = %f, ntimes = %d\n",
            angle_measure, ntimes);        
}


void forward_distance(float r_tilt, float req_distance) {
    int n_iter = 0, ntimes;
    // Set to a random value
    float   derr = 0, 
            prev_err = 0, 
            err = 0, 
            tilt_angle = 0;
          
    // Reset the inertial state
    inertial_state.x_distance = 0;
    
    while ( (inertial_state.x_distance < req_distance) && 
            (n_iter <= 50)) {
                
        err = ( req_distance - inertial_state.x_distance ) / 
                req_distance;        
        if (n_iter > 0)
            derr = err - prev_err;
        
        // A simple PD controller, P coeff of 1 and a D coeff of 2.5
        tilt_angle = 0.2*err + 2.5*derr;
        
        if ((tilt_angle < 0) || (tilt_angle > 1.0)) {
            tilt_angle = 0.1;
            break;
        }
        else                
            tilt_forward(tilt_angle);
        usleep(50000);
        prev_err = err;
        n_iter++;
    }
    sleep(2);
    printf("done %f m, n_iter %d\n", 
            inertial_state.x_distance, n_iter);
}


/**
 * Deprecated, not used anymore
 */
void clockwise(float r_angle, int ntimes) {    
    float psi_old = inertial_state.psi_val,
          angle_measure = 0;
    
    while (ntimes > 0) {
        rotate_right(r_angle);
        usleep(50000);
        if (inertial_state.psi_val >= psi_old)        
            angle_measure = angle_measure +
                            (inertial_state.psi_val - psi_old) / 1000.0;
        else
            angle_measure = angle_measure + 360 + 
                            (inertial_state.psi_val - psi_old) / 1000.0;
            
        psi_old = inertial_state.psi_val;
        ntimes--;
    }    
    
    printf("angle_measure clockwise = %f\n", angle_measure);
}


/**
 * Deprecated, not used anymore
 */
void anti_clockwise(float l_angle, int ntimes) {
    float psi_old = inertial_state.psi_val, 
          angle_measure = 0;
    
    while (ntimes > 0) {
        usleep(50000);
        angle_measure = angle_measure + 
                        (inertial_state.psi_val - psi_old) / 1000.0;
        psi_old = inertial_state.psi_val;
        ntimes--;
    }
    
    printf("angle_measure counter-clockwise = %f\n", angle_measure);        
}

/**
 * Deprecated, not used anymore
 */
void go_forward(float r_tilt, int ntimes) {
    while (ntimes > 0) {
        tilt_forward(r_tilt);
        usleep(50000);                    
        ntimes--;
    }
    usleep(500000);
}

/**
 * Deprecated, not used anymore
 */
void go_backward(float r_tilt, int ntimes) {
    while (ntimes > 0) {
        tilt_forward(r_tilt);
        usleep(50000);
        ntimes--;
    }
}
