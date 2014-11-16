#include "actuation.h"
#include <pthread.h>

void register_actuation(int socket_descriptor, struct sockaddr_in *socket_info) {
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&sock_info, 0, sizeof(sock_info));

    struct hostent *hp = gethostbyname("192.168.1.3");
    char localhostaddr[20];
    memcpy(&sock_info.sin_addr, (char *) hp->h_addr, hp->h_length);

    sock_info.sin_family = AF_INET;
    sock_info.sin_port = htons(PORT_NO);    
    at_seq = 1;
    pthread_mutex_init(&at_mutex, NULL);
}

void flat_trim() {
    char ft_str[40];
    int num_bytes;
    
    strcpy(ft_str, "AT*FTRIM=");
    memset(at_seq_str, 0x00, 6);
    snprintf(at_seq_str, 6, "%d", at_seq++);    
    strcat(ft_str, at_seq_str);
    strcat(ft_str, "\r");

    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,ft_str, strlen(ft_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);    
}

void set_outdoor(uint8_t flag) {
    char r_str1[80];
    int num_bytes;
    
    if (flag)
        sprintf(r_str1, 
                "AT*CONFIG=%d,\"control:outdoor\",\"TRUE\"\r", 
                at_seq++);
    else
        sprintf(r_str1, 
                "AT*CONFIG=%d,\"control:outdoor\",\"FALSE\"\r", 
                at_seq++);
        
    printf("flag %d, outdoor %s\n", flag, r_str1);
    
    pthread_mutex_lock(&at_mutex);    
    num_bytes = sendto(sockfd,r_str1, strlen(r_str1), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);
}


void set_control_yaw(float yaw_radians) {
    char r_str1[80];
    char r_str2[80];
    int num_bytes;
    
    sprintf(r_str1, 
            "AT*CONFIG=%d,\"control:outdoor\",\"TRUE\"\r", 
            at_seq++);
    
    pthread_mutex_lock(&at_mutex);    
    num_bytes = sendto(sockfd,r_str1, strlen(r_str1), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);
}

void test_leds() {
    char test_str[] = "AT*LED=1,0,1056964608,10\r";
    int num_bytes;
    
    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,test_str, sizeof(test_str)-1, 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);
}

void vertical_down(float speed) {
    char r_str[80];
    char *r_str_2 = compute_IEEE(-speed);
    int num_bytes;
    
    sprintf(r_str, 
        "AT*PCMD_MAG=%d,1,0,0,%s,0,0,0\rAT*REF=%d,290718208\r",
        at_seq++, r_str_2, at_seq++);
    
    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,r_str, strlen(r_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);    
}


void vertical_up(float speed) {
    char r_str[80];
    char *r_str_2 = compute_IEEE(speed);
    int num_bytes;
    
    sprintf(r_str, 
            "AT*PCMD_MAG=%d,1,0,0,%s,0,0,0\rAT*REF=%d,290718208\r",
            at_seq++, r_str_2, at_seq++);

    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,r_str, strlen(r_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);    
}



void rotate_left(float l_angle) {
    char r_str[80];
    char *r_str_2 = compute_IEEE(-l_angle);
    int num_bytes;
    
    sprintf(r_str, 
            "AT*PCMD_MAG=%d,1,0,0,0,%s,0,0\rAT*REF=%d,290718208\r",
             at_seq++, r_str_2, at_seq++);
             
    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,r_str, strlen(r_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);    
}

void rotate_right(float r_angle) {
    char r_str[80];
    char *r_str_2 = compute_IEEE(r_angle);
    int num_bytes;
    
    sprintf(r_str, 
            "AT*PCMD_MAG=%d,1,0,0,0,%s,0,0\rAT*REF=%d,290718208\r",
             at_seq++, r_str_2, at_seq++);
    
    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,r_str, strlen(r_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);
}

void tilt_left(float r_tilt) {
    char r_str[80];
    char *r_str_2 = compute_IEEE(-r_tilt);
    int num_bytes;
    
    sprintf(r_str, 
            "AT*PCMD_MAG=%d,1,%s,0,0,0,0,0\rAT*REF=%d,290718208\r",
             at_seq++, r_str_2, at_seq++);

    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,r_str, strlen(r_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);
}

void tilt_right(float r_tilt) {
    char r_str[80];
    char *r_str_2 = compute_IEEE(r_tilt);
    int num_bytes;
    
    sprintf(r_str, 
            "AT*PCMD_MAG=%d,1,%s,0,0,0,0,0\rAT*REF=%d,290718208\r",
             at_seq++, r_str_2, at_seq++);

    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,r_str, strlen(r_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);
}

void tilt_forward(float r_tilt) {
    char r_str[80];
    char *r_str_2 = compute_IEEE(-r_tilt);
    int num_bytes;
    
    sprintf(r_str, 
            "AT*PCMD_MAG=%d,1,0,%s,0,0,0,0\rAT*REF=%d,290718208\r",
            at_seq++, r_str_2, at_seq++);
             
    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,r_str, strlen(r_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);
}


void tilt_backward(float r_tilt) {
    char r_str[80];
    char *r_str_2 = compute_IEEE(r_tilt);
    int num_bytes;
    
    sprintf(r_str, 
            "AT*PCMD_MAG=%d,1,0,%s,0,0,0,0\rAT*REF=%d,290718208\r",
            at_seq++, r_str_2, at_seq++);

    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,r_str, strlen(r_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);
}


void takeoff() {
    char takeoff_str[40];
    int num_bytes;
    
    sprintf(takeoff_str, "AT*REF=%d,290718208\r", at_seq++);
    
    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,takeoff_str, strlen(takeoff_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);
}

void land() {
    char land_str[40];
    int num_bytes;
    
    sprintf(land_str, "AT*REF=%d,290717696\r", at_seq++);

    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,land_str, strlen(land_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);
}

void calibrate_magneto() {
    char calib_str[40];
    int num_bytes;
    
    sprintf(calib_str, "AT*CALIB=%d,0\r", at_seq++);

    pthread_mutex_lock(&at_mutex);
    num_bytes = sendto(sockfd,calib_str, strlen(calib_str), 0, 
        (struct sockaddr *) &sock_info, sizeof(sock_info));
    pthread_mutex_unlock(&at_mutex);

}

void close_actuation() {
    close(sockfd);
}

void set_navdata_options(char demo) {
    char nav_str[60];
    int num_bytes;

    pthread_mutex_lock(&at_mutex);
    memset(nav_str, 0x00, sizeof(nav_str));
    if (demo)		
        sprintf(nav_str,
                "AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r",
                at_seq++);	
	else
		sprintf(nav_str, 
                "AT*CONFIG=%d,\"general:navdata_demo\",\"FALSE\"\r",
                at_seq++);

    num_bytes = sendto(sockfd, nav_str, strlen(nav_str), 0, 
                (struct sockaddr *) &sock_info, sizeof(sock_info));
	
    /** Enable only the GPS TAG from the Navdata, 
     *  along with the demo			   
     */
    if (!demo) {
        memset(nav_str, 0x00, sizeof(nav_str));
        sprintf(nav_str, 
                "AT*CONFIG=%d,\"general:navdata_options\",\"134217729\"\r",
                at_seq++);
        num_bytes = sendto(sockfd, nav_str, strlen(nav_str), 0, 
                    (struct sockaddr *) &sock_info, sizeof(sock_info));
    }


    pthread_mutex_unlock(&at_mutex);
}


void roll_right(float r_tilt, int ntimes) {
    while (ntimes > 0) {
        tilt_right(r_tilt);
        usleep(60000);
        ntimes--;
    }
}
void roll_left(float r_tilt, int ntimes) {
    printf("roll left %4.3f\n", r_tilt);
    while (ntimes > 0) {
        tilt_left(r_tilt);
        usleep(60000);
        ntimes--;
    }
}

void move_up(float speed, int ntimes) {
    while (ntimes > 0) {
        vertical_up(speed);
        usleep(60000);
        ntimes--;
    }
}

void move_down(float speed, int ntimes) {
    while (ntimes > 0) {
        vertical_down(speed);
        usleep(60000);
        ntimes--;
    }
}


