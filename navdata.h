#ifndef NAVDATA_H
#define NAVDATA_H
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
#include <errno.h>
#include <stdint.h>
#include <math.h>

#define NAV_PORT 5554
#define LOCAL_NAV_PORT 15554
#define NAV_BUFFER_SIZE 3000
#define GPS_STRUCT_MAX_ELEMENTS 20
#define NAV_DATA_RESOLUTION  20
#define MAX_WAYPOINTS 10

/** Deprecated struct, not used anymore
 */
typedef struct _nav_gps_heading {
    float gps_lat;
    float gps_lon;
    float heading;
    float wind_angle;
    float wind_speed;
} nav_gps_heading_t;

typedef struct _gps_coordinate {
	float gps_lat;
	float gps_lon;
} gps_coordinate_t;

typedef struct _inertial_state {
    float psi_val;
    float vx_val;
    float vy_val;
    float vz_val;
    float x_distance;
} inertial_state_t;

int lock_on;

gps_coordinate_t gps_points[MAX_WAYPOINTS]; 
gps_coordinate_t gps_data[GPS_STRUCT_MAX_ELEMENTS], gps_state;
inertial_state_t inertial_state;

uint8_t wptr, num_waypoints;

float yaw_calibration_p, yaw_calibration_n;
char navdata_buffer[NAV_BUFFER_SIZE];

#define bool int32_t

uint16_t struct_ptr, nav_resolution_ptr;
pthread_mutex_t gps_heading_mutex;

/**
 * @brief Navdata structure sent over the network.
 */
typedef struct matrix33 { 
    float m11, m12, m13;
    float m21, m22, m23;
    float m31, m32, m33;
} matrix33_t;

// 3x1 vector
typedef union vector31 {
    float v[3];
    struct {
        float x;
        float y;
        float z;
    };
} vector31_t;

// 2x1 vector
typedef union vector21 {
    float v[2];
    struct {
        float x;
        float y;
    };
} vector21_t;

// Velocities
typedef struct velocities {
    float x;
    float y;
    float z;
} velocities_t;

// Screen point
typedef struct screen_point {
    int x;
    int y;
} screen_point_t;

typedef struct _navdata_option_t {
  uint16_t  tag;
  uint16_t  size;
  uint8_t   data[];
} navdata_option_t;

// Demo
typedef struct NAVDATA_DEMO {
    unsigned short tag;
    unsigned short size;
    unsigned int   ctrl_state;
    unsigned int   vbat_flying_percentage;
    float          theta;
    float          phi;
    float          psi;
    int            altitude;
    float          vx;
    float          vy;
    float          vz;
    unsigned int   num_frames;                // Don't use
    matrix33_t     detection_camera_rot;      // Don't use
    vector31_t     detection_camera_trans;    // Don't use
    unsigned int   detection_tag_index;       // Don't use
    unsigned int   detection_camera_type;     // Don't use
    matrix33_t     drone_camera_rot;          // Don't use
    vector31_t     drone_camera_trans;        // Don't use
} __attribute__((packed)) nav_demo;

// Timestamp
typedef struct NAVDATA_TIME {
    unsigned short tag;
    unsigned short size;
    unsigned int   time;
} __attribute__((packed)) nav_time;

// Raw measurements
typedef struct NAVDATA_RAW_MEASURES {
    unsigned short tag;
    unsigned short size;
    unsigned short raw_accs[3];         // filtered accelerometers
    short          raw_gyros[3];        // filtered gyrometers
    short          raw_gyros_110[2];    // gyrometers  x/y 110 deg/s
    unsigned int   vbat_raw;            // battery voltage raw (mV)
    unsigned short us_debut_echo;
    unsigned short us_fin_echo;
    unsigned short us_association_echo;
    unsigned short us_distance_echo;
    unsigned short us_courbe_temps;
    unsigned short us_courbe_valeur;
    unsigned short us_courbe_ref;
    unsigned short flag_echo_ini;
    //unsigned short frame_number;
    unsigned short nb_echo;
    unsigned int   sum_echo;
    int            alt_temp_raw;
    short          gradient;
} __attribute__((packed)) raw_measures;

// Physical measurements
typedef struct NAVDATA_PHYS_MEASURES {
    unsigned short tag;
    unsigned short size;
    float          accs_temp;
    unsigned short gyro_temp;
    float          phys_accs[3];
    float          phys_gyros[3];
    unsigned int   alim3V3;         // 3.3 volt alim       [LSB]
    unsigned int   vrefEpson;       // ref volt Epson gyro [LSB]
    unsigned int   vrefIDG;         // ref volt IDG gyro   [LSB]
} __attribute__((packed)) phys_measures;

// Gyros offsets
typedef struct NAVDATA_GYROS_OFFSETS {
    unsigned short tag;
    unsigned short size;
    float          offset_g[3];
} __attribute__((packed)) gyros_offsets;

// Euler angles
typedef struct NAVDATA_EULER_ANGLES {
    unsigned short tag;
    unsigned short size;
    float          theta_a;
    float          phi_a;
} __attribute__((packed)) euler_angles;

// References
typedef struct NAVDATA_REFERENCES {
    unsigned short tag;
    unsigned short size;
    int            ref_theta;
    int            ref_phi;
    int            ref_theta_I;
    int            ref_phi_I;
    int            ref_pitch;
    int            ref_roll;
    int            ref_yaw;
    int            ref_psi;
    float          vx_ref;
    float          vy_ref;
    float          theta_mod;
    float          phi_mod;
    float          k_v_x;
    float          k_v_y;
    unsigned int   k_mode;
    float          ui_time;
    float          ui_theta;
    float          ui_phi;
    float          ui_psi;
    float          ui_psi_accuracy;
    int            ui_seq;
} __attribute__((packed)) references;

// Trims
typedef struct NAVDATA_TRIMS {
    unsigned short tag;
    unsigned short size;
    float          angular_rates_trim_r;
    float          euler_angles_trim_theta;
    float          euler_angles_trim_phi;
} __attribute__((packed)) trims;

// RC references
typedef struct NAVDATA_RC_REFERENCES {
    unsigned short tag;
    unsigned short size;
    int            rc_ref_pitch;
    int            rc_ref_roll;
    int            rc_ref_yaw;
    int            rc_ref_gaz;
    int            rc_ref_ag;
} __attribute__((packed)) rc_references;

// PWM
typedef struct NAVDATA_PWM {
    unsigned short tag;
    unsigned short size;
    unsigned char  motor1;
    unsigned char  motor2;
    unsigned char  motor3;
    unsigned char  motor4;
    unsigned char  sat_motor1;
    unsigned char  sat_motor2;
    unsigned char  sat_motor3;
    unsigned char  sat_motor4;
    float          gaz_feed_forward;
    float          gaz_altitude;
    float          altitude_integral;
    float          vz_ref;
    int            u_pitch;
    int            u_roll;
    int            u_yaw;
    float          yaw_u_I;
    int            u_pitch_planif;
    int            u_roll_planif;
    int            u_yaw_planif;
    float          u_gaz_planif;
    unsigned short current_motor1;
    unsigned short current_motor2;
    unsigned short current_motor3;
    unsigned short current_motor4;
    float          altitude_prop;
    float          altitude_der;
} __attribute__((packed)) pwm;

// Altitude
typedef struct NAVDATA_ALTITUDE {
    unsigned short tag;
    unsigned short size;
    int            altitude_vision;
    float          altitude_vz;
    int            altitude_ref;
    int            altitude_raw;
    float          obs_accZ;
    float          obs_alt;
    vector31_t     obs_x;
    unsigned int   obs_state;
    vector21_t     est_vb;
    unsigned int   est_state;
} __attribute__((packed)) altitude;

// Vision (raw)
typedef struct NAVDATA_VISION_RAW {
    unsigned short tag;
    unsigned short size;
    float          vision_tx_raw;
    float          vision_ty_raw;
    float          vision_tz_raw;
} __attribute__((packed)) vision_raw;

// Vision (offset?)
typedef struct NAVDATA_VISION_OF {
    unsigned short tag;
    unsigned short size;
    float          of_dx[5];
    float          of_dy[5];
} __attribute__((packed)) vision_of;

// Vision
typedef struct NAVDATA_VISION {
    unsigned short tag;
    unsigned short size;
    unsigned int   vision_state;
    int            vision_misc;
    float          vision_phi_trim;
    float          vision_phi_ref_prop;
    float          vision_theta_trim;
    float          vision_theta_ref_prop;
    int            new_raw_picture;
    float          theta_capture;
    float          phi_capture;
    float          psi_capture;
    int            altitude_capture;
    unsigned int   time_capture;    // time in TSECDEC format (see config.h)
    velocities_t   body_v;
    float          delta_phi;
    float          delta_theta;
    float          delta_psi;
    unsigned int   gold_defined;
    unsigned int   gold_reset;
    float          gold_x;
    float          gold_y;
} __attribute__((packed)) vision;

// Vision performances
typedef struct NAVDATA_VISION_PERF {
    unsigned short tag;
    unsigned short size;
    float          time_szo;
    float          time_corners;
    float          time_compute;
    float          time_tracking;
    float          time_trans;
    float          time_update;
    float          time_custom[20];
} __attribute__((packed)) vision_perf;

// Trackers
typedef struct NAVDATA_TRACKERS_SEND {
    unsigned short tag;
    unsigned short size;
    int            locked[30];
    screen_point_t point[30];
} __attribute__((packed)) trackers_send;

// Vision detection
typedef struct NAVDATA_VISION_DETECT {
    unsigned short tag;
    unsigned short size;
    unsigned int   nb_detected;
    unsigned int   type[4];
    unsigned int   xc[4];
    unsigned int   yc[4];
    unsigned int   width[4];
    unsigned int   height[4];
    unsigned int   dist[4];
    float          orientation_angle[4];
    matrix33_t     rotation[4];
    vector31_t     translation[4];
    unsigned int   camera_source[4];
} __attribute__((packed)) vision_detect;

// Watchdog
typedef struct NAVDATA_WATCHDOG {
    unsigned short tag;
    unsigned short size;
    int            watchdog;
} __attribute__((packed)) watchdog;

// ADC data
typedef struct NAVDATA_ADC_DATA_FRAME {
    unsigned short tag;
    unsigned short size;
    unsigned int   version;
    unsigned char  data_frame[32];
} __attribute__((packed)) adc_data_frame;

// Video stream
typedef struct NAVDATA_VIDEO_STREAM {
    unsigned short tag;
    unsigned short size;
    unsigned char  quant;               // quantizer reference used to encode frame [1:31]
    unsigned int   frame_size;          // frame size (bytes)
    unsigned int   frame_number;        // frame index
    unsigned int   atcmd_ref_seq;       // atmcd ref sequence number
    unsigned int   atcmd_mean_ref_gap;  // mean time between two consecutive atcmd_ref (ms)
    float          atcmd_var_ref_gap;
    unsigned int   atcmd_ref_quality;   // estimator of atcmd link quality

    // drone2
    unsigned int   out_bitrate;         // measured out throughput from the video tcp socket
    unsigned int   desired_bitrate;     // last frame size generated by the video encoder
    int            data1;
    int            data2;
    int            data3;
    int            data4;
    int            data5;
    unsigned int   tcp_queue_level;
    unsigned int   fifo_queue_level;
} __attribute__((packed)) video_stream;

// Games
typedef struct NAVDATA_GAMES {
    unsigned short tag;
    unsigned short size;
    unsigned int   double_tap_counter;
    unsigned int   finish_line_counter;
} __attribute__((packed)) games;

// Preassure (raw)
typedef struct NAVDATA_PRESSURE_RAW {
    unsigned short tag;
    unsigned short size;
    unsigned int   up;
    unsigned short ut;
    unsigned int   temperature_meas;
    unsigned int   pression_meas;
} __attribute__((packed)) pressure_raw;

// Magneto
typedef struct NAVDATA_MAGNETO {
    unsigned short tag;
    unsigned short size;
    short          mx;
    short          my;
    short          mz;
    vector31_t     magneto_raw;             // magneto in the body frame, in mG
    vector31_t     magneto_rectified;
    vector31_t     magneto_offset;
    float          heading_unwrapped;
    float          heading_gyro_unwrapped;
    float          heading_fusion_unwrapped;
    char           magneto_calibration_ok;
    unsigned int   magneto_state;
    float          magneto_radius;
    float          error_mean;
    float          error_var;
    float          tmp1, tmp2;              // dummy ?
} __attribute__((packed)) magneto;

// Wind
typedef struct NAVDATA_WIND {
    unsigned short tag;
    unsigned short size;
    float          wind_speed;              // estimated wind speed [m/s]
    float          wind_angle;              // estimated wind direction in North-East frame [rad] e.g. if wind_angle is pi/4, wind is from South-West to North-East
    float          wind_compensation_theta;
    float          wind_compensation_phi;
    float          state_x1;
    float          state_x2;
    float          state_x3;
    float          state_x4;
    float          state_x5;
    float          state_x6;
    float          magneto_debug1;
    float          magneto_debug2;
    float          magneto_debug3;
} __attribute__((packed)) wind;

// Kalman filter
typedef struct NAVDATA_KALMAN_PRESSURE {
    unsigned short tag;
    unsigned short size;
    float          offset_pressure;
    float          est_z;
    float          est_zdot;
    float          est_bias_PWM;
    float          est_biais_pression;
    float          offset_US;
    float          prediction_US;
    float          cov_alt;
    float          cov_PWM;
    float          cov_vitesse;
    bool           bool_effet_sol;
    float          somme_inno;
    bool           flag_rejet_US;
    float          u_multisinus;
    float          gaz_altitude;
    bool           flag_multisinus;
    bool           flag_multisinus_debut;
} __attribute__((packed)) kalman_pressure;

// HD video stream
typedef struct NAVDATA_HDVIDEO_STREAM {
    unsigned short tag;
    unsigned short size;
    unsigned int   hdvideo_state;
    unsigned int   storage_fifo_nb_packets;
    unsigned int   storage_fifo_size;
    unsigned int   usbkey_size;           // USB key in kbytes - 0 if no key present
    unsigned int   usbkey_freespace;      // USB key free space in kbytes - 0 if no key present
    unsigned int   frame_number;          // 'frame_number' PaVE field of the frame starting to be encoded for the HD stream
    unsigned int   usbkey_remaining_time; // time in seconds
} __attribute__((packed)) hdvideo_stream;

// WiFi
typedef struct NAVDATA_WIFI {
    unsigned short tag;
    unsigned short size;
    unsigned int   link_quality;
} __attribute__((packed)) wifi;

// Zimmu 3000
typedef struct NAVDATA_ZIMMU_3000 {
    unsigned short tag;
    unsigned short size;
    int            vzimmuLSB;
    float          vzfind;
} __attribute__((packed)) zimmu_3000;

// GPS (for AR.Drone 2.4.1, or later)
// From https://github.com/paparazzi/paparazzi/blob/master/sw/airborne/boards/ardrone/at_com.h
typedef struct NAVDATA_GPS {
    unsigned short tag;                  /*!< Navdata block ('option') identifier */
    unsigned short size;                 /*!< set this to the size of this structure */
    double         lat;                  /*!< Latitude */
    double         lon;                  /*!< Longitude */
    double         elevation;            /*!< Elevation */
    double         hdop;                 /*!< hdop */
    int            data_available;       /*!< When there is data available */
    unsigned char  unk_0[8];
    double         lat0;                 /*!< Latitude ??? */
    double         lon0;                 /*!< Longitude ??? */
    double         lat_fuse;             /*!< Latitude fused */
    double         lon_fuse;             /*!< Longitude fused */
    unsigned int   gps_state;            /*!< State of the GPS, still need to figure out */
    unsigned char  unk_1[40];
    double         vdop;                 /*!< vdop */
    double         pdop;                 /*!< pdop */
    float          speed;                /*!< speed */
    unsigned int   last_frame_timestamp; /*!< Timestamp from the last frame */
    float          degree;               /*!< Degree */
    float          degree_mag;           /*!< Degree of the magnetic */
    unsigned char  unk_2[16];
    struct {
        unsigned char sat;
        unsigned char cn0;
    } channels[12];
    int             gps_plugged;         /*!< When the gps is plugged */
    unsigned char   unk_3[108];
    double          gps_time;            /*!< The gps time of week */
    unsigned short  week;                /*!< The gps week */
    unsigned char   gps_fix;             /*!< The gps fix */
    unsigned char   num_sattelites;      /*!< Number of sattelites */
    unsigned char   unk_4[24];
    double          ned_vel_c0;          /*!< NED velocity */
    double          ned_vel_c1;          /*!< NED velocity */
    double          ned_vel_c2;          /*!< NED velocity */
    double          pos_accur_c0;        /*!< Position accuracy */
    double          pos_accur_c1;        /*!< Position accuracy */
    double          pos_accur_c2;        /*!< Position accuracy */
    float           speed_acur;          /*!< Speed accuracy */
    float           time_acur;           /*!< Time accuracy */
    unsigned char   unk_5[72];
    float           temprature;
    float           pressure;
} __attribute__((packed)) gps;

// Check sum
typedef struct NAVDATA_CKS {
    unsigned short tag;
    unsigned short size;
    unsigned int   cks;
} __attribute__((packed)) cks;

typedef struct _navdata_t {
    // Header
    unsigned int header;
    unsigned int ardrone_state;
    unsigned int sequence;
    unsigned int vision_defined;    

    navdata_option_t options[1];
} __attribute__((packed)) navdata_t;


// Navdata tags
enum ARDRONE_NAVDATA_TAG {
    ARDRONE_NAVDATA_DEMO_TAG            =  0,
    ARDRONE_NAVDATA_TIME_TAG            =  1,
    ARDRONE_NAVDATA_RAW_MEASURES_TAG    =  2,
    ARDRONE_NAVDATA_PHYS_MEASURES_TAG   =  3,
    ARDRONE_NAVDATA_GYROS_OFFSETS_TAG   =  4,
    ARDRONE_NAVDATA_EULER_ANGLES_TAG    =  5,
    ARDRONE_NAVDATA_REFERENCES_TAG      =  6,
    ARDRONE_NAVDATA_TRIMS_TAG           =  7,
    ARDRONE_NAVDATA_RC_REFERENCES_TAG   =  8,
    ARDRONE_NAVDATA_PWM_TAG             =  9,
    ARDRONE_NAVDATA_ALTITUDE_TAG        = 10,
    ARDRONE_NAVDATA_VISION_RAW_TAG      = 11,
    ARDRONE_NAVDATA_VISION_OF_TAG       = 12,
    ARDRONE_NAVDATA_VISION_TAG          = 13,
    ARDRONE_NAVDATA_VISION_PERF_TAG     = 14,
    ARDRONE_NAVDATA_TRACKERS_SEND_TAG   = 15,
    ARDRONE_NAVDATA_VISION_DETECT_TAG   = 16,
    ARDRONE_NAVDATA_WATCHDOG_TAG        = 17,
    ARDRONE_NAVDATA_ADC_DATA_FRAME_TAG  = 18,
    ARDRONE_NAVDATA_VIDEO_STREAM_TAG    = 19,
    ARDRONE_NAVDATA_GAME_TAG            = 20,       // AR.Drone 1.7.4
    ARDRONE_NAVDATA_PRESSURE_RAW_TAG    = 21,       // AR.Drone 2.0
    ARDRONE_NAVDATA_MAGNETO_TAG         = 22,       // AR.Drone 2.0
    ARDRONE_NAVDATA_WIND_TAG            = 23,       // AR.Drone 2.0
    ARDRONE_NAVDATA_KALMAN_PRESSURE_TAG = 24,       // AR.Drone 2.0
    ARDRONE_NAVDATA_HDVIDEO_STREAM_TAG  = 25,       // AR.Drone 2.0
    ARDRONE_NAVDATA_WIFI_TAG            = 26,       // AR.Drone 2.0
    ARDRONE_NAVDATA_GPS_TAG             = 27,       // AR.Drone 2.4.1
    ARDRONE_NAVDATA_CKS_TAG             = 0xFFFF
};


int navsock;
uint32_t navdata_tag_mask;
struct sockaddr_in navsock_info, navdata_info, navfrom;
#endif

/**
 * @brief Navdata pointers
 */
nav_demo *demo_ptr;
altitude *altitude_ptr;
gps      *gps_ptr;


/** Initialize Navigation port */
void nav_port_init();
void nav_read();


/** Printing navigation data */
void enable_navdata_print(uint8_t tagp);
void disable_navdata_print(uint8_t tagp);
void print_nav_data(uint16_t tagp, uint16_t sizep, uint8_t *n);

/** Fetch average GPS and heading values */
void get_avg_heading();

/** A simple controller that orients the drone heading in the 
 * direction 'ref' degrees from North. Positive values indicate 
 * clockwise direction, while negative values denote anti-clockwise 
 * direction
 */
void set_drone_heading(float ref);

/** Compute GPS bearing and distance */
float get_bearing(uint8_t waypoint_ptr);

/** Compute distance between current location and the location
 *  given by (gps_lat, gps_lon)
 */
float get_distance(float gps_lat, float gps_lon);

/** Point to point navigation */
void navigate_next(uint8_t waypoint_ptr);

/** Deprecated functions
 */
void clockwise(float r_angle, int ntimes);
void anti_clockwise(float l_angle, int ntimes);
void go_forward(float r_tilt, int ntimes);
void go_backward(float r_tilt, int ntimes);

/** P-controller for heading, and P-D controller for pitching forward
 */
void clockwise_turn(float r_angle, float setpoint);
void anti_clockwise_turn(float l_angle, float setpoint);
void forward_distance(float r_tilt, float req_distance);
