#ifndef _SERIAL_H
#define _SERIAL_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <sys/time.h>

#define SERIAL_BUFFER_SIZE 30
char serial_buffer[SERIAL_BUFFER_SIZE];
char serial_str[SERIAL_BUFFER_SIZE];
int serial_fd;
struct termios options;

char log_string[90];

typedef struct _print_stats {
    char id[2];
    uint8_t len;
    uint16_t avg_rssi;
    uint32_t var_rssi;
    uint16_t avg_lqi;
    uint32_t var_lqi;
    uint8_t prr;
} __attribute__((packed)) print_stats_t;

#define NEWLINE	0x0a
#define ESC_CHAR	0x1b
#endif

void serial_init();
void serial_read();
void handle_serial_str();
