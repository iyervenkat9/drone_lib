#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>


#define PORT_NO 6000


void data_server_start() {
	int sockfd;
	struct sockaddr_in sock_info, rx_info;
	char rx_buffer[200];

	sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	memset(&sock_info, 0, sizeof(sock_info));


	struct hostent *hp = gethostbyname("192.168.1.4");
	char localhostaddr[20];
	memcpy(&sock_info.sin_addr, (char *) hp->h_addr, hp->h_length);

	sock_info.sin_family = AF_INET;
	sock_info.sin_port = htons(PORT_NO);

	bind(sockfd, (struct sockaddr *) &sock_info, sizeof(sock_info));

	int num_bytes, rx_info_len = sizeof(rx_info);

	while (1) {

		num_bytes = recvfrom(sockfd,rx_buffer, 200, 0, (struct sockaddr *) &rx_info, &rx_info_len);

		if (num_bytes == -1)
			printf("recvfrom failed\n");
		else {
			printf("recvfrom with %d bytes\n", num_bytes);	
			rx_buffer[num_bytes] = 0;
			printf("string is %s\n", rx_buffer);
			int ptr = 0; 
			printf("Hexadecimal: ");
			for (ptr = 0; ptr < num_bytes; ptr++) 
				printf("%02x", rx_buffer[ptr]);
			printf("\n");
		}
	}

	close(sockfd);
}

void data_client_start() {
	
}
