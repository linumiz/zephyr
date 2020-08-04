#include <string.h>
#include <zephyr.h>
#include <errno.h>
#include <stdio.h>

#include <net/socket.h>
#include <net/tls_credentials.h>

#include <net/net_mgmt.h>
#include <net/net_event.h>
#include <net/net_conn_mgr.h>

#include <stdlib.h> 
#include <string.h> 
#define MAX 80 
#define PORT 8080 
#define SA struct sockaddr 

void func(int sockfd) 
{ 
	char buff[MAX] = {0}; 
	int n; 
	for (;;) { 
		memset(buff, 0, sizeof(buff)); 
		printk("Enter the string : "); 
		n = 0; 
		strncpy(buff, "hello", 6);
		send(sockfd, buff, sizeof(buff), 0); 
		memset(buff, 0, sizeof(buff)); 
		recv(sockfd, buff, sizeof(buff), 0); 
		printk("From Server : %s", buff); 
		if ((strncmp(buff, "exit", 4)) == 0) { 
			printk("Client Exit...\n"); 
			break; 
		} 
	} 
} 

int main() 
{ 
	int rc;
	int sockfd, connfd; 
	struct sockaddr_in servaddr, cli; 

	app_dhcpv4_startup();
	// socket create and varification 
	sockfd = socket(AF_INET, SOCK_STREAM, 0); 
	if (sockfd == -1) { 
		printk("socket creation failed...\n"); 
		exit(0); 
	} 
	else
		printk("Socket successfully created..\n"); 
	memset(&servaddr, 0, sizeof(servaddr)); 

	// assign IP, PORT 
	servaddr.sin_family = AF_INET; 
	servaddr.sin_port = htons(PORT); 
	inet_pton(AF_INET, "192.168.178.66", &servaddr.sin_addr);

	// connect the client socket to server socket 
//	if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) { 
	if ((rc = connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr))) != 0) { 
		printk("connection with the server failed... %s\n", strerror(errno)); 
		exit(0); 
	} 
	else
		printk("connected to the server..\n"); 

	// function for chat 
	func(sockfd); 

	// close the socket 
	close(sockfd); 
} 

