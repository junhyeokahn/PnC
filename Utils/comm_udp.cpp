#include "comm_udp.hpp"

#include <string.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdexcept>
#include <iostream>
namespace COMM{

    void send_data(int & _socket, int port, void* data, int data_size, const char* ip_addr){
        int slen;
        if(_socket == 0){
            // printf("[Send Data] Open Socket\n");
            if ((_socket =socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) <= -1){
                std::cerr<<"[Send Data] fail to make socket\n"<<std::endl;
                throw new std::runtime_error("[Send Data] fail to make socket\n");
                
            }
            // printf("socket num: %i\n", _socket);
        }

        // throw new std::runtime_error("[Send Data] fail to make socket\n");
        struct sockaddr_in si_to;
        memset((char *) &si_to, 0, sizeof(si_to));

        si_to.sin_family = AF_INET;
        si_to.sin_port = htons(port);
        si_to.sin_addr.s_addr = inet_addr(ip_addr);

        int ret = sendto(_socket, data, data_size, 0,  (const struct sockaddr*)&si_to, sizeof(si_to));
    }
    

    void receive_data(int & _socket, int port, void* data, int data_size, const char* ip_addr){
	struct sockaddr_in si_me, so; 

        if(_socket == 0){
            // printf("[Recieve Data] Open Socket\n");
            if ((_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) <= -1){
                throw new std::runtime_error("fail to make socket\n");
            }
            // printf("socket num: %i\n", _socket);
        }
 
    	memset((char *) &si_me, 0, sizeof(si_me));

	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(port);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	int bindresult = bind(_socket, (const sockaddr*)&si_me, (socklen_t)sizeof(si_me));

	so.sin_family = AF_INET;
	so.sin_port = htons(port);
	so.sin_addr.s_addr = inet_addr(ip_addr);
        int slen = sizeof(so);

        recvfrom(_socket, data, data_size, 0, (sockaddr*)&so, (socklen_t*) & slen);
    }


    int client_socket(0);

    void receive_data_unix(const char* server_name, void * data, int data_size){
        sockaddr_un server_addr, client_addr;
        int server_socket, rval;
        
        if((server_socket = socket(AF_UNIX, SOCK_STREAM, 0)) == -1){
                printf("[server socket] fail to make socket\n");
                return;
        }
        server_addr.sun_family = AF_UNIX;
        strcpy(server_addr.sun_path, server_name);
        if(bind(server_socket, (struct sockaddr*) & server_addr, sizeof(struct sockaddr_un))){
            // perror("biding server socket");
        }
        listen(server_socket, 5);

        //client socket address
        client_addr.sun_family = AF_UNIX;
        strcpy(client_addr.sun_path, server_name);
        int client_len = sizeof(struct sockaddr_un);
        client_socket = accept(server_socket, (sockaddr*) & client_addr, (socklen_t*) & client_len);
        if( client_socket == -1){
            perror("accept");
        }
        else {
            rval = read(client_socket, data, data_size);
            if(rval < 0){ perror("reading stream message"); }
        }
        close(client_socket);
        close(server_socket);
    }

    void send_data_unix(const char* server_name, void * data, int data_size){
        sockaddr_un addr;
        int sock;

        sock = socket(AF_UNIX, SOCK_STREAM, 0);
        if( sock < 0) { printf("[client socket] fail to open socket\n"); }

        addr.sun_family = AF_UNIX;
        strcpy(addr.sun_path, server_name);

        if( connect(sock, (struct sockaddr *) &addr, sizeof(struct sockaddr_un)) <0){
            close(sock);
            perror("connecting client socket");
        }
        if( write(sock, data, data_size) < 0) {
            perror( "writing on socket");
        }
        close(sock);
    }
}
