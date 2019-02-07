#ifndef COMM_UDP
#define COMM_UDP

#include <string>

namespace COMM{
    void receive_data(int &socket, int port, void* data, int data_size, const char* ip_addr);
    void send_data(int &socket, int port, void * data, int data_size, const char* ip_addr);
//    void check_send_socket(int* socket, int* ret_socket);

    void receive_data_unix(const char* server_name, void* data, int data_size);
    void send_data_unix   (const char* server_name, void* data, int data_size);
}

#endif
