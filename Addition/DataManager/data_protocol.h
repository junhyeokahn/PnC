#ifndef DATA_PROTOCOL
#define DATA_PROTOCOL
#include <Configuration.h>
#include <stdint.h>

#define IP_ADDR_MYSELF "127.0.0.1"
#define IP_JUNHYEOK "192.168.0.151"
#define PORT_DATA_SETUP 61124
#define PORT_DATA_RECEIVE 61125

#define IP_ADDR IP_ADDR_MYSELF
//#define IP_ADDR IP_JUNHYEOK

#define MAX_NUM_DATA 100

namespace DATA_Protocol {
typedef struct {
    int num_data;
    int tot_num_array_data;
    char data_name[MAX_NUM_DATA][20];
    int num_array_data[MAX_NUM_DATA];
} DATA_SETUP;
};  // namespace DATA_Protocol

#endif
