#ifndef DATA_SAVE
#define DATA_SAVE

#include <Utils/IO/Pthread.hpp>
#include "DataProtocol.h"

class DataSave: public Pthread{
public:
    DataSave(bool b_verbose);
    virtual ~DataSave();
    virtual void run (void );

private:
    bool b_verbose_;
    void _ShowDataSetup(const DATA_Protocol::DATA_SETUP & data_setup);
    int socket1_;
    int socket2_;
};

#endif
