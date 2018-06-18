#ifndef DATA_SAVE
#define DATA_SAVE

#include <Utils/DartpThread.hpp>
#include "data_protocol.h"

class DataSave: public DartpThread{
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
