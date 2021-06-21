#ifndef DATA_MANAGER
#define DATA_MANAGER

#include <Addition/DataManager/DataProtocol.h>
#include <Utils/IO/Pthread.hpp>
#include <vector>
#include <string>
#include <Eigen/Dense>

enum DATA_Type{
    DOUBLE,
    INT,
    VECT2,
    VECT3,
    VECT4,
    QUATERNION,
    VECT
};

class DataManager: public Pthread{
public:
    static DataManager* GetDataManager();
    virtual ~DataManager();

    void RegisterData(const void* data_addr, DATA_Type dtype, const std::string & dname, int num_array = 1);

    virtual void run();
    void SaveDataFromValues(double * data, int st_idx, int data_idx);

private:
    void _ShowSendingMessage(const DATA_Protocol::DATA_SETUP & data_setup, const double * data);

    DataManager();
    int num_data_;
    int tot_num_array_data_;
    std::vector<const void*> data_addr_vec_;
    std::vector<DATA_Type> data_type_vec_;
    std::vector<int> data_array_size_;
    std::vector<std::string> data_name_vec_;

    int socket1_, socket2_;
};

#endif
