#include "Utils/IO/DataManager.hpp"
#include <Utils/IO/comm_udp.hpp>
#include <iostream>

DataManager::~DataManager() {}

DataManager* DataManager::GetDataManager() {
    static DataManager data_manager_;
    return &data_manager_;
}

DataManager::DataManager()
    : num_data_(0), socket1_(0), socket2_(0), tot_num_array_data_(0) {
    data_addr_vec_.clear();
    data_type_vec_.clear();
    data_name_vec_.clear();
    data_array_size_.clear();
}

void DataManager::RegisterData(const void* data_addr, DATA_Type dtype,
                               const std::string& dname, int num_array) {
    data_addr_vec_.push_back(data_addr);
    data_type_vec_.push_back(dtype);
    data_name_vec_.push_back(dname);
    data_array_size_.push_back(num_array);
    ++num_data_;
    tot_num_array_data_ += num_array;
}

void DataManager::SaveDataFromValues(double* data, int st_idx, int data_idx) {
    switch (data_type_vec_[data_idx]) {
        case DOUBLE:
            double tmp_d;
            std::memcpy(&tmp_d, data_addr_vec_[data_idx], sizeof(double));
            data[st_idx] = tmp_d;
            break;
        case INT:
            int tmp_i;
            std::memcpy(&tmp_i, data_addr_vec_[data_idx], sizeof(int));
            data[st_idx] = (double)tmp_i;
            break;
        case VECT2: {
            Eigen::Vector2d* tmp_vec2 =
                (Eigen::Vector2d*)data_addr_vec_[data_idx];
            if (data_array_size_[data_idx] != 2) {
                printf("[Warning] Array size incorrect: %i th data\n",
                       data_idx);
            }
            for (int i(0); i < data_array_size_[data_idx]; ++i) {
                data[st_idx + i] = (*tmp_vec2)[i];
            }
            break;
        }

        case VECT3: {
            Eigen::Vector3d* tmp_vec3 =
                (Eigen::Vector3d*)data_addr_vec_[data_idx];
            if (data_array_size_[data_idx] != 3) {
                printf("[Warning] Array size incorrect: %i th data\n",
                       data_idx);
            }
            for (int i(0); i < data_array_size_[data_idx]; ++i) {
                data[st_idx + i] = (*tmp_vec3)[i];
            }
            break;
        }
        case VECT4: {
            Eigen::Vector4d* tmp_vec4 =
                (Eigen::Vector4d*)data_addr_vec_[data_idx];
            if (data_array_size_[data_idx] != 4) {
                printf("[Warning] Array size incorrect: %i th data\n",
                       data_idx);
            }

            for (int i(0); i < data_array_size_[data_idx]; ++i) {
                data[st_idx + i] = (*tmp_vec4)[i];
            }
            break;
        }
        case QUATERNION: {
            Eigen::Quaternion<double>* tmp_quat =
                (Eigen::Quaternion<double>*)data_addr_vec_[data_idx];
            if (data_array_size_[data_idx] != 4) {
                printf("[Warning] Array size incorrect: %i th data\n",
                       data_idx);
            }
            data[st_idx] = (*tmp_quat).w();
            data[st_idx + 1] = (*tmp_quat).x();
            data[st_idx + 2] = (*tmp_quat).y();
            data[st_idx + 3] = (*tmp_quat).z();
            break;
        }
        case VECT: {
            Eigen::VectorXd* tmp_vec =
                (Eigen::VectorXd*)data_addr_vec_[data_idx];
            for (int i(0); i < data_array_size_[data_idx]; ++i) {
                data[st_idx + i] = (*tmp_vec)[i];
            }
            break;
        }
    }
}
void DataManager::run() {
    // Data Setup
    DATA_Protocol::DATA_SETUP data_setup;

    data_setup.num_data = num_data_;
    data_setup.tot_num_array_data = tot_num_array_data_;

    for (int i(0); i < num_data_; ++i) {
        memcpy(data_setup.data_name[i], data_name_vec_[i].c_str(),
               data_name_vec_[i].size());
        data_setup.num_array_data[i] = data_array_size_[i];
    }
    COMM::send_data(socket1_, PORT_DATA_SETUP, &data_setup,
                    sizeof(DATA_Protocol::DATA_SETUP), IP_ADDR);

    double* data = new double[tot_num_array_data_];

    while (true) {
        // Generate Sending Message
        int st_idx(0);
        for (int i(0); i < num_data_; ++i) {
            SaveDataFromValues(data, st_idx, i);
            st_idx += data_array_size_[i];
        }
        // _ShowSendingMessage(data_setup, data);
        COMM::send_data(socket2_, PORT_DATA_RECEIVE, data,
                        data_setup.tot_num_array_data * sizeof(double),
                        IP_ADDR);

        usleep(5000);
    }

    delete[] data;
}
void DataManager::_ShowSendingMessage(
    const DATA_Protocol::DATA_SETUP& data_setup, const double* data) {
    int st_idx = 0;
    for (int i(0); i < data_setup.num_data; ++i) {
        std::cout << i << "th data [ " << data_name_vec_[i] << " ]: ";
        for (int j(0); j < data_setup.num_array_data[i]; ++j) {
            std::cout << data[st_idx + j] << ", ";
        }
        st_idx += data_setup.num_array_data[i];
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
