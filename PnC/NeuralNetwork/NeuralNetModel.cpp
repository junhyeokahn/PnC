#include <cmath>
#include <random> 
#include "PnC/NeuralNetwork/NeuralNetModel.hpp"

Layer::Layer(Eigen::MatrixXd weight, Eigen::MatrixXd bias, ActivationFunction act_fn) {
    weight_ = weight;
    bias_ = bias;
    num_input_ = weight.rows();
    num_output_ = weight.cols();
    act_fn_ = act_fn;
}

Layer::~Layer() {}

Eigen::MatrixXd Layer::GetOutput(const Eigen::MatrixXd & input) {
    int num_data(input.rows());
    Eigen::MatrixXd aug_bias = Eigen::MatrixXd::Zero(num_data, num_output_);
    for (int i = 0; i < num_data; ++i) {
        aug_bias.block(i, 0, 1, num_output_) = bias_;
    }
    Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(num_data, num_output_);
    ret = input * weight_ + aug_bias;
    switch (act_fn_) {
        case ActivationFunction::None:
            return ret;
            break;
        case ActivationFunction::Tanh:
            for (int row(0); row < num_data; ++row) {
                for (int col(0); col < num_output_; ++col) {
                    ret(row, col) = std::tanh(ret(row, col));
                }
            }
            return ret;
            break;
        case ActivationFunction::ReLU:
            for (int row(0); row < num_data; ++row) {
                for (int col(0); col < num_output_; ++col) {
                    if (ret(row, col) < 0) { ret(row,col) = 0.; }
                }
            }
            return ret;
            break;
        default:
            break;
    }
    return ret;
}

NeuralNetModel::NeuralNetModel(std::vector<Layer> layers, Eigen::MatrixXd logstd) {
    layers_ = layers;
    num_layer_ = layers.size();
    num_input_ = layers[0].GetNumInput();
    num_output_ = layers.back().GetNumOutput();
    b_stochastic_ = true;
    logstd_ = logstd;
    std_ = logstd;
    for (int i = 0; i < num_output_; ++i) {
        std_(0, i) = std::exp(logstd_(0, i));
    }
}

NeuralNetModel::NeuralNetModel(std::vector<Layer> layers) {
    layers_ = layers;
    num_layer_ = layers.size();
    num_input_ = layers[0].GetNumInput();
    num_output_ = layers.back().GetNumOutput();
    b_stochastic_ = false;
}

NeuralNetModel::~NeuralNetModel() {}

Eigen::MatrixXd NeuralNetModel::GetOutput( const Eigen::MatrixXd & input) {

    int num_data(input.rows());
    Eigen::MatrixXd ret = input;
    for (int i = 0; i < num_layer_; ++i) {
        ret = (layers_[i]).GetOutput(ret);
    }

    if (b_stochastic_) {
        std::random_device rd{};
        std::mt19937 gen{rd()};
        for (int row = 0; row < num_data; ++row) {
            for( int col = 0; col < num_output_; ++col ) {
                std::normal_distribution<> d{ret(row, col), std_(0, col)};
                ret(row, col) = d(gen);
            }
        }
    }
    return ret;
}
