#include "Utils/NeuralNetModel.hpp"

Layer::Layer(Eigen::MatrixXd weight, Eigen::VectorXd bias, ActivationFunction act_fn) {
    weight_ = weight;
    bias_ = bias;
    num_input_ = weight.rows();
    num_output_ = weight.cols();
    act_fn_ = act_fn;
}

Layer::~Layer() {}

Eigen::VectorXd Layer::GetOutput(const Eigen::VectorXd & input) {
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(num_output_);

    return ret;
}

NeuralNetModel::NeuralNetModel(std::vector<Layer> layers) {
    layers_ = layers;
    num_hidden_layer_ = layers.size();
    num_input_ = layers[0].GetNumInput();
    num_output_ = layers.back().GetNumOutput();
}

NeuralNetModel::~NeuralNetModel() {}

Eigen::VectorXd NeuralNetModel::GetOutput( const Eigen::VectorXd & input ) {
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(num_output_);

    return ret;
}
