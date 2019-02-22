#include <ReinforcementLearning/RLInterface/NeuralNetModel.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <cassert>
#include <cmath>
#include <random>

Layer::Layer(Eigen::MatrixXd weight, Eigen::MatrixXd bias,
             ActivationFunction act_fn) {
    weight_ = weight;
    bias_ = bias;
    num_input_ = weight.rows();
    num_output_ = weight.cols();
    act_fn_ = act_fn;
}

Layer::~Layer() {}

Eigen::MatrixXd Layer::GetOutput(const Eigen::MatrixXd& input) {
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
                    if (ret(row, col) < 0) {
                        ret(row, col) = 0.;
                    }
                }
            }
            return ret;
            break;
        default:
            break;
    }
    return ret;
}

NeuralNetModel::NeuralNetModel(const YAML::Node& node, bool b_stochastic) {
    int num_layer;
    Eigen::MatrixXd w, b, logstd;
    std::vector<Layer> layers;
    layers.clear();
    int act_fn;

    try {
        myUtils::readParameter(node, "num_layer", num_layer);
        for (int idx_layer = 0; idx_layer < num_layer; ++idx_layer) {
            myUtils::readParameter(node, "w" + std::to_string(idx_layer), w);
            myUtils::readParameter(node, "b" + std::to_string(idx_layer), b);
            myUtils::readParameter(node, "act_fn" + std::to_string(idx_layer),
                                   act_fn);
            layers.push_back(
                Layer(w, b, static_cast<ActivationFunction>(act_fn)));
        }
        if (b_stochastic) {
            myUtils::readParameter(node, "logstd", logstd);
        }
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

    Initialize_(layers, b_stochastic, logstd);
}

NeuralNetModel::NeuralNetModel(std::vector<Layer> layers,
                               Eigen::VectorXd logstd) {
    Initialize_(layers, true, logstd);
}

NeuralNetModel::NeuralNetModel(std::vector<Layer> layers) {
    Initialize_(layers);
}

NeuralNetModel::~NeuralNetModel() {}

Eigen::MatrixXd NeuralNetModel::GetOutput(const Eigen::MatrixXd& input,
                                          int ith) {
    Eigen::MatrixXd ret = input;
    if (ith > num_layer_) {
        std::cout << "[[Error]] Wrong output layer idx!!" << std::endl;
        exit(0);
    }

    for (int i = 0; i < ith; ++i) {
        ret = (layers_[i]).GetOutput(ret);
    }

    return ret;
}

Eigen::MatrixXd NeuralNetModel::GetOutput(const Eigen::MatrixXd& input) {
    int num_data(input.rows());
    Eigen::MatrixXd ret = input;
    for (int i = 0; i < num_layer_; ++i) {
        ret = (layers_[i]).GetOutput(ret);
    }

    if (b_stochastic_) {
        std::random_device rd{};
        std::mt19937 gen{rd()};
        for (int row = 0; row < num_data; ++row) {
            for (int col = 0; col < num_output_; ++col) {
                std::normal_distribution<> d{ret(row, col), std_(0, col)};
                ret(row, col) = d(gen);
            }
        }
    }
    return ret;
}

void NeuralNetModel::GetOutput(const Eigen::MatrixXd& _input,
                               const Eigen::VectorXd& _lb,
                               const Eigen::VectorXd& _ub,
                               Eigen::MatrixXd& _output, Eigen::MatrixXd& _mean,
                               Eigen::VectorXd& _neglogp) {
    assert(b_stochastic_);
    assert(_lb.size() == _ub.size());
    assert(_lb.size() == num_output_);

    int num_data(_input.rows());
    Eigen::MatrixXd aug_lb = Eigen::MatrixXd::Zero(num_data, num_output_);
    Eigen::MatrixXd aug_ub = Eigen::MatrixXd::Zero(num_data, num_output_);
    for (int data_idx = 0; data_idx < num_data; ++data_idx) {
        for (int output_idx = 0; output_idx < num_output_; ++output_idx) {
            aug_lb(data_idx, output_idx) = _lb(output_idx);
            aug_ub(data_idx, output_idx) = _ub(output_idx);
        }
    }

    Eigen::MatrixXd mean = _input;
    Eigen::MatrixXd output = Eigen::MatrixXd::Zero(num_data, num_output_);
    Eigen::VectorXd neglogp = Eigen::VectorXd::Zero(num_data);
    for (int i = 0; i < num_layer_; ++i) {
        mean = (layers_[i]).GetOutput(mean);
    }

    std::random_device rd{};
    std::mt19937 gen{rd()};
    for (int row = 0; row < num_data; ++row) {
        for (int col = 0; col < num_output_; ++col) {
            std::normal_distribution<> d{mean(row, col), std_(col)};
            output(row, col) = d(gen);
        }
    }
    output = myUtils::CropMatrix(output, aug_lb, aug_ub, "neural net output");

    for (int data_idx = 0; data_idx < num_data; ++data_idx) {
        for (int output_idx = 0; output_idx < num_output_; ++output_idx) {
            neglogp(data_idx) = neglogp(data_idx) +
                                0.5 * std::pow((output(data_idx, output_idx) -
                                                mean(data_idx, output_idx)) /
                                                   std_(output_idx),
                                               2) +
                                logstd_(output_idx);
        }
        neglogp(data_idx) =
            neglogp(data_idx) + 0.5 * log(2 * M_PI) * (double)num_output_;
    }

    _output = output;
    _mean = mean;
    _neglogp = neglogp;
}

void NeuralNetModel::Initialize_(std::vector<Layer> layers, bool b_stoch,
                                 Eigen::VectorXd logstd) {
    layers_ = layers;
    num_layer_ = layers.size();
    num_input_ = layers[0].GetNumInput();
    num_output_ = layers.back().GetNumOutput();
    b_stochastic_ = b_stoch;
    if (b_stochastic_) {
        logstd_ = logstd;
        std_ = logstd;
        for (int i = 0; i < num_output_; ++i) {
            std_(i) = std::exp(logstd_(i));
        }
    }
}

void NeuralNetModel::PrintInfo() {
    std::cout << "Num Layer : " << num_layer_ << std::endl;
    for (int i = 0; i < num_layer_; ++i) {
        std::cout << i << " th Layer : (" << layers_[i].GetNumInput() << ", "
                  << layers_[i].GetNumOutput() << ") with Activation type::"
                  << layers_[i].GetActivationFunction() << std::endl;
        myUtils::pretty_print(layers_[i].GetWeight(), std::cout, "w");
        myUtils::pretty_print(layers_[i].GetBias(), std::cout, "b");
    }
    if (b_stochastic_) {
        myUtils::pretty_print(logstd_, std::cout, "logstd");
    }
}
