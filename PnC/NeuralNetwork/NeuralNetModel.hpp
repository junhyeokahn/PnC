#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <Utils/IO/IOUtilities.hpp>

enum ActivationFunction {
    None = 0,
    Tanh = 1,
    ReLU = 2
};

class Layer
{
public:
    Layer ( Eigen::MatrixXd weight, Eigen::MatrixXd bias, ActivationFunction act_fn);
    virtual ~Layer ();
    Eigen::MatrixXd GetOutput( const Eigen::MatrixXd & input );
    int GetNumInput() { return num_input_; }
    int GetNumOutput() { return num_output_; }
    Eigen::MatrixXd GetWeight() { return weight_; }
    Eigen::MatrixXd GetBias() { return bias_; }
    ActivationFunction GetActivationFunction() { return act_fn_; }

private:
    Eigen::MatrixXd weight_;
    Eigen::MatrixXd bias_;
    int num_input_;
    int num_output_;
    ActivationFunction act_fn_;
};

class NeuralNetModel
{
public:
    NeuralNetModel (std::vector<Layer> layers);
    NeuralNetModel (std::vector<Layer> layers, Eigen::MatrixXd logstd);
    NeuralNetModel (const YAML::Node& node);

    virtual ~NeuralNetModel ();

    std::vector<Layer> GetLayers() { return layers_; }
    Eigen::MatrixXd GetOutput( const Eigen::MatrixXd & input );
    Eigen::MatrixXd GetOutput( const Eigen::MatrixXd & input, int idx );
    int GetNumInput() { return num_input_; }
    int GetNumOutput() { return num_output_; }
    void PrintInfo();

private:
    void Initialize_(std::vector<Layer> layers,
                     bool b_stoch=false,
                     Eigen::MatrixXd logstd = Eigen::MatrixXd::Zero(1, 1));
    int num_input_;
    int num_output_;
    int num_layer_;
    std::vector<Layer> layers_;
    bool b_stochastic_;
    Eigen::MatrixXd logstd_;
    Eigen::MatrixXd std_;
};
