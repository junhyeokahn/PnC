#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>

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
    virtual ~NeuralNetModel ();

    Eigen::MatrixXd GetOutput( const Eigen::MatrixXd & input );

    Eigen::MatrixXd GetOutput( const Eigen::MatrixXd & input,
                               const Eigen::MatrixXd & logstd );
private:
    int num_input_;
    int num_output_;
    int num_layer_;
    std::vector<Layer> layers_;
};
