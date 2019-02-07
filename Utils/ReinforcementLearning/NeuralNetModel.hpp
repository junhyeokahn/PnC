#pragma once

#include "Utils/IO/IOUtilities.hpp"

/*! \enum ActivationFunction
 *
 *  Detailed description
 */
enum ActivationFunction { 
    None = 0,
    Tanh = 1,
    ReLU = 2
};

class Layer
{
public:
    Layer ( Eigen::MatrixXd weight, Eigen::VectorXd bias, ActivationFunction act_fn);
    virtual ~Layer ();
    Eigen::VectorXd GetOutput( const Eigen::VectorXd & input );
    int GetNumInput() { return num_input_; }
    int GetNumOutput() { return num_output_; }

private:
    Eigen::MatrixXd weight_;
    Eigen::VectorXd bias_;
    int num_input_;
    int num_output_;
    ActivationFunction act_fn_;
};

class NeuralNetModel
{
public:
    NeuralNetModel (std::vector<Layer> layers);
    virtual ~NeuralNetModel ();

    Eigen::VectorXd GetOutput( const Eigen::VectorXd & input );

private:
    int num_input_;
    int num_output_;
    int num_layer_;
    std::vector<Layer> layers_;
};
