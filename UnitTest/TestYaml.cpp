#include "gtest/gtest.h"
#include <iostream>
#include <Utils/IO/IOUtilities.hpp>

TEST(TestMyyaml, myyaml) {
    try {
        YAML::Node cfg = YAML::LoadFile("/Users/junhyeokahn/test.yml");
        Eigen::MatrixXd w0;
        Eigen::MatrixXd b0;
        bool b_stochastic;
        myUtils::readParameter(cfg["pol_param"], "w0", w0);
        myUtils::readParameter(cfg["pol_param"], "b0", b0);
        myUtils::readParameter(cfg["pol_param"], "b_stochastic", b_stochastic);
        myUtils::pretty_print(w0, std::cout, "w0");
        myUtils::pretty_print(b0, std::cout, "b0");
        std::cout << b_stochastic << std::endl;
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
    std::cout << "yaml test" << std::endl;
}
