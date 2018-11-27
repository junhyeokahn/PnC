#pragma once

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include <list>
#include <Eigen/Dense>
#include "Configuration.h"
#include "ExternalSource/myYaml/include/myYaml/yaml.h"

enum myColor {
    Red=0,
    BoldRed=1,
    Green=2,
    BoldGreen=3,
    Yellow=4,
    BoldYellow=5,
    Blue=6,
    BoldBlue=7,
    Magneta=8,
    BoldMagneta=9,
    Cyan=10,
    BoldCyan=11
};

namespace myUtils
{
    // =============
    // Linear Algebra
    // =============

    // ============
    // Matrix Utils
    // ============
    Eigen::MatrixXd hStack(const Eigen::MatrixXd & a_, const Eigen::MatrixXd & b_);
    Eigen::MatrixXd vStack(const Eigen::MatrixXd & a_, const Eigen::MatrixXd & b_);
    Eigen::MatrixXd vStack(const Eigen::VectorXd & a_, const Eigen::VectorXd & b_);
    Eigen::MatrixXd deleteRow(const Eigen::MatrixXd & a_, int row);
    Eigen::MatrixXd deleteCol(const Eigen::MatrixXd & a_, int col);

    // ===========
    // Save Vector
    // ===========
    void saveVector(const Eigen::VectorXd & vec_,
                    std::string name_,
                    bool b_param = false);
    void saveVector(double * _vec, std::string _name, int size, bool b_param = false);
    void saveVector(const std::vector<double> & _vec, std::string _name, bool b_param = false);
    void saveValue(double _value, std::string _name, bool b_param = false);
    void cleaningFile(std::string file_name_,
                      std::string & ret_file_,
                      bool b_param);
    static std::list< std::string > gs_fileName_string; //global & static

    // =========
    // Read File
    // =========
    void readFile(std::string file_name_, std::vector<std::string> & _vec);
    void splitString(std::string* str_array, std::string strTarget, std::string strTok );
    template <typename YamlType>
        YamlType readParameter(const YAML::Node& node, const std::string& name) {
            try { return node[name.c_str()].as<YamlType>(); }
            catch (...) { throw std::runtime_error(name); }
        };

    template <typename YamlType>
        void readParameter(const YAML::Node& node, const std::string& name, YamlType& parameter) {
            try { parameter = readParameter<YamlType>(node, name); }
            catch (...) { throw std::runtime_error(name); }
        };

    // ============
    // Pretty Print
    // ============
    void pretty_print(Eigen::VectorXd const & vv, std::ostream & os,
            std::string const & title,
            std::string const & prefix="", bool nonl = false);
    void pretty_print(Eigen::MatrixXd const & mm, std::ostream & os,
            std::string const & title,
            std::string const & prefix ="",
            bool vecmode = false, bool nonl = false);
    void pretty_print( Eigen::Quaternion<double> const & qq, std::ostream & os,
            std::string const & title,
            std::string const & prefix="", bool nonl = false);
    void pretty_print( Eigen::Vector3d const & vv, std::ostream & os,
            std::string const & title, std::string const & prefix="", bool nonl = false);
    void pretty_print(const std::vector<double> & _vec, const char* title);
    void pretty_print(const std::vector<int> & _vec, const char* title);
    void pretty_constructor(const int& _num_tab, const std::string& _name);
    void color_print(const myColor & _color, const std::string& _name);
    std::string pretty_string(Eigen::VectorXd const & vv);
    std::string pretty_string(Eigen::MatrixXd const & mm, std::string const & prefix);
    std::string pretty_string(double vv);


    // ===========================
    // Simple Trajectory Generator
    // ===========================
    double smooth_changing(double ini, double end,
            double moving_duration, double curr_time);
    double smooth_changing_vel(double ini, double end,
            double moving_duration, double curr_time);
    double smooth_changing_acc(double ini, double end,
            double moving_duration, double curr_time);
    void getSinusoidTrajectory(double initTime_,
                               const Eigen::VectorXd & midPoint_,
                               const Eigen::VectorXd & amp_,
                               const Eigen::VectorXd & freq_,
                               double evalTime_,
                               Eigen::VectorXd & p_,
                               Eigen::VectorXd & v_,
                               Eigen::VectorXd & a_);

    // ===
    // ETC
    // ===
    double bind_half_pi(double);
    bool isEqual(const Eigen::VectorXd a,
                 const Eigen::VectorXd b,
                 const double threshold=0.00001);
    double cropValue(double value, double min, double max, std::string source);
    bool isInBoundingBox( const Eigen::VectorXd & lb,
                          const Eigen::VectorXd & val,
                          const Eigen::VectorXd & ub);
    Eigen::VectorXd eulerIntegration( const Eigen::VectorXd & x,
                                      const Eigen::VectorXd & xdot,
                                      double dt );
    Eigen::VectorXd doubleIntegration ( const Eigen::VectorXd & q,
                                        const Eigen::VectorXd & alpha,
                                        const Eigen::VectorXd & alphad,
                                        double dt );
} /* myUtils */
