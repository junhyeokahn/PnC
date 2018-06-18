#ifndef UTILITIES
#define UTILITIES

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include <Configuration.h>
#include <list>
#include <Eigen/Dense>

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
    void cleaningFile(std::string file_name_,
                      std::string & ret_file_,
                      bool b_param);
    static std::list< std::string > gs_fileName_string; //global & static

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
    std::string pretty_string(Eigen::VectorXd const & vv);
    std::string pretty_string(Eigen::MatrixXd const & mm, std::string const & prefix);
    std::string pretty_string(double vv);

    // ===============
    // Smooth Changing
    // ===============
    double smooth_changing(double ini, double end,
            double moving_duration, double curr_time);
    double smooth_changing_vel(double ini, double end,
            double moving_duration, double curr_time);
    double smooth_changing_acc(double ini, double end,
            double moving_duration, double curr_time);

    // ===
    // ETC
    // ===
    double bind_half_pi(double);
} /* myUtils */

#endif
