#include <Configuration.h>
#include "Utils/Utilities.hpp"
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include "drake/math/eigen_sparse_triplet.h"

namespace myUtils {
    Eigen::MatrixXd hStack(const Eigen::MatrixXd a,
            const Eigen::MatrixXd b) {

        if ( a.rows() != b.rows() ) {
            std::cout << "[hStack] Matrix Size is Wrong" << std::endl; exit(0);
        }

        Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows(), a.cols() + b.cols());
        ab << a, b;
        return ab;
    }

    Eigen::MatrixXd vStack(const Eigen::MatrixXd a,
            const Eigen::MatrixXd b) {
        if ( a.cols() != b.cols() ) {
            std::cout << "[vStack] Matrix Size is Wrong" << std::endl;
            exit(0);
        }
        Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols());
        ab << a, b;
        return ab;
    }

    Eigen::MatrixXd vStack(const Eigen::VectorXd a,
            const Eigen::VectorXd b) {

        if (a.size() != b.size()) {
            std::cout << "[vStack] Vector Size is Wrong" << std::endl;
            exit(0);
        }
        Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.size(), 2);
        ab << a, b;
        return ab;
    }

    Eigen::MatrixXd deleteRow(const Eigen::MatrixXd & a_, int row_) {
        Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(a_.rows()-1, a_.cols());
        ret.block(0, 0, row_, a_.cols()) = a_.block(0, 0, row_, a_.cols());
        ret.block(row_, 0, ret.rows()-row_, a_.cols()) =
            a_.block(row_+1, 0, ret.rows()-row_, a_.cols());
        return ret;
    }

    void saveVector(const Eigen::VectorXd & vec_, std::string name_, bool b_param){
        std::string file_name;
        cleaningFile(name_, file_name, b_param);

        std::ofstream savefile(file_name.c_str(), std::ios::app);
        for (int i(0); i < vec_.rows(); ++i){
            savefile<<vec_(i)<< "\t";
        }
        savefile<<"\n";
        savefile.flush();
    }

    void saveValue(double _value, std::string _name, bool b_param){
        std::string file_name;
        cleaningFile(_name, file_name, b_param);
        std::ofstream savefile(file_name.c_str(), std::ios::app);

        savefile<<_value <<"\n";
        savefile.flush();
    }

    void saveVector(double * _vec, std::string _name, int size, bool b_param){
        std::string file_name;
        cleaningFile(_name, file_name, b_param);
        std::ofstream savefile(file_name.c_str(), std::ios::app);

        for (int i(0); i < size; ++i){
            savefile<<_vec[i]<< "\t";
        }
        savefile<<"\n";
        savefile.flush();
    }

    void saveVector(const std::vector<double> & _vec, std::string _name, bool b_param){
        std::string file_name;
        cleaningFile(_name, file_name, b_param);
        std::ofstream savefile(file_name.c_str(), std::ios::app);
        for (int i(0); i < _vec.size(); ++i){
            savefile<<_vec[i]<< "\t";
        }
        savefile<<"\n";
        savefile.flush();
    }

    void cleaningFile(std::string  _file_name, std::string & _ret_file, bool b_param){
        if(b_param)
            _ret_file += THIS_COM;
        else
            _ret_file += THIS_COM"ExperimentData/";

        _ret_file += _file_name;
        _ret_file += ".txt";

        std::list<std::string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
        if(gs_fileName_string.end() == iter){
            gs_fileName_string.push_back(_file_name);
            remove(_ret_file.c_str());
        }
    }

    void pretty_print(Eigen::VectorXd const & vv, std::ostream & os,
            std::string const & title, std::string const & prefix,
            bool nonl) {
        pretty_print((Eigen::MatrixXd const &) vv, os, title, prefix, true, nonl);
    }

    void pretty_print(Eigen::MatrixXd const & mm, std::ostream & os,
            std::string const & title, std::string const & prefix,
            bool vecmode, bool nonl) {
        char const * nlornot("\n");
        if (nonl) {
            nlornot = "";
        }
        if ( ! title.empty()) {
            os << title << nlornot;
        }
        if ((mm.rows() <= 0) || (mm.cols() <= 0)) {
            os << prefix << " (empty)" << nlornot;
        }
        else {
            // if (mm.cols() == 1) {
            //   vecmode = true;
            // }

            if (vecmode) {
                if ( ! prefix.empty())
                    os << prefix;
                for (int ir(0); ir < mm.rows(); ++ir) {
                    os << pretty_string(mm.coeff(ir, 0));
                }
                os << nlornot;

            }
            else {

                for (int ir(0); ir < mm.rows(); ++ir) {
                    if ( ! prefix.empty())
                        os << prefix;
                    for (int ic(0); ic < mm.cols(); ++ic) {
                        os << pretty_string(mm.coeff(ir, ic));
                    }
                    os << nlornot;
                }

            }
        }
    }

    std::string pretty_string(Eigen::VectorXd const & vv) {
        std::ostringstream os;
        pretty_print(vv, os, "", "", true);
        return os.str();
    }

    std::string pretty_string(Eigen::MatrixXd const & mm, std::string const & prefix) {
        std::ostringstream os;
        pretty_print(mm, os, "", prefix);
        return os.str();
    }

    std::string pretty_string(double vv)
    {
        static int const buflen(32);
        static char buf[buflen];
        memset(buf, 0, sizeof(buf));
        snprintf(buf, buflen-1, "% 6.6f  ", vv);
        std::string str(buf);
        return str;
    }

    double smooth_changing(double ini, double end, double moving_duration, double curr_time){
        double ret;
        ret = ini + (end - ini)*0.5*(1-cos(curr_time/moving_duration * M_PI));
        if(curr_time>moving_duration){
            ret = end;
        }
        return ret;
    }

    double smooth_changing_vel(double ini, double end, double moving_duration, double curr_time){
        double ret;
        ret = (end - ini)*0.5*(M_PI/moving_duration)*sin(curr_time/moving_duration*M_PI);
        if(curr_time>moving_duration){
            ret = 0.0;
        }
        return ret;
    }
    double smooth_changing_acc(double ini, double end, double moving_duration, double curr_time){
        double ret;
        ret = (end - ini)*0.5*(M_PI/moving_duration)*(M_PI/moving_duration)*cos(curr_time/moving_duration*M_PI);
        if(curr_time>moving_duration){
            ret = 0.0;
        }
        return ret;
    }
    void getSinusoidTrajectory(double initTime_,
                               const Eigen::VectorXd & midPoint_,
                               const Eigen::VectorXd & amp_,
                               const Eigen::VectorXd & freq_,
                               double evalTime_,
                               Eigen::VectorXd & p_,
                               Eigen::VectorXd & v_,
                               Eigen::VectorXd & a_) {
        int dim = midPoint_.size();
        p_ = Eigen::VectorXd::Zero(dim);
        v_ = Eigen::VectorXd::Zero(dim);
        a_ = Eigen::VectorXd::Zero(dim);
        for (int i = 0; i < dim; ++i) {
            p_[i] =
                amp_[i] * sin(2*M_PI*freq_[i]*(evalTime_-initTime_)) + midPoint_[i];
            v_[i] =
                amp_[i]*2*M_PI*freq_[i]*cos(2*M_PI*freq_[i]*(evalTime_-initTime_));
            a_[i] =
                -amp_[i]*2*M_PI*freq_[i]*2*M_PI*freq_[i]*sin(2*M_PI*freq_[i]*(evalTime_-initTime_));
        }
    }

    double bind_half_pi(double ang){
        if(ang > M_PI/2){
            return ang - M_PI;
        }
        if(ang < -M_PI/2){
            return ang + M_PI;
        }
        return ang;
    }

    void readFile(std::string _file_name, std::vector<std::string> & _vec){
        std::ifstream  InputFile(_file_name.c_str());
        std::string tempstring;
        if(!InputFile.is_open()){
            std::cout << "Data file load error... check the data file" << std::endl;
            exit(0);
        }
        else{
            while(!InputFile.eof()){
                InputFile.clear();
                std::getline(InputFile,tempstring);
                _vec.push_back(tempstring);
            }
            InputFile.close();
        }
    }

    void splitString(std::string* str_array, std::string strTarget, std::string strTok ){
        int nCutPos = 0;
        int nIndex = 0;
        while ((nCutPos = strTarget.find_first_of(strTok)) != strTarget.npos){
            if (nCutPos > 0){
                str_array[nIndex++] = strTarget.substr(0, nCutPos);
            }
            strTarget = strTarget.substr(nCutPos+1);
        }
        if(strTarget.length() > 0){
            str_array[nIndex++] = strTarget.substr(0, nCutPos);
        }
    }

    bool isEqual(const Eigen::VectorXd a, const Eigen::VectorXd b, const double threshold) {
        bool ret(true);
        for (int i = 0; i < a.size(); ++i) {
            if ( (a(i) - b(i)) > threshold || ((a(i) - b(i))) < -threshold) {
                ret = false;
            }
        }
        return ret;
    }

    double cropValue(double value, double min, double max, std::string source){
        if(value> max){
            printf("%s: Hit The MAX\t", source.c_str());
            // printf("Original Value: %f\n", value);
            return max;
        }
        else if(value < min){
            printf("%s: Hit The MIN\t", source.c_str());
            // printf("Original Value: %f\n", value);
            return min;
        }
        else{
            return value;
        }
    }

    void collectNonZeroIdxAndValue( const Eigen::MatrixXd A,
                                          Eigen::VectorXi & rows,
                                          Eigen::VectorXi & cols,
                                          Eigen::VectorXd & vals) {
        std::vector<Eigen::Index> row_indices;
        std::vector<Eigen::Index> col_indices;
        std::vector<double> val;
        drake::math::SparseMatrixToRowColumnValueVectors(A, row_indices, col_indices, val);
        std::vector<int> non_zero_row_indices;
        std::vector<int> non_zero_col_indices;
        std::vector<double> non_zero_val;
        for (int i = 0; i < row_indices.size(); ++i) {
            if (!(val[i] > -0.00001 && val[i] < 0.00001)) {
                non_zero_row_indices.push_back(row_indices[i]);
                non_zero_col_indices.push_back(col_indices[i]);
                non_zero_val.push_back(val[i]);
            }
        }
        rows.resize(non_zero_row_indices.size());
        cols.resize(non_zero_row_indices.size());
        vals.resize(non_zero_row_indices.size());
        for (int i = 0; i < non_zero_row_indices.size(); ++i) {
            rows[i] = non_zero_row_indices[i];
            cols[i] = non_zero_col_indices[i];
            vals[i] = non_zero_val[i];
        }
    }

    bool isInBoundingBox( const Eigen::VectorXd & lb,
                          const Eigen::VectorXd & val,
                          const Eigen::VectorXd & ub) {
        int n = lb.size();
        bool ret(true);
        for (int i = 0; i < n; ++i) {
            if (lb[i] <= val[i] && val[i] <= ub[i]) {

            } else {
                ret = false;
            }
        }
        return ret;
    }

    Eigen::VectorXd eulerIntegration( const Eigen::VectorXd & x,
                                      const Eigen::VectorXd & xdot,
                                      double dt ) {
        Eigen::VectorXd ret = x;
        ret += xdot * dt;
        return ret;
    }

    Eigen::VectorXd doubleIntegration( const Eigen::VectorXd & q,
                                       const Eigen::VectorXd & alpha,
                                       const Eigen::VectorXd & alphad,
                                       double dt ) {
        Eigen::VectorXd ret = q;
        ret += alpha * dt + alphad * dt * dt * 0.5;
        return ret;
    }
}
