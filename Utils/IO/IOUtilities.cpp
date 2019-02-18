#include "Utils/IO/IOUtilities.hpp"
#include <Configuration.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace myUtils {

void saveVector(const Eigen::VectorXd& vec_, std::string name_, bool b_param) {
    std::string file_name;
    cleaningFile(name_, file_name, b_param);

    std::ofstream savefile(file_name.c_str(), std::ios::app);
    for (int i(0); i < vec_.rows(); ++i) {
        savefile << vec_(i) << "\t";
    }
    savefile << "\n";
    savefile.flush();
}

void saveValue(double _value, std::string _name, bool b_param) {
    std::string file_name;
    cleaningFile(_name, file_name, b_param);
    std::ofstream savefile(file_name.c_str(), std::ios::app);

    savefile << _value << "\n";
    savefile.flush();
}

void saveVector(double* _vec, std::string _name, int size, bool b_param) {
    std::string file_name;
    cleaningFile(_name, file_name, b_param);
    std::ofstream savefile(file_name.c_str(), std::ios::app);

    for (int i(0); i < size; ++i) {
        savefile << _vec[i] << "\t";
    }
    savefile << "\n";
    savefile.flush();
}

void saveVector(const std::vector<double>& _vec, std::string _name,
                bool b_param) {
    std::string file_name;
    cleaningFile(_name, file_name, b_param);
    std::ofstream savefile(file_name.c_str(), std::ios::app);
    for (int i(0); i < _vec.size(); ++i) {
        savefile << _vec[i] << "\t";
    }
    savefile << "\n";
    savefile.flush();
}

void cleaningFile(std::string _file_name, std::string& _ret_file,
                  bool b_param) {
    if (b_param)
        _ret_file += THIS_COM;
    else
        _ret_file += THIS_COM "ExperimentData/";

    _ret_file += _file_name;
    _ret_file += ".txt";

    std::list<std::string>::iterator iter = std::find(
        gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
    if (gs_fileName_string.end() == iter) {
        gs_fileName_string.push_back(_file_name);
        remove(_ret_file.c_str());
    }
}

void pretty_print(Eigen::VectorXd const& vv, std::ostream& os,
                  std::string const& title, std::string const& prefix,
                  bool nonl) {
    pretty_print((Eigen::MatrixXd const&)vv, os, title, prefix, true, nonl);
}

void pretty_constructor(const int& _num_tab, const std::string& _name) {
    myColor color;
    color_print(myColor::BoldCyan, "|", false);
    std::string content = " ";
    int space_to_go(0);
    if (_num_tab != 0) {
        for (int i = 0; i < _num_tab; ++i) {
            content += "    ";
        }
        content = content + "||--" + _name;
        switch (_num_tab) {
            case 1:
                color = myColor::BoldGreen;
                break;
            case 2:
                color = myColor::BoldYellow;
                break;
            case 3:
                color = myColor::BoldBlue;
                break;
            case 4:
                color = myColor::BoldMagneta;
                break;
            default:
                std::cout << "no such color in pretty_constructor" << std::endl;
                exit(0);
        }
    } else {
        content += _name;
        color = myColor::BoldRed;
    }
    space_to_go = 78 - content.length();
    // std::cout << space_to_go << std::endl;
    for (int i = 0; i < space_to_go; ++i) {
        content += " ";
    }
    color_print(color, content, false);
    color_print(myColor::BoldCyan, "|");
}

void color_print(const myColor& _color, const std::string& _name,
                 bool line_change) {
    switch (_color) {
        case Red:
            printf("\033[0;31m");
            break;
        case BoldRed:
            printf("\033[1;31m");
            break;
        case Green:
            printf("\033[0;32m");
            break;
        case BoldGreen:
            printf("\033[1;32m");
            break;
        case Yellow:
            printf("\033[0;33m");
            break;
        case BoldYellow:
            printf("\033[1;33m");
            break;
        case Blue:
            printf("\033[0;34m");
            break;
        case BoldBlue:
            printf("\033[1;34m");
            break;
        case Magneta:
            printf("\033[0;35m");
            break;
        case BoldMagneta:
            printf("\033[1;35m");
            break;
        case Cyan:
            printf("\033[0;36m");
            break;
        case BoldCyan:
            printf("\033[1;36m");
            break;
        default:
            std::cout << "No Such Color" << std::endl;
            exit(0);
    }
    if (line_change)
        printf("%s\n", _name.c_str());
    else
        printf("%s", _name.c_str());
    printf("\033[0m");
}

void pretty_print(const std::vector<double>& _vec, const char* title) {
    std::printf("%s: ", title);
    for (int i(0); i < _vec.size(); ++i) {
        std::printf("% 6.4f, \t", _vec[i]);
    }
    std::printf("\n");
}

void pretty_print(const std::vector<int>& _vec, const char* title) {
    std::printf("%s: ", title);
    for (int i(0); i < _vec.size(); ++i) {
        std::printf("%d, \t", _vec[i]);
    }
    std::printf("\n");
}

void pretty_print(Eigen::MatrixXd const& mm, std::ostream& os,
                  std::string const& title, std::string const& prefix,
                  bool vecmode, bool nonl) {
    char const* nlornot("\n");
    if (nonl) {
        nlornot = "";
    }
    if (!title.empty()) {
        os << title << nlornot;
    }
    if ((mm.rows() <= 0) || (mm.cols() <= 0)) {
        os << prefix << " (empty)" << nlornot;
    } else {
        // if (mm.cols() == 1) {
        //   vecmode = true;
        // }

        if (vecmode) {
            if (!prefix.empty()) os << prefix;
            for (int ir(0); ir < mm.rows(); ++ir) {
                os << pretty_string(mm.coeff(ir, 0));
            }
            os << nlornot;

        } else {
            for (int ir(0); ir < mm.rows(); ++ir) {
                if (!prefix.empty()) os << prefix;
                for (int ic(0); ic < mm.cols(); ++ic) {
                    os << pretty_string(mm.coeff(ir, ic));
                }
                os << nlornot;
            }
        }
    }
}

void pretty_print(Eigen::Vector3d const& vv, std::ostream& os,
                  std::string const& title, std::string const& prefix,
                  bool nonl) {
    pretty_print((Eigen::MatrixXd const&)vv, os, title, prefix, true, nonl);
}

void pretty_print(Eigen::Quaternion<double> const& qq, std::ostream& os,
                  std::string const& title, std::string const& prefix,
                  bool nonl) {
    pretty_print(qq.coeffs(), os, title, prefix, true, nonl);
}

std::string pretty_string(Eigen::VectorXd const& vv) {
    std::ostringstream os;
    pretty_print(vv, os, "", "", true);
    return os.str();
}

std::string pretty_string(Eigen::MatrixXd const& mm,
                          std::string const& prefix) {
    std::ostringstream os;
    pretty_print(mm, os, "", prefix);
    return os.str();
}

std::string pretty_string(double vv) {
    static int const buflen(32);
    static char buf[buflen];
    memset(buf, 0, sizeof(buf));
    snprintf(buf, buflen - 1, "% 6.6f  ", vv);
    std::string str(buf);
    return str;
}

double smooth_changing(double ini, double end, double moving_duration,
                       double curr_time) {
    double ret;
    ret =
        ini + (end - ini) * 0.5 * (1 - cos(curr_time / moving_duration * M_PI));
    if (curr_time > moving_duration) {
        ret = end;
    }
    return ret;
}

double smooth_changing_vel(double ini, double end, double moving_duration,
                           double curr_time) {
    double ret;
    ret = (end - ini) * 0.5 * (M_PI / moving_duration) *
          sin(curr_time / moving_duration * M_PI);
    if (curr_time > moving_duration) {
        ret = 0.0;
    }
    return ret;
}
double smooth_changing_acc(double ini, double end, double moving_duration,
                           double curr_time) {
    double ret;
    ret = (end - ini) * 0.5 * (M_PI / moving_duration) *
          (M_PI / moving_duration) * cos(curr_time / moving_duration * M_PI);
    if (curr_time > moving_duration) {
        ret = 0.0;
    }
    return ret;
}
void getSinusoidTrajectory(double initTime_, const Eigen::VectorXd& midPoint_,
                           const Eigen::VectorXd& amp_,
                           const Eigen::VectorXd& freq_, double evalTime_,
                           Eigen::VectorXd& p_, Eigen::VectorXd& v_,
                           Eigen::VectorXd& a_) {
    int dim = midPoint_.size();
    p_ = Eigen::VectorXd::Zero(dim);
    v_ = Eigen::VectorXd::Zero(dim);
    a_ = Eigen::VectorXd::Zero(dim);
    for (int i = 0; i < dim; ++i) {
        p_[i] = amp_[i] * sin(2 * M_PI * freq_[i] * (evalTime_ - initTime_)) +
                midPoint_[i];
        v_[i] = amp_[i] * 2 * M_PI * freq_[i] *
                cos(2 * M_PI * freq_[i] * (evalTime_ - initTime_));
        a_[i] = -amp_[i] * 2 * M_PI * freq_[i] * 2 * M_PI * freq_[i] *
                sin(2 * M_PI * freq_[i] * (evalTime_ - initTime_));
    }
}

void readFile(std::string _file_name, std::vector<std::string>& _vec) {
    std::ifstream InputFile(_file_name.c_str());
    std::string tempstring;
    if (!InputFile.is_open()) {
        std::cout << "Data file load error... check the data file" << std::endl;
        exit(0);
    } else {
        while (!InputFile.eof()) {
            InputFile.clear();
            std::getline(InputFile, tempstring);
            _vec.push_back(tempstring);
        }
        InputFile.close();
    }
}

void splitString(std::string* str_array, std::string strTarget,
                 std::string strTok) {
    int nCutPos = 0;
    int nIndex = 0;
    while ((nCutPos = strTarget.find_first_of(strTok)) != strTarget.npos) {
        if (nCutPos > 0) {
            str_array[nIndex++] = strTarget.substr(0, nCutPos);
        }
        strTarget = strTarget.substr(nCutPos + 1);
    }
    if (strTarget.length() > 0) {
        str_array[nIndex++] = strTarget.substr(0, nCutPos);
    }
}

bool isEqual(const Eigen::VectorXd a, const Eigen::VectorXd b,
             const double threshold) {
    bool ret(true);
    for (int i = 0; i < a.size(); ++i) {
        if ((a(i) - b(i)) > threshold || ((a(i) - b(i))) < -threshold) {
            ret = false;
        }
    }
    return ret;
}
}  // namespace myUtils
