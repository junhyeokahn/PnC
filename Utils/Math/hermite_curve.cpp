#include <cmath>
#include <Utils/Math/hermite_curve.hpp>

// Constructor
HermiteCurve::HermiteCurve(){
  p1 = 0; v1 = 0;
  p2 = 0; v2 = 0;
  s_ = 0;
  // std::cout << "[Hermite Curve] constructed" << std::endl;
}

HermiteCurve::HermiteCurve(const double & start_pos, const double & start_vel, 
                           const double & end_pos, const double & end_vel): p1(start_pos), v1(start_vel), p2(end_pos), v2(end_vel){
  s_ = 0;
  // std::cout << "[Hermite Curve] constructed with values" << std::endl;
}

// Destructor
HermiteCurve::~HermiteCurve(){}


// Cubic Hermite Spline: 
// From https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Unit_interval_(0,_1)
// p(s) = (2s^3 - 3s^2 + 1)*p1 + (-2*s^3 + 3*s^2)*p2 + (s^3 - 2s^2 + s)*v1 + (s^3 - s^2)*v2
// where 0 <= s <= 1. 
double HermiteCurve::evaluate(const double & s_in){
  s_ = this->clamp(s_in);
  return p1*(2*std::pow(s_,3) - 3*std::pow(s_,2) + 1) + 
         p2*(-2*std::pow(s_,3) + 3*std::pow(s_,2))    + 
         v1*(std::pow(s_,3) - 2*std::pow(s_,2) + s_)  + 
         v2*(std::pow(s_,3) - std::pow(s_,2)); 
}

double HermiteCurve::evaluateFirstDerivative(const double & s_in){
  s_ = this->clamp(s_in);
  return p1 * (6 * std::pow(s_, 2) - 6 * s_) +
         p2 * (-6 * std::pow(s_, 2) + 6 * s_) +
         v1 * (3 * std::pow(s_, 2) - 4 * s_ + 1) +
         v2 * (3 * std::pow(s_, 2) - 2 * s_);
}

double HermiteCurve::evaluateSecondDerivative(const double & s_in){
  s_ = this->clamp(s_in);
  return p1*(12*s_ - 6)  + 
         p2*(-12*s_ + 6) +
         v1*(6*s_ - 4)  + 
         v2*(6*s_ - 2); 
}

double HermiteCurve::clamp(const double & s_in, double lo, double hi){
    if (s_in < lo){
        return lo;
    }
    else if(s_in > hi){
        return hi;
    }else{
        return s_in;
    }

}

