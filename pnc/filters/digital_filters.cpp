#include "pnc/filters/digital_filters.hpp"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "utils/util.hpp"

DigitalFilter::DigitalFilter() {}

DigitalFilter::~DigitalFilter() {}

SimpleMovingAverage::SimpleMovingAverage(int num_data)
    : DigitalFilter(), num_data_(num_data), idx_(0), sum_(0.0) {
  // buffer_ = new double[num_data_];
  // memset((void *)buffer_, 0.0, sizeof(double) * num_data_);
  buffer_ = Eigen::VectorXd::Zero(num_data_);
}

void SimpleMovingAverage::Input(double input_value) {
  sum_ -= buffer_[idx_];
  sum_ += input_value;
  buffer_[idx_] = input_value;
  ++idx_;
  idx_ %= num_data_;
}

double SimpleMovingAverage::Output() { return sum_ / num_data_; }

void SimpleMovingAverage::Clear() {
  sum_ = 0.0;
  // memset((void *)buffer_, 0.0, sizeof(double) * num_data_);
  buffer_ = Eigen::VectorXd::Zero(num_data_);
}

SimpleMovingAverage::~SimpleMovingAverage() {
  // delete[] buffer_;
}

ButterWorthFilter::ButterWorthFilter(int num_sample, double dt,
                                     double cutoff_frequency)
    : DigitalFilter() {
  mNumSample = num_sample;
  mDt = dt;
  mCutoffFreq = cutoff_frequency;

  mpBuffer = new double[num_sample];
  memset((void *)mpBuffer, 0, sizeof(double) * num_sample);

  mCurIdx = 0;
}

ButterWorthFilter::~ButterWorthFilter() { delete[] mpBuffer; }

void ButterWorthFilter::Input(double input_value) {
  int j;
  double sqrt_2 = sqrt(2);
  double value = 0;
  for (j = mNumSample - 2; j >= 0; j--) {
    mpBuffer[j + 1] = mpBuffer[j];
  }

  mpBuffer[0] = input_value;
  for (j = 0; j < mNumSample; j++) {
    double t = (double)j * mDt;
    value += sqrt_2 / mCutoffFreq * mpBuffer[j] * exp(-1. / sqrt_2 * t) *
             sin(mCutoffFreq / sqrt_2 * t) * mDt;
  }
  mValue = value;
}

double ButterWorthFilter::Output() { return mValue; }

void ButterWorthFilter::Clear() {
  for (int i(0); i < mNumSample; ++i) {
    mpBuffer[i] = 0.0;
  }
}

LowPassFilter::LowPassFilter(double w_c, double t_s) : DigitalFilter() {
  Lpf_in_prev[0] = Lpf_in_prev[1] = 0;
  Lpf_out_prev[0] = Lpf_out_prev[1] = 0;
  Lpf_in1 = 0, Lpf_in2 = 0, Lpf_in3 = 0, Lpf_out1 = 0, Lpf_out2 = 0;
  float den = 2500 * t_s * t_s * w_c * w_c + 7071 * t_s * w_c + 10000;

  Lpf_in1 = 2500 * t_s * t_s * w_c * w_c / den;
  Lpf_in2 = 5000 * t_s * t_s * w_c * w_c / den;
  Lpf_in3 = 2500 * t_s * t_s * w_c * w_c / den;
  Lpf_out1 = -(5000 * t_s * t_s * w_c * w_c - 20000) / den;
  Lpf_out2 = -(2500 * t_s * t_s * w_c * w_c - 7071 * t_s * w_c + 10000) / den;
}

LowPassFilter::~LowPassFilter() {}

void LowPassFilter::Input(double lpf_in) {
  lpf_out = Lpf_in1 * lpf_in + Lpf_in2 * Lpf_in_prev[0] +
            Lpf_in3 * Lpf_in_prev[1] + // input component
            Lpf_out1 * Lpf_out_prev[0] +
            Lpf_out2 * Lpf_out_prev[1]; // output component
  Lpf_in_prev[1] = Lpf_in_prev[0];
  Lpf_in_prev[0] = lpf_in;
  Lpf_out_prev[1] = Lpf_out_prev[0];
  Lpf_out_prev[0] = lpf_out;
}

double LowPassFilter::Output() { return lpf_out; }

void LowPassFilter::Clear() {
  Lpf_in_prev[1] = 0;
  Lpf_in_prev[0] = 0;
  Lpf_out_prev[1] = 0;
  Lpf_out_prev[0] = 0;
}

DerivativeLowPassFilter::DerivativeLowPassFilter(double w_c, double t_s) {
  Lpf_in_prev[0] = 0;
  Lpf_in_prev[1] = 0;
  Lpf_out_prev[0] = 0;
  Lpf_out_prev[1] = 0;
  Lpf_in1 = 0;
  Lpf_in2 = 0;
  Lpf_in3 = 0;
  Lpf_out1 = 0;
  Lpf_out2 = 0;
  double a = 1.4142;
  double den = 4 + 2 * a * w_c * t_s + t_s * t_s * w_c * w_c;

  Lpf_in1 = 2 * t_s * w_c * w_c / den;
  Lpf_in2 = 0;
  Lpf_in3 = -2. * t_s * w_c * w_c / den;
  Lpf_out1 = -1. * (-8 + t_s * t_s * w_c * w_c * 2) / den;
  Lpf_out2 = -1. * (4 - 2 * a * w_c * t_s + t_s * t_s * w_c * w_c) / den;
  lpf_out = 0.0;
  Clear();
}

DerivativeLowPassFilter::~DerivativeLowPassFilter() {}

void DerivativeLowPassFilter::Input(double lpf_in) {
  lpf_out = Lpf_in1 * lpf_in + Lpf_in2 * Lpf_in_prev[0] +
            Lpf_in3 * Lpf_in_prev[1] + // input component
            Lpf_out1 * Lpf_out_prev[0] +
            Lpf_out2 * Lpf_out_prev[1]; // output component

  Lpf_in_prev[1] = Lpf_in_prev[0];
  Lpf_in_prev[0] = lpf_in;
  Lpf_out_prev[1] = Lpf_out_prev[0];
  Lpf_out_prev[0] = lpf_out;
}

double DerivativeLowPassFilter::Output() { return lpf_out; }

void DerivativeLowPassFilter::Clear() {
  Lpf_in_prev[1] = 0;
  Lpf_in_prev[0] = 0;
  Lpf_out_prev[1] = 0;
  Lpf_out_prev[0] = 0;
}

ExponentialMovingAverageFilter::ExponentialMovingAverageFilter(
    double dt, double time_constant, Eigen::VectorXd init_value,
    Eigen::VectorXd min_crop, Eigen::VectorXd max_crop) {
  dt_ = dt;
  init_value_ = init_value;
  double T =
      std::max(time_constant, 2 * dt_); // Nyquist-Shannon sampling theorem
  time_constant_ = T;
  alpha_ = 1. - std::exp(-dt_ / T);
  min_crop_ = min_crop;
  max_crop_ = max_crop;

  average_ = init_value_;
}

ExponentialMovingAverageFilter::~ExponentialMovingAverageFilter() {}

void ExponentialMovingAverageFilter::Input(Eigen::VectorXd input_value) {
  average_ += alpha_ * (input_value - average_);
  raw_value_ = input_value;

  for (int i = 0; i < average_.size(); ++i) {
    if (average_[i] < min_crop_[i])
      average_[i] = min_crop_[i];
    if (average_[i] > max_crop_[i])
      average_[i] = max_crop_[i];
  }
}

Eigen::VectorXd ExponentialMovingAverageFilter::Output() { return average_; }

void ExponentialMovingAverageFilter::Clear() { average_ = init_value_; }
