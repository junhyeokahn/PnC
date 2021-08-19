#pragma once

#include <Eigen/Dense>

/// class DigitalFilter
class DigitalFilter {
public:
  /// \{ \name Constructor and Destructor
  DigitalFilter();

  virtual ~DigitalFilter();
  /// \}

  /// Input to this filter
  virtual void Input(double input_value) = 0;

  /// Output of this filter
  virtual double Output() = 0;

  /// Reset data history in this filter
  virtual void Clear() = 0;
};

/// class ButterWorthFilter
class ButterWorthFilter : public DigitalFilter {
public:
  ButterWorthFilter(int num_sample, double dt, double cutoff_frequency);
  virtual ~ButterWorthFilter();
  virtual void Input(double input_value);
  virtual double Output();
  virtual void Clear();

private:
  double *mpBuffer;
  int mCurIdx;
  int mNumSample;
  double mDt;
  double mCutoffFreq;
  double mValue;
};

/// class LowPassFilter
class LowPassFilter : public DigitalFilter {
public:
  LowPassFilter(double w_c, double t_s);
  virtual ~LowPassFilter();
  virtual void Input(double input_value);
  virtual double Output();
  virtual void Clear();

private:
  double Lpf_in_prev[2];
  double Lpf_out_prev[2];
  double Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
  double lpf_out;
};

/// class SimpleMovingAverage
class SimpleMovingAverage : public DigitalFilter {
public:
  SimpleMovingAverage(int num_data);
  virtual ~SimpleMovingAverage();
  virtual void Input(double input_value);
  virtual double Output();
  virtual void Clear();

private:
  Eigen::VectorXd buffer_;
  int num_data_;
  int idx_;
  double sum_;
};

/// class DerivativeLowPassFilter
class DerivativeLowPassFilter : public DigitalFilter {
public:
  DerivativeLowPassFilter(double w_c, double t_s);
  virtual ~DerivativeLowPassFilter();
  virtual void Input(double input_value);
  virtual double Output();
  virtual void Clear();

private:
  double Lpf_in_prev[2];
  double Lpf_out_prev[2];
  double Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
  double lpf_out;
};
