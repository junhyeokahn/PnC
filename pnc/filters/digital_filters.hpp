#pragma once

class DigitalFilter {
public:
  DigitalFilter(void);
  virtual ~DigitalFilter(void);
  virtual void Input(double input_value) = 0;
  virtual double Output(void) = 0;
  virtual void Clear(void) = 0;
};

class ButterWorthFilter : public DigitalFilter {
public:
  ButterWorthFilter(int num_sample, double dt, double cutoff_frequency);
  virtual ~ButterWorthFilter(void);
  virtual void Input(double input_value);
  virtual double Output(void);
  virtual void Clear(void);

private:
  double *mpBuffer;
  int mCurIdx;
  int mNumSample;
  double mDt;
  double mCutoffFreq;
  double mValue;
};

class LowPassFilter : public DigitalFilter {
public:
  LowPassFilter(double w_c, double t_s);
  virtual ~LowPassFilter(void);
  virtual void Input(double input_value);
  virtual double Output(void);
  virtual void Clear(void);

private:
  double Lpf_in_prev[2];
  double Lpf_out_prev[2];
  double Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
  double lpf_out;
};

class SimpleMovingAverage : public DigitalFilter {
public:
  SimpleMovingAverage(int num_data);
  virtual ~SimpleMovingAverage();
  virtual void Input(double input_value);
  virtual double Output(void);
  virtual void Clear(void);

private:
  double *buffer_;
  int num_data_;
  int idx_;
  double sum_;
};

class DerivativeLowPassFilter : public DigitalFilter {
public:
  DerivativeLowPassFilter(double w_c, double t_s);
  virtual ~DerivativeLowPassFilter(void);
  virtual void Input(double input_value);
  virtual double Output(void);
  virtual void Clear(void);

private:
  double Lpf_in_prev[2];
  double Lpf_out_prev[2];
  double Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
  double lpf_out;
};
