#ifndef FILTERS_
#define FILTERS_

class filter
{
public:
	filter(void);
	virtual ~filter(void);
	virtual void input(double input_value) = 0;
	virtual double output(void) = 0;
	virtual void clear(void) = 0;
};

class butterworth_filter : public filter
{
public:
	butterworth_filter(int num_sample, double dt, double cutoff_frequency);
	virtual ~butterworth_filter(void);
	virtual void input(double input_value);
	virtual double output(void);
	virtual void clear(void);
private:
	double *mpBuffer;
	int mCurIdx;
	int mNumSample;
	double mDt;
	double mCutoffFreq;
	double mValue;
};

class digital_lp_filter : public filter
{
public:
	digital_lp_filter(double w_c, double t_s);
	virtual ~digital_lp_filter(void);
	virtual void input(double input_value);
	virtual double output(void);
	virtual void clear(void);
private:
	double Lpf_in_prev[2];
	double Lpf_out_prev[2];
	double Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
	double lpf_out;
};

class moving_average_filter : public filter
{
public:
    moving_average_filter(int num_data);
    virtual ~moving_average_filter();
    virtual void input(double input_value);
    virtual double output(void);
    virtual void clear(void);
private:
    double * buffer_;
    int num_data_;
    int idx_;
    double sum_;
};

class deriv_lp_filter : public filter
{
public:
	deriv_lp_filter(double w_c, double t_s);
	virtual ~deriv_lp_filter(void);
	virtual void input(double input_value);
	virtual double output(void);
	virtual void clear(void);
private:
	double Lpf_in_prev[2];
	double Lpf_out_prev[2];
	double Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
	double lpf_out;
};

class ff01_filter : public filter
{
public:
	ff01_filter(float t_s, float w_c);
	virtual ~ff01_filter(void);
	virtual void input(double input_value);
	virtual double output(void);
	virtual void clear(void);
private:
	double Lpf_in_prev[2];
	double Lpf_out_prev[2];
	double Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
	double lpf_out;
};

class ff02_filter : public filter
{
public:
	ff02_filter(float t_s, float w_c);
	virtual ~ff02_filter(void);
	virtual void input(double input_value);
	virtual double output(void);
	virtual void clear(void);
private:
	double Lpf_in_prev[2];
	double Lpf_out_prev[2];
	double Lpf_in1, Lpf_in2, Lpf_in3, Lpf_out1, Lpf_out2;
	double lpf_out;
};


class CFilterButterworth24db
{
public:
    CFilterButterworth24db(void);
    virtual ~CFilterButterworth24db(void);
    void SetSampleRate(float fs);
    void Set(float cutoff, float q);
    float Run(float input);

private:
    float t0, t1, t2, t3;
    float coef0, coef1, coef2, coef3;
    float history1, history2, history3, history4;
    float gain;
    float min_cutoff, max_cutoff;
};


class AverageFilter: public filter
{
    public:
        AverageFilter(double dt, double t_const, double limit);
        virtual ~AverageFilter();
        virtual void input(double input_value);
        virtual double output(void);
        virtual void clear(void);
    private:
        double est_value_;
        double dt_;
        double t_const_;
        double limit_;
};
#endif
