#include <math.h>
#include <string.h>
#include "filters.hpp"
#include <stdio.h>
#include <stdlib.h>

#define BUDDA_Q_SCALE 6.f


filter::filter(void)
{
}

filter::~filter(void)
{
}

moving_average_filter::moving_average_filter(int num_data): num_data_(num_data),
                                                            idx_(0),
                                                            sum_(0.0){
    buffer_ = new double [num_data_];
    memset((void *)buffer_, 0.0, sizeof(double)*num_data_);
}
void moving_average_filter::input(double input_value){
    sum_ -= buffer_[idx_];
    sum_ += input_value;
    buffer_[idx_] = input_value;
    ++idx_;
    idx_%=num_data_;
}
double moving_average_filter::output(){
    return sum_/num_data_;
}
void moving_average_filter::clear(void){
    sum_ = 0.0;
    memset((void *)buffer_, 0.0, sizeof(double)*num_data_);
}

moving_average_filter::~moving_average_filter(){
    delete [] buffer_;
}

butterworth_filter::butterworth_filter(int num_sample, double dt, double cutoff_frequency)
{
    mNumSample	= num_sample;
    mDt			= dt;
    mCutoffFreq	= cutoff_frequency;

    mpBuffer	= new double[num_sample];
    memset((void *)mpBuffer, 0, sizeof(double)*num_sample);

    mCurIdx		= 0;
}

butterworth_filter::~butterworth_filter(void)
{
    delete[] mpBuffer;
}

void butterworth_filter::input(double input_value)
{
    int j;
    double sqrt_2 = sqrt(2);
    double value = 0;
    for ( j = mNumSample-2 ; j >= 0 ; j-- )
    {
        mpBuffer[j+1] = mpBuffer[j];
    }

    mpBuffer[0] = input_value;
    for ( j = 0 ; j < mNumSample ; j++ )
    {
        double t = (double)j*mDt;
        value += sqrt_2 / mCutoffFreq * mpBuffer[j] * exp(-1./sqrt_2*t) * sin(mCutoffFreq/sqrt_2*t ) * mDt;
//		value += sqrt_2 * exp(-1./sqrt_2*t) * sin(1./sqrt_2*t ) * mDt;
    }
    mValue = value;
}

double butterworth_filter::output(void)
{
    return mValue;
}

void butterworth_filter::clear(void)
{
    for(int i(0); i<mNumSample; ++i){
        mpBuffer[i] = 0.0;
    }
}

digital_lp_filter::digital_lp_filter(double w_c, double t_s)
{
    Lpf_in_prev[0] = Lpf_in_prev[1] = 0;
    Lpf_out_prev[0] = Lpf_out_prev[1] = 0;
    Lpf_in1=0, Lpf_in2=0, Lpf_in3=0, Lpf_out1=0, Lpf_out2=0;
    float den = 2500*t_s*t_s*w_c*w_c  + 7071*t_s*w_c + 10000;

    Lpf_in1 = 2500*t_s*t_s*w_c*w_c / den;
    Lpf_in2 = 5000*t_s*t_s*w_c*w_c / den;
    Lpf_in3 = 2500*t_s*t_s*w_c*w_c / den;
    Lpf_out1 = -(5000*t_s*t_s*w_c*w_c  - 20000) / den;
    Lpf_out2 = -(2500*t_s*t_s*w_c*w_c  - 7071*t_s*w_c + 10000) / den;

}

digital_lp_filter::~digital_lp_filter(void)
{
}

void digital_lp_filter::input(double lpf_in)
{
    lpf_out = Lpf_in1*lpf_in + Lpf_in2*Lpf_in_prev[0] + Lpf_in3*Lpf_in_prev[1] + //input component
	Lpf_out1*Lpf_out_prev[0] + Lpf_out2*Lpf_out_prev[1]; //output component
    Lpf_in_prev[1] = Lpf_in_prev[0];
    Lpf_in_prev[0] = lpf_in;
    Lpf_out_prev[1] = Lpf_out_prev[0];
    Lpf_out_prev[0] = lpf_out;
}

double digital_lp_filter::output(void)
{
    return lpf_out;
}

void digital_lp_filter::clear(void)
{
    Lpf_in_prev[1] = 0;
    Lpf_in_prev[0] = 0;
    Lpf_out_prev[1] = 0;
    Lpf_out_prev[0] = 0;
}


deriv_lp_filter::deriv_lp_filter(double w_c, double t_s)
{
    Lpf_in_prev[0] = 0;
    Lpf_in_prev[1] = 0;
    Lpf_out_prev[0] = 0;
    Lpf_out_prev[1] = 0;
    Lpf_in1=0;
    Lpf_in2=0;
    Lpf_in3=0;
    Lpf_out1=0;
    Lpf_out2=0;
    double a = 1.4142;
    double den = 4 + 2*a*w_c*t_s + t_s*t_s*w_c*w_c;

    Lpf_in1 = 2*t_s*w_c*w_c / den;
    Lpf_in2 = 0;
    Lpf_in3 = -2.*t_s*w_c*w_c / den;
    Lpf_out1 = -1. *(-8 + t_s*t_s*w_c*w_c*2) / den; 
    Lpf_out2 = -1. *(4 - 2*a * w_c*t_s + t_s*t_s*w_c*w_c) / den;
    lpf_out = 0.0;
    clear();
}

deriv_lp_filter::~deriv_lp_filter(void)
{
}

void deriv_lp_filter::input(double lpf_in)
{
    // static int i(0);
    lpf_out = Lpf_in1*lpf_in + Lpf_in2*Lpf_in_prev[0] + Lpf_in3*Lpf_in_prev[1] + //input component
        Lpf_out1*Lpf_out_prev[0] + Lpf_out2*Lpf_out_prev[1]; //output component

    // printf("%i th filter (%f): %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",i,
    //        lpf_out,
    //        Lpf_in1, lpf_in, Lpf_in2,
    //        Lpf_in_prev[0], Lpf_in3, Lpf_in_prev[1],
    //        Lpf_out1, Lpf_out_prev[0], Lpf_out2, Lpf_out_prev[1]);
    
    // if(lpf_out>100){
    //     exit(0);
    // }
        
    Lpf_in_prev[1] = Lpf_in_prev[0];
    Lpf_in_prev[0] = lpf_in;
    Lpf_out_prev[1] = Lpf_out_prev[0];
    Lpf_out_prev[0] = lpf_out;
    // ++i;
}

double deriv_lp_filter::output(void)
{
    return lpf_out;
}

void deriv_lp_filter::clear(void)
{
    Lpf_in_prev[1] = 0;
    Lpf_in_prev[0] = 0;
    Lpf_out_prev[1] = 0;
    Lpf_out_prev[0] = 0;
}

ff01_filter::ff01_filter(float t_s, float w_c)
{
    Lpf_in_prev[0] = Lpf_in_prev[1] = 0;
    Lpf_out_prev[0] = Lpf_out_prev[1] = 0;
    Lpf_in1=0, Lpf_in2=0, Lpf_in3=0, Lpf_out1=0, Lpf_out2=0;
    double a = 1.4142;
    double den = 4 + 2*a*w_c*t_s + t_s*t_s*w_c*w_c;
    double J = 0.00008;
    double B = 0.0002;

    Lpf_in1 = B*t_s*t_s*w_c*w_c + 2*J*t_s*w_c*w_c;
    Lpf_in2 = 2*B*t_s*t_s*w_c*w_c;
    Lpf_in3 = B*t_s*t_s*w_c*w_c - 2 * J * t_s * w_c * w_c;
    Lpf_out1 = -1. *(-8 + t_s*t_s*w_c*w_c*2) / den; 
    Lpf_out2 = -1. *(4 - 2*a * w_c*t_s + t_s*t_s*w_c*w_c) / den;
}

ff01_filter::~ff01_filter(void)
{
}

void ff01_filter::input(double lpf_in)
{
    lpf_out = Lpf_in1*lpf_in + Lpf_in2*Lpf_in_prev[0] + Lpf_in3*Lpf_in_prev[1] + //input component
        Lpf_out1*Lpf_out_prev[0] + Lpf_out2*Lpf_out_prev[1]; //output component
    Lpf_in_prev[1] = Lpf_in_prev[0];
    Lpf_in_prev[0] = lpf_in;
    Lpf_out_prev[1] = Lpf_out_prev[0];
    Lpf_out_prev[0] = lpf_out;
}

double ff01_filter::output(void)
{
    return lpf_out;
}

void ff01_filter::clear(void)
{
    Lpf_in_prev[1] = 0;
    Lpf_in_prev[0] = 0;
    Lpf_out_prev[1] = 0;
    Lpf_out_prev[0] = 0;
}

ff02_filter::ff02_filter(float t_s, float w_c)
{
    double J = 0.003216;

    Lpf_in_prev[0] = Lpf_in_prev[1] = 0;
    Lpf_out_prev[0] = Lpf_out_prev[1] = 0;
    Lpf_in1=0, Lpf_in2=0, Lpf_in3=0, Lpf_out1=0, Lpf_out2=0;

    double a = 1.4142;
    double den = 4 + 2*a*w_c*t_s + t_s*t_s*w_c*w_c;

    Lpf_in1 = J*2*t_s*w_c*w_c / den;
    Lpf_in2 = 0;
    Lpf_in3 = -2.*J*t_s*w_c*w_c / den;
    Lpf_out1 = -1. *(-8 + t_s*t_s*w_c*w_c*2) / den; 
    Lpf_out2 = -1. *(4 - 2*a * w_c*t_s + t_s*t_s*w_c*w_c) / den;

    clear();

}

ff02_filter::~ff02_filter(void)
{
}

void ff02_filter::input(double lpf_in)
{
    lpf_out = Lpf_in1*lpf_in + Lpf_in2*Lpf_in_prev[0] + Lpf_in3*Lpf_in_prev[1] + //input component
        Lpf_out1*Lpf_out_prev[0] + Lpf_out2*Lpf_out_prev[1]; //output component
    Lpf_in_prev[0] = lpf_in;
    Lpf_in_prev[1] = Lpf_in_prev[0];
    Lpf_out_prev[0] = lpf_out;
    Lpf_out_prev[1] = Lpf_out_prev[0];
}

double ff02_filter::output(void)
{
    return lpf_out;
}

void ff02_filter::clear(void)
{
    Lpf_in_prev[1] = 0;
    Lpf_in_prev[0] = 0;
    Lpf_out_prev[1] = 0;
    Lpf_out_prev[0] = 0;
}


////////////////////////////////////////////
///// FilterButterworth24db
////////////////////////////////////////////

CFilterButterworth24db::CFilterButterworth24db(void)
{
    this->history1 = 0.f;
    this->history2 = 0.f;
    this->history3 = 0.f;
    this->history4 = 0.f;

    this->SetSampleRate(44100.f);
    this->Set(22050.f, 0.0);
}

CFilterButterworth24db::~CFilterButterworth24db(void)
{
}

void CFilterButterworth24db::SetSampleRate(float fs)
{
    float pi = 4.f * atanf(1.f);

    this->t0 = 4.f * fs * fs;
    this->t1 = 8.f * fs * fs;
    this->t2 = 2.f * fs;
    this->t3 = pi / fs;

    this->min_cutoff = fs * 0.01f;
    this->max_cutoff = fs * 0.45f;
}

void CFilterButterworth24db::Set(float cutoff, float q)
{
    if (cutoff < this->min_cutoff)
        cutoff = this->min_cutoff;
    else if(cutoff > this->max_cutoff)
        cutoff = this->max_cutoff;

    if(q < 0.f)
        q = 0.f;
    else if(q > 1.f)
        q = 1.f;

    float wp = this->t2 * tanf(this->t3 * cutoff);
    float bd, bd_tmp, b1, b2;

    q *= BUDDA_Q_SCALE;
    q += 1.f;

    b1 = (0.765367f / q) / wp;
    b2 = 1.f / (wp * wp);

    bd_tmp = this->t0 * b2 + 1.f;

    bd = 1.f / (bd_tmp + this->t2 * b1);

    this->gain = bd * 0.5f;

    this->coef2 = (2.f - this->t1 * b2);

    this->coef0 = this->coef2 * bd;
    this->coef1 = (bd_tmp - this->t2 * b1) * bd;

    b1 = (1.847759f / q) / wp;

    bd = 1.f / (bd_tmp + this->t2 * b1);

    this->gain *= bd;
    this->coef2 *= bd;
    this->coef3 = (bd_tmp - this->t2 * b1) * bd;
}

float CFilterButterworth24db::Run(float input)
{
    float output = input * this->gain;
    float new_hist;

    output -= this->history1 * this->coef0;
    new_hist = output - this->history2 * this->coef1;

    output = new_hist + this->history1 * 2.f;
    output += this->history2;

    this->history2 = this->history1;
    this->history1 = new_hist;

    output -= this->history3 * this->coef2;
    new_hist = output - this->history4 * this->coef3;

    output = new_hist + this->history3 * 2.f;
    output += this->history4;

    this->history4 = this->history3;
    this->history3 = new_hist;

    return output;
}
AverageFilter::AverageFilter(double dt, double t_const, double limit):
    dt_(dt), t_const_(t_const), limit_(limit)
{
    est_value_ = 0.;

}

AverageFilter::~AverageFilter(){ est_value_ = 0; }
void AverageFilter::clear(){ est_value_ = 0.; }
void AverageFilter::input(double input){
    double update_value = input - est_value_;
    if(fabs(update_value) > limit_){ update_value = 0.; }
    est_value_ += (dt_/(dt_ + t_const_))*update_value;
}
double AverageFilter::output(){ return est_value_; }
