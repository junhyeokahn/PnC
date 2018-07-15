#include <iostream>
#include "CentroidPlanner.hpp"
#include <memory>
#include "Configuration.h"
#include "Utilities.hpp"
#include <vector>

double m_curr_time(0.0), m_stab_time(0.5), m_high_freq(300.), m_low_freq(0.001);
double m_rate(0.015), m_offset(1.0), m_amp(1.0);
std::vector<double> m_save_signal;
std::vector<double> m_save_time;
std::vector<float> m_mirrored;
bool isSaved(false);

void chirpTest() {
    static float prev_sample_time = 0;
    static float prev_effective_angle = 0;

    double elapsed_time = m_curr_time - m_stab_time;
    float signal(0.0);
    float effective_angle, effective_switching_freq_hz;
    float range = m_high_freq - m_low_freq;
    float period = elapsed_time - prev_sample_time;
    prev_sample_time = elapsed_time;

    //exponential chirp
    effective_angle = 2 * M_PI * m_low_freq * (pow(m_rate * range, elapsed_time) - 1) / log(m_rate * range);
    //ROS_INFO("effective_angle:%f\t elapsed_time:%f", effective_angle, elapsed_time);

    signal = m_amp * sin(effective_angle) + m_offset;

    if(period <= 0.0)
    {
        effective_switching_freq_hz = 0.0;
    }
    else
    {
        effective_switching_freq_hz = (effective_angle - prev_effective_angle) / period / 2 / M_PI;
    }
    prev_effective_angle = effective_angle;

    if(effective_switching_freq_hz < m_high_freq) {
        m_mirrored.push_back(signal);
    }
    if(effective_switching_freq_hz > m_high_freq)
    {
        if (m_mirrored.size() > 0) {
            signal = m_mirrored.back();
            m_mirrored.pop_back();
        } else {
            signal = 0.0;
        }
    }

    double ramp_t(1.0);
    if (m_curr_time < ramp_t) {
        signal*= m_curr_time/ramp_t;
    }

    //m_save_signal.push_back(signal);
    //m_save_time.push_back(m_curr_time);
    //if(m_curr_time > 100.0) {
        //myUtils::saveVector(m_save_signal, "signal");
        //myUtils::saveVector(m_save_time, "time");
        //isSaved = true;
        //std::cout << "saved" << std::endl;
        //exit(0);
    //}
    if(effective_switching_freq_hz < m_high_freq) {
        m_save_signal.push_back(signal);
        m_save_time.push_back(m_curr_time);
    } else {
        if (!isSaved) {
            myUtils::saveVector(m_save_signal, "signal");
            myUtils::saveVector(m_save_time, "time");
            isSaved = true;
        }
    }
}

int main(int argc, char *argv[])
{
    //std::shared_ptr<CentroidPlannerParameter> mParam =
        //std::make_shared<CentroidPlannerParameter>();
    //mParam->paramSetFromYaml(THIS_COM"Config/Draco/CENTROID_PLANNER.yaml");

    //std::unique_ptr<CentroidPlanner> mPlanner =
        //std::make_unique<CentroidPlanner>();

    //mPlanner->updatePlanningParameter(mParam);
    //double time(1.0);
    //Eigen::VectorXd pos, vel, trq;
    //mPlanner->getPlan(time, pos, vel, trq);
    m_save_signal.reserve(5000);
    m_save_time.reserve(5000);
    while (true) {
        chirpTest();
        m_curr_time += 0.001;
    }
    std::cout << "Done" << std::endl;
    return 0;
}
