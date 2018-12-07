#pragma once

#include <chrono>

class Clock {
    public:
        Clock(){}
        ~Clock(){}

        void start() { ini_time_ = std::chrono::high_resolution_clock::now(); }
        double stop() {
            end_time_ = std::chrono::high_resolution_clock::now();
            duration_ = std::chrono::duration_cast<std::chrono::microseconds>(end_time_-ini_time_);
            return double(duration_.count())*1e-3;
            //return double(duration_.count());
        }

    private:
        std::chrono::microseconds duration_;
        std::chrono::high_resolution_clock::time_point ini_time_, end_time_;
};
