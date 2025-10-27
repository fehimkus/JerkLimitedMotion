#include <unistd.h>
#include <csignal>
#include <cmath>
#include <iostream>
#include <vector>
#include <chrono>


#pragma once


class Profiler
{
public:
    Profiler()
    {
    }

    ~Profiler()
    {
    }
    unsigned int stage{0};
    std::chrono::time_point<std::chrono::high_resolution_clock> begin;
    double t11{0.0};
    double t12{0.0};
    double t13{0.0};
    double t21{0.0};
    double t22{0.0};
    double t23{0.0};
    double Jerk{0.0};
    double MaxVelocity{0.0};
    double MaxAcceleration{0.0};
    double MaxDeceleration{0.0};
    double CurrentPosition{0.0};
    double CurrentVelocity{0.0};
    double CurrentAcceleration{0.0};
    double TargetPosition{0.0};
    double TargetDistance{0.0};
    double direction{1};
    double x11{0.0};
    double x12{0.0};
    double x13{0.0};
    double x21{0.0};
    double x22{0.0};
    double x23{0.0};
    double v11{0.0};
    double v12{0.0};
    double v13{0.0};
    double v21{0.0};
    double v22{0.0};
    double v23{0.0};
    double a11{0.0};
    double a12{0.0};
    double a13{0.0};
    double a21{0.0};
    double a22{0.0};
    double a23{0.0};

    void CalculatePositionProfile();

private:
    void CalculateShorterProfile();
};