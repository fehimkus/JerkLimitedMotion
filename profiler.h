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
    
    bool newtonRaphson3(double t21, double J, double Vcurr,
                        double acurr, double c,
                        double tol = 1e-6, int max_iter = 100);
    
    void computeJacobian3(double J_mat[2][2],
                                double t21, double J, double Vcurr,
                                double acurr);
    void computeSystem3(double f[2],
                              double t21, double J, double Vcurr,
                              double acurr, double c);


    std::vector<double> newtonRaphson1(double t11, double t13, double t21,
                                       double a11, double v11, double Jerk, double c2,
                                       double t22_init, double t12_init, double tol = 1e-6,
                                       int max_iter = 100);
    
    std::vector<double> newtonRaphson2(double t11, double t13, double a11, double V11,
                                   double J, double c, double t21_initial,
                                   double t12_initial, double tol = 1e-6,
                                   int max_iter = 100);
                                   

    bool solveCubic(double a, double b, double c, double d,
                    double& root1, double& root2, double& root3);
    bool solveQuadratic(double a, double b, double c,
                        double& root1, double& root2);
    void solve2x2System(const double A[2][2], const double b[2], double x[2]);
    void computeSystem1(const double x[2], double f[2],
                        double t11, double t13, double t21,
                        double a11, double V11, double J, double c);
    void computeJacobian1(const double x[2], double J_mat[2][2],
                          double t11, double t21,
                          double a11, double V11, double J, double t13);
    void computeSystem2(const double x[2], double f[2],
                        double t11, double t13,
                        double a11, double V11, double J, double c);
    void computeJacobian2(const double x[2], double J[2][2],
                          double t11, double t13,
                          double a11, double V11, double J_param);

private:
    void CalculateShorterProfile();
};