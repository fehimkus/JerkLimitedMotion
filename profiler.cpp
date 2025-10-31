#include "profiler.h"

bool Profiler::solveQuadratic(double a, double b, double c, double &x1, double &x2)
{
    double discriminant = b * b - 4 * a * c;

    if (discriminant < 0)
    {
        return false;
    }

    double sqrt_discriminant = std::sqrt(discriminant);
    double denom = 2 * a;

    x1 = (-b + sqrt_discriminant) / denom;
    x2 = (-b - sqrt_discriminant) / denom;

    return true;
}

bool Profiler::solve2x2System(const double A[2][2], const double b[2], double x[2])
{
    const double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];

    // //std::cout << "Solving 2x2 system, det = " << det << std::endl;
    
    if (std::abs(det) < 1e-12) {
        // //std::cout << "Warning: Near-singular matrix in solve2x2System" << std::endl;
        x[0] = x[1] = 0.0;
        return false; // Return false for singular matrix
    }

    const double inv_det = 1.0 / det;
    x[0] = (A[1][1] * b[0] - A[0][1] * b[1]) * inv_det;
    x[1] = (A[0][0] * b[1] - A[1][0] * b[0]) * inv_det;
    
    //std::cout << "Solution: x = [" << x[0] << ", " << x[1] << "]" << std::endl;
    
    return true; // Return true for successful solution
}

void Profiler::computeSystem1(double f[2], double c)
{
    // Precompute squares for better performance and readability
    const double t21_sq = t21 * t21;
    const double t11_sq = t11 * t11;
    const double t13_sq = t13 * t13;
    const double t12_sq = t12 * t12;

    // First equation
    f[0] = (t21_sq + t21 * t22) - (0.5 * t11_sq + t11 * t12 + t11 * t13 - 0.5 * t13_sq);

    // Second equation  
    f[1] = (v11 * t12 + 0.5 * a11 * t12_sq + a11 * t12 * t13 +
            1.5 * Jerk * t21_sq * t22 + 0.5 * Jerk * t21 * t22 * t22) - c;
}

void Profiler::computeJacobian1(double J_mat[2][2])
{
    // CORRECTED: Proper partial derivatives
    // For f1 = t21² + t21*t22 - (0.5*t11² + t11*t12 + t11*t13 - 0.5*t13²)
    
    // ∂f1/∂t21 = 2*t21 + t22
    J_mat[0][0] = 2.0 * t21 + t22;
    
    // ∂f1/∂t22 = t21  
    J_mat[0][1] = t21;
    
    // For f2 = v11*t12 + 0.5*a11*t12² + a11*t12*t13 + 1.5*J*t21²*t22 + 0.5*J*t21*t22² - c
    
    // ∂f2/∂t21 = 3.0 * Jerk * t21 * t22 + 0.5 * Jerk * t22 * t22
    J_mat[1][0] = 3.0 * Jerk * t21 * t22 + 0.5 * Jerk * t22 * t22;
    
    // ∂f2/∂t22 = 1.5 * Jerk * t21 * t21 + Jerk * t21 * t22
    J_mat[1][1] = 1.5 * Jerk * t21 * t21 + Jerk * t21 * t22;
}

bool Profiler::newtonRaphson1(double c, double tol, int max_iter)
{
    double f[2];
    double J_mat[2][2];
    double dx[2];
    double minus_f[2];

    // Store initial values for debugging
    const double initial_t21 = t21;
    const double initial_t22 = t22;
    
    //std::cout << "Newton-Raphson started with: t21=" << t21 << ", t22=" << t22 << std::endl;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        computeSystem1(f, c);

        // Check for convergence using squared L2 norm
        const double norm_sq = f[0] * f[0] + f[1] * f[1];
        //std::cout << "Iteration " << iter << ": f = [" << f[0] << ", " << f[1] 
        //          << "], norm_sq = " << norm_sq 
        //          << ", t21=" << t21 << ", t22=" << t22 << std::endl;
        
        if (norm_sq < tol * tol)
        {
            //std::cout << "Converged after " << iter << " iterations" << std::endl;
            return true;
        }

        // Compute Jacobian
        computeJacobian1(J_mat);
        
        //std::cout << "Jacobian: [[" << J_mat[0][0] << ", " << J_mat[0][1] 
        //          << "], [" << J_mat[1][0] << ", " << J_mat[1][1] << "]]" << std::endl;

        // Solve J*dx = -f
        minus_f[0] = -f[0];
        minus_f[1] = -f[1];
        
        if (!solve2x2System(J_mat, minus_f, dx)) {
            //std::cout << "Linear system solve failed - singular matrix" << std::endl;
            return false;
        }

        //std::cout << "Update: dx = [" << dx[0] << ", " << dx[1] << "]" << std::endl;

        // CORRECTED: Update both variables
        t21 += dx[0];  // You were missing this!
        t22 += dx[1];

        // Add bounds checking
        if (t21 < 0.0) t21 = 0.0;
        if (t22 < 0.0) t22 = 0.0;
        
        // Check for NaN or infinity
        if (!std::isfinite(t21) || !std::isfinite(t22)) {
            //std::cout << "Numerical instability detected" << std::endl;
            return false;
        }
    }
    
    //std::cout << "Failed to converge after " << max_iter << " iterations" << std::endl;
    return false;
}

// Compute the Jacobian matrix (optimized)

// Compute the system of equations (optimized)
void Profiler::computeSystem2(double f[2], double c)
{
    // const double t21 = x[0];
    // const double t12 = x[1];

    // first equation
    f[0] = (t21 * t21) - ((t11 * t11 * 0.5) + (t11 * t12) + (t11 * t13) - (0.5 * t13 * t13));

    // second equation
    f[1] = (v11 * t12) + (0.5 * a11 * t12 * t12) + (a11 * t12 * t13) + (Jerk * t21 * t21 * t21) - c;
}

// Solve 2x2 linear system directly (optimized for small system)
void Profiler::computeJacobian2(double J[2][2])
{
    // const double t21 = x[0];
    // const double t12 = x[1];

    // Partial derivatives for first equation
    J[0][0] = 2.0f;  // df1/dt21
    J[0][1] = -1.0f; // df1/dt12

    // Partial derivatives for second equation
    J[1][0] = 3.0f * Jerk * t21 * t21;         // df2/dt21
    J[1][1] = 2.0f * a11 * t12 + v11 + a11 * t13; // df2/dt12
}

// Optimized Newton-Raphson method for 2D system
bool Profiler::newtonRaphson2(double c, double tol,
                                   int max_iter)
{
    double f[2];
    double J_mat[2][2];
    double dx[2];
    double minus_f[2];

    for (int iter = 0; iter < max_iter; ++iter)
    {
        computeSystem2(f, c);

        // Check for convergence using L2 norm
        const double norm = f[0] * f[0] + f[1] * f[1];
        if (norm < tol * tol)
        { // Compare squared norm to squared tolerance
            return true;
        }

        // Compute Jacobian
        computeJacobian2(J_mat);

        // Solve J*dx = -f
        minus_f[0] = -f[0];
        minus_f[1] = -f[1];
        // solve2x2System(J_mat, minus_f, dx);
        if (!solve2x2System(J_mat, minus_f, dx)) {
            //std::cout << "Linear system solve failed - singular matrix" << std::endl;
            return false;
        }
        // Update solution
        t21 += dx[0];
        t12 += dx[1];
    }
    return false;

    // return {x[0], x[1]};
}



void Profiler::computeSystem3(double f[2], double c)
{
    double t13 = t11 + (CurrentAcceleration / Jerk);

    // First equation:
    // J*t21^2 + J*t21*t22 = Vcurr + acurr*t11 + 0.5*J*t11^2 + acurr*t13 + 0.5*J*t13^2 + 0.5*acurr^2/J
    f[0] = (Jerk * t21 * t21 + Jerk * t21 * t22)
         - (CurrentVelocity + CurrentAcceleration * t11 + 0.5 * Jerk * t11 * t11
            + CurrentAcceleration * t13 + 0.5 * Jerk * t13 * t13
            + 0.5 * (CurrentAcceleration * CurrentAcceleration / Jerk));
    
    // Second equation:
    // c = (acurr^2*t11/J) + 2.5*acurr*t11^2 + Vcurr*t11 + 1.33334*J*t11^3 +
    //      1.5*J*t21^2*t22 + 0.5*J*t21*t22^2
    f[1] = (CurrentAcceleration * CurrentAcceleration * t11 / Jerk)
         + 2.5 * CurrentAcceleration * t11 * t11
         + CurrentVelocity * t11
         + (4.0/3.0) * Jerk * t11 * t11 * t11  // 1.33334 yerine 4/3
         + 1.5 * Jerk * t21 * t21 * t22
         + 0.5 * Jerk * t21 * t22 * t22
         - c;
}

void Profiler::computeJacobian3(double J_mat[2][2])
{
    // ----------- df1/dt11 -----------
    // f1 = J*t21^2 + J*t21*t22 - [Vcurr + acurr*t11 + 0.5*J*t11^2 + acurr*t13 + 0.5*J*t13^2 + 0.5*acurr^2/J]
    // t13 = t11 + acurr/J => dt13/dt11 = 1
    // df1/dt11 = -acurr - J*t11 - acurr - J*t13
    //          = -2*acurr - J*t11 - J*(t11 + acurr/J)
    //          = -2*acurr - J*t11 - J*t11 - acurr
    //          = -3*acurr - 2*J*t11
    double t13 = t11 + (CurrentAcceleration / Jerk);
    J_mat[0][0] = -CurrentAcceleration - Jerk * t11 - CurrentAcceleration - Jerk * t13;
    
    // ----------- df1/dt22 -----------
    // f1 = J*t21^2 + J*t21*t22 - [...]
    // df1/dt22 = J*t21
    J_mat[0][1] = Jerk * t21;

    // ----------- df2/dt11 -----------
    // f2 = (acurr^2*t11/J) + 2.5*acurr*t11^2 + Vcurr*t11 + (4/3)*J*t11^3 + ... - c
    // df2/dt11 = acurr^2/J + 5*acurr*t11 + Vcurr + 4*J*t11^2
    J_mat[1][0] = (CurrentAcceleration * CurrentAcceleration / Jerk)
                + 5.0 * CurrentAcceleration * t11
                + CurrentVelocity
                + 4.0 * Jerk * t11 * t11;

    // ----------- df2/dt22 -----------
    // f2 = ... + 1.5*J*t21^2*t22 + 0.5*J*t21*t22^2 + ... - c
    // df2/dt22 = 1.5*J*t21^2 + J*t21*t22
    J_mat[1][1] = 1.5 * Jerk * t21 * t21 + Jerk * t21 * t22;
}

bool Profiler::newtonRaphson3(double c,
                              double tol, int max_iter)
{
    double f[2], J_mat[2][2], dx[2];

    for (int iter = 0; iter < max_iter; ++iter)
    {
        computeSystem3(f, c);

        if (f[0] * f[0] + f[1] * f[1] < tol * tol)
            return true;

        computeJacobian3(J_mat);

        double minus_f[2] = {-f[0], -f[1]};
        // solve2x2System(J_mat, minus_f, dx);
        if (!solve2x2System(J_mat, minus_f, dx)) {
            //std::cout << "Linear system solve failed - singular matrix" << std::endl;
            return false;
        }
        // x[0] ve x[1] yerine artık direkt class member güncelleniyor
        t11 += dx[0];
        t22 += dx[1];

        if (t11 < 0.0) t11 = 0.0;
        if (t22 < 0.0) t22 = 0.0;
    }

    return false;
}


void Profiler::CalculateShorterProfile()
{

    //std::cout << "Target position is NOT enough to reach max velocity and max acceleration" << std::endl;
    //---------------------rising without t2
    if (t21 > t11)
    {
        //std::cout << " --------------CASE 1: t21 > t11 ------------------" << std::endl;
        double need_distance = (Jerk * std::pow(t21, 3));

        double x11_modified_t2 = (0.1666667 * Jerk * std::pow(t11, 3)) +
                                 (0.5 * CurrentAcceleration * std::pow(t11, 2)) +
                                 (CurrentVelocity * t11);

        double v11_modified_t2 = (CurrentVelocity +
                                  (CurrentAcceleration * t11) +
                                  (0.5 * Jerk * std::pow(t11, 2)));
        double a11_modified_t2 = (CurrentAcceleration +
                                  (Jerk * t11));

        if (t12 > 0.0001)
        {
            double t12_1, t12_2;
            double c = (0.5 * a11_modified_t2 * std::pow(t13, 2) +
                        (0.1666667 * Jerk * std::pow(t13, 3)) - need_distance);
    
            bool solved = solveQuadratic(a11_modified_t2 * 0.5, 
                (v11_modified_t2 + (a11_modified_t2 * t13)), c, t12_1, t12_2);
    
            if (solved)
            {
                t12 = std::max(t12_1, t12_2);
            }
            else
            {
                t12 = 0.0;
                return; // Error handling for no real solution
            }
        }
        else
        {
            //std::cout << "t12 is set to zero" << std::endl;
            t12 = 0.0;
        }

        double x12_modified_t2 = (v11_modified_t2 * t12) +
                                 (0.5 * a11_modified_t2 * std::pow(t12, 2));

        double v12_modified_t2 = (v11_modified_t2 + (a11_modified_t2 * t12));
        double a12_modified_t2 = a11_modified_t2;

        double x13_modified_t2 = (v12_modified_t2 * t13) +
                                 (0.5 * a12_modified_t2 * std::pow(t13, 2)) -
                                 (0.1666667 * Jerk * std::pow(t13, 3));
        double v13_modified_t2 = (v12_modified_t2 + (a12_modified_t2 * t13)) -
                                 (0.5 * Jerk * std::pow(t13, 2));
        double a13_modified_t2 = (a12_modified_t2 + (Jerk * t13));

        double min_distance1 = (x11_modified_t2 + x12_modified_t2 + x13_modified_t2 + need_distance);
        // //std::cout << "min_distance1: " << min_distance1 << std::endl;

        if (TargetDistance >= min_distance1)
        {
            //std::cout << "---------------------CASE 1a: WITH t2 --------------------------" << std::endl;
            x11 = (0.1666667 * Jerk * std::pow(t11, 3)) +
                                           (0.5 * CurrentAcceleration * std::pow(t11, 2)) +
                                           (CurrentVelocity * t11);

            v11 = (CurrentVelocity +
                                            (CurrentAcceleration * t11) +
                                            (0.5 * Jerk * std::pow(t11, 2)));

            a11 = (CurrentAcceleration +
                                            (Jerk * t11));

            a21 = -(Jerk * t21);
            
            double c2 = (TargetDistance) -
                        x11 - (v11 * t13) -
                        (0.5 * a11 * std::pow(t13, 2)) +
                        (0.1666667 * Jerk * std::pow(t13, 3)) -
                        (Jerk * std::pow(t21, 3));


            bool result = newtonRaphson1(c2);

            if (!result)
            {
                //std::cout << "Newton-Raphson 1 did not converge!" << std::endl;
            }

            // t22 = result[0];
            // t12 = result[1];

            x12 = (v11 * t12) + (0.5 * a11 * std::pow(t12, 2));
            v12 = (v11 + (a11 * t12));
            a12 = a11;
            x13 = (v12 * t13) + (0.5 * a12 * std::pow(t13, 2)) -
                                (0.1666667 * Jerk * std::pow(t13, 3));
            v13 = (v12 + (a12 * t13)) - (0.5 * Jerk * std::pow(t13, 2));
            a13 = (a12 - (Jerk * t13));
            //---------------------falling--------------------------
            x21 = (v13 * t21) - (0.1666667 * Jerk * std::pow(t21, 3));
            v21 = (v13 - (0.5 * Jerk * std::pow(t21, 2)));
            a21 = -(Jerk * t21);
            x22 = (v21 * t22) + (0.5 * a21 * std::pow(t22, 2));
            v22 = (v21 + (a21 * t22));
            a22 = a21;
            x23 = (v22 * t23) + (0.5 * a22 * std::pow(t23, 2)) +
                                (0.1666667 * Jerk * std::pow(t23, 3));
            v23 = (v22 + (a22 * t23)) + (0.5 * Jerk * std::pow(t23, 2));
            a23 = (a22 + (Jerk * t23));

        }
        else
        {
            //std::cout << "---------------------CASE 1b: WITHOUT t2 --------------------------" << std::endl;
            double x11_no_t2 = (0.1666667 * Jerk * std::pow(t11, 3)) +
                               (0.5 * CurrentAcceleration * std::pow(t11, 2)) +
                               (CurrentVelocity * t11);
            double v11_no_t2 = (CurrentVelocity +
                                (CurrentAcceleration * t11) +
                                (0.5 * Jerk * std::pow(t11, 2)));
            double a11_no_t2 = (CurrentAcceleration +
                                (Jerk * t11));
            double x13_no_t2 = (v11_no_t2 * t13) +
                               (0.5 * a11_no_t2 * std::pow(t13, 2)) -
                               (0.1666667 * Jerk * std::pow(t13, 3));
            double v13_no_t2 = (v11_no_t2 + (a11_no_t2 * t13)) -
                               (0.5 * Jerk * std::pow(t13, 2));
            double a13_no_t2 = (a11_no_t2 + (Jerk * t13));
            double min_distance2 = 2 * (x11_no_t2 + x13_no_t2);

            // //std::cout << "min_distance2: " << min_distance2 << std::endl;

            if (TargetDistance >= min_distance2)
            {
                //std::cout << "---------------------CASE 1b-1: WITHOUT t2, but enough to reach max acceleration from one side --------------------------" << std::endl;
                //std::cout << "Target position is enough to reach max acceleration from one side" << std::endl;
                x11 = (0.1666667 * Jerk * std::pow(t11, 3)) +
                                               (0.5 * CurrentAcceleration * std::pow(t11, 2)) +
                                               (CurrentVelocity * t11);
                v11 = (CurrentVelocity +
                                                (CurrentAcceleration * t11) +
                                                (0.5 * Jerk * std::pow(t11, 2)));
                a11 = (CurrentAcceleration +
                                                (Jerk * t11));

                double c3 = (TargetDistance) -
                            x11 - (v11 * t13) -
                            (0.5 * a11 * std::pow(t13, 2)) +
                            (0.1666667 * Jerk * std::pow(t13, 3));

                bool result = newtonRaphson2(c3);

                // t21 = result[0];
                // t12 = result[1];
                t23 = t21;
                t22 = 0.0;

                x12 = (v11 * t12) + (0.5 * a11 * std::pow(t12, 2));
                v12 = (v11 + (a11 * t12));
                a12 = a11;

                x13 = (v12 * t13) + (0.5 * a12 * std::pow(t13, 2)) -
                                    (0.1666667 * Jerk * std::pow(t13, 3));

                a13 = (a12 - (Jerk * t13));

                v13 = (v12 + (a12 * t13)) - (0.5 * Jerk * std::pow(t13, 2));

                x21 = (v13 * t21) - (0.1666667 * Jerk * std::pow(t21, 3));
                v21 = (v13 - (0.5 * Jerk * std::pow(t21, 2)));
                a21 = -(Jerk * t21);

                x22 = 0.0;
                v22 = v21;
                a22 = a21;

                x23 = (v21 * t23) + (0.5 * a21 * std::pow(t23, 2)) +
                                               (0.1666667 * Jerk * std::pow(t23, 3));

                v23 = (v21 + (a21 * t23)) +
                                               (0.5 * Jerk * std::pow(t23, 2));
                a23 = (a21 + (Jerk * t23));

            }
            else
            {
                //std::cout << "---------------------CASE 1b-2: WITHOUT t2, NOT enough to reach max acceleration --------------------------" << std::endl;
                double t_calculated = std::cbrt((TargetDistance) / (2 * Jerk));

                x11 = (0.1666667 * Jerk * std::pow(t_calculated, 3)) +
                        (0.5 * CurrentAcceleration * std::pow(t_calculated, 2)) +
                        (CurrentVelocity * t_calculated);

                v11 = (CurrentVelocity + (CurrentAcceleration * t_calculated) +
                                        (0.5 * Jerk * std::pow(t_calculated, 2)));
                a11 = (CurrentAcceleration + (Jerk * t_calculated));
                x12 = 0.0;
                v12 = v11;
                a12 = a11;
                x13 = (v11 * t_calculated) + (0.5 * a11 * std::pow(t_calculated, 2)) -
                                            (0.1666667 * Jerk * std::pow(t_calculated, 3));
                v13 = (v11 + (a11 * t_calculated)) - (0.5 * Jerk * std::pow(t_calculated, 2));
                a13 = (a11 + (Jerk * t_calculated));

                x21 = (v13 * t_calculated) - (0.1666667 * Jerk * std::pow(t_calculated, 3));
                v21 = (v13 - (0.5 * Jerk * std::pow(t_calculated, 2)));
                a21 = -(Jerk * t_calculated);
                x22 = 0.0;
                v22 = v21;
                a22 = a21;
                x23 = (v22 * t_calculated) + (0.5 * a22 * std::pow(t_calculated, 2)) +
                                               (0.1666667 * Jerk * std::pow(t_calculated, 3));
                v23 = (v22 + (a22 * t_calculated)) + (0.5 * Jerk * std::pow(t_calculated, 2));
                a23 = (a22 + (Jerk * t_calculated));
            }
        }
    }
    else if (t21 < t11)
    {
        //std::cout << " --------------CASE 2: t21 < t11 ------------------" << std::endl;
        double need_distance = (Jerk * std::pow(t11, 3));

        double x21_modified_t2 = (0.1666667 * Jerk * std::pow(t21, 3)) +
                                 (0.5 * CurrentAcceleration * std::pow(t21, 2)) +
                                 (CurrentVelocity * t21);

        double v21_modified_t2 = (CurrentVelocity +
                                  (CurrentAcceleration * t21) +
                                  (0.5 * Jerk * std::pow(t21, 2)));
        double a21_modified_t2 = (CurrentAcceleration +
                                  (Jerk * t21));

        if (t22 > 0.0001)
        {
            double t22_1, t22_2;
            double c = (0.5 * a21_modified_t2 * std::pow(t23, 2) +
                        (0.1666667 * Jerk * std::pow(t23, 3)) - need_distance);
    
            bool solved = solveQuadratic(a21_modified_t2 * 0.5, 
                (v21_modified_t2 + (a21_modified_t2 * t23)), c, t22_1, t22_2);
    
            if (solved)
            {
                t22 = std::max(t22_1, t22_2);
            }
            else
            {
                t22 = 0.0;
                return; // Error handling for no real solution
            }
        }
        else
        {
            //std::cout << "t22 is set to zero" << std::endl;
            t22 = 0.0;
        }

        double x22_modified_t2 = (v21_modified_t2 * t22) + (0.5 * a21_modified_t2 * std::pow(t22, 2));

        double v22_modified_t2 = (v21_modified_t2 + (a21_modified_t2 * t22));
        
        double a22_modified_t2 = a21_modified_t2;

        double x23_modified_t2 = (v22_modified_t2 * t13) +
                                 (0.5 * a22_modified_t2 * std::pow(t13, 2)) +
                                 (0.1666667 * Jerk * std::pow(t13, 3));

        double v23_modified_t2 = (v22_modified_t2 + (a22_modified_t2 * t13)) +
                                 (0.5 * Jerk * std::pow(t13, 2));

        double a23_modified_t2 = (a22_modified_t2 + (Jerk * t13));

        double min_distance1 = (x21_modified_t2 + x22_modified_t2 + x23_modified_t2 + need_distance);

        if ((TargetDistance >= min_distance1) && (t22 > 0.0001))
        {
            //std::cout << "---------------------CASE 2a: WITH t2 --------------------------" << std::endl;
            x11 = (0.1666667 * Jerk * std::pow(t11, 3)) +
                                           (0.5 * CurrentAcceleration * std::pow(t11, 2)) +
                                           (CurrentVelocity * t11);

            v11 = (CurrentVelocity + (CurrentAcceleration * t11) +
                                            (0.5 * Jerk * std::pow(t11, 2)));

            a11 = (CurrentAcceleration + (Jerk * t11));

            double c2 = (TargetDistance) -
                        x11 - (v11 * t13) -
                        (0.5 * a11 * std::pow(t13, 2)) +
                        (0.1666667 * Jerk * std::pow(t13, 3)) -
                        (Jerk * std::pow(t21, 3));

            a21 = -(Jerk * t21);

            bool result = newtonRaphson1(c2);

            if (!result)
            {
                //std::cout << "Newton-Raphson 1 did not converge!" << std::endl;
            }

            x12 = (v11 * t12) + (0.5 * a11 * std::pow(t12, 2));
            v12 = (v11 + (a11 * t12));
            a12 = a11;
            x13 = (v12 * t13) + (0.5 * a12 * std::pow(t13, 2)) -
                                           (0.1666667 * Jerk * std::pow(t13, 3));
            v13 = (v12 + (a12 * t13)) - (0.5 * Jerk * std::pow(t13, 2));
            a13 = (a12 - (Jerk * t13));
            //---------------------falling--------------------------
            x21 = (v13 * t21) - (0.1666667 * Jerk * std::pow(t21, 3));
            v21 = (v13 - (0.5 * Jerk * std::pow(t21, 2)));
            a21 = -(Jerk * t21);
            x22 = (v21 * t22) + (0.5 * a21 * std::pow(t22, 2));
            v22 = (v21 + (a21 * t22));
            a22 = a21;
            x23 = (v22 * t23) + (0.5 * a22 * std::pow(t23, 2)) +
                                           (0.1666667 * Jerk * std::pow(t23, 3));
            v23 = (v22 + (a22 * t23)) + (0.5 * Jerk * std::pow(t23, 2));
            a23 = (a22 + (Jerk * t23));
        }
        else
        {
            //std::cout << "---------------------CASE 2b: WITHOUT t2 --------------------------" << std::endl;
            double x21_no_t2 = (0.1666667 * Jerk * std::pow(t21, 3)) +
                               (0.5 * CurrentAcceleration * std::pow(t21, 2)) +
                               (CurrentVelocity * t21);
            double v21_no_t2 = (CurrentVelocity +
                                (CurrentAcceleration * t21) +
                                (0.5 * Jerk * std::pow(t21, 2)));
            double a21_no_t2 = (CurrentAcceleration +
                                (Jerk * t21));
            double x23_no_t2 = (v21_no_t2 * t13) +
                               (0.5 * a21_no_t2 * std::pow(t13, 2)) +
                               (0.1666667 * Jerk * std::pow(t13, 3));
            double v23_no_t2 = (v21_no_t2 + (a21_no_t2 * t13)) +
                               (0.5 * Jerk * std::pow(t13, 2));
            double a23_no_t2 = (a21_no_t2 + (Jerk * t13));
            double min_distance2 = 2 * (x21_no_t2 + x23_no_t2);

            // //std::cout << "min_distance2: " << min_distance2 << std::endl;

            if (TargetDistance >= min_distance2)
            {
                //std::cout << "---------------------CASE 2b-1: WITHOUT t2, but enough to reach max acceleration from both side --------------------------" << std::endl;
                double c3 = (TargetDistance) -
                (0.16667f * Jerk * std::pow(t21, 3)) -
                (CurrentVelocity * CurrentAcceleration / Jerk) -
                (0.5f * CurrentAcceleration * std::pow(t21, 3) / Jerk) -
                (0.5f * CurrentAcceleration * std::pow(t21, 2) / Jerk);
                
                bool result = newtonRaphson3(c3);
                if (!result)
                {
                    //std::cout << "Newton-Raphson 3 did not conv1erge!" << std::endl;
                }


                t13 = (CurrentAcceleration / Jerk) + t11;

                x11 = (0.1666667 * Jerk * std::pow(t11, 3)) +
                                               (0.5 * CurrentAcceleration * std::pow(t11, 2)) +
                                               (CurrentVelocity * t11);

                v11 = (CurrentVelocity + (CurrentAcceleration * t11) +
                                        (0.5 * Jerk * std::pow(t11, 2)));

                a11 = (CurrentAcceleration + (Jerk * t11));

                t12 = 0.0;
                x12 = 0.0;
                v12 = v11;
                a12 = a11;

                x13 = (v12 * t13) + (0.5 * a12 * std::pow(t13, 2)) -
                        (0.1666667 * Jerk * std::pow(t13, 3));

                a13 = (a12 - (Jerk * t13));
                v13 = (v12) + (a12 * t13) - (0.5 * Jerk * std::pow(t13, 2));

                x21 = (v13 * t21) - (0.1666667 * Jerk * std::pow(t21, 3));
                v21 = (v13 - (0.5 * Jerk * std::pow(t21, 2)));
                a21 = -(Jerk * t21);

                x22 = (v21 * t22) + (0.5 * a21 * std::pow(t22, 2));
                v22 = (v21) + (a21 * t22);
                a22 = a21;

                x23 = (v22 * t23) + (0.5 * a22 * std::pow(t23, 2)) +
                                               (0.1666667 * Jerk * std::pow(t23, 3));

                v23 = (v22 + (a22 * t23)) + (0.5 * Jerk * std::pow(t23, 2));
                a23 = (a22 + (Jerk * t23));

            }
            else
            {
                //std::cout << "---------------------CASE 2b-2: WITHOUT t2, NOT enough to reach max acceleration --------------------------" << std::endl;
                
                double t_calculated = std::cbrt((TargetDistance) / (2 * Jerk));
                if (t_calculated > t11)
                {
                    t_calculated = t11;
                }

                x11 = (0.1666667 * Jerk * std::pow(t_calculated, 3)) +
                                               (0.5 * CurrentAcceleration * std::pow(t_calculated, 2)) +
                                               (CurrentVelocity * t_calculated);

                v11 = (CurrentVelocity + (CurrentAcceleration * t_calculated) +
                                                (0.5 * Jerk * std::pow(t_calculated, 2)));
                a11 = (CurrentAcceleration + (Jerk * t_calculated));

                x12 = 0.0;
                v12 = v11;
                a12 = a11;
                x13 = (v11 * t_calculated) +
                                               (0.5 * a11 * std::pow(t_calculated, 2)) -
                                               (0.1666667 * Jerk * std::pow(t_calculated, 3));
                v13 = (v11 + (a11 * t_calculated)) -
                                               (0.5 * Jerk * std::pow(t_calculated, 2));
                a13 = (a11 + (Jerk * t_calculated));

                x21 = (v13 * t_calculated) - (0.1666667 * Jerk * std::pow(t_calculated, 3));
                v21 = (v13 - (0.5 * Jerk * std::pow(t_calculated, 2)));
                a21 = -(Jerk * t_calculated);
                x22 = 0.0;
                v22 = v21;
                a22 = a21;
                x23 = (v22 * t_calculated) +
                                               (0.5 * a22 * std::pow(t_calculated, 2)) +
                                               (0.1666667 * Jerk * std::pow(t_calculated, 3));
                v23 = (v22 + (a22 * t_calculated)) +
                                               (0.5 * Jerk * std::pow(t_calculated, 2));
                a23 = (a22 + (Jerk * t_calculated));

            }
        }
    }
    else
    {
        //std::cout << " --------------CASE 3: t21 = t11 ------------------" << std::endl;
        double x11_no_t2 = (0.1666667 * Jerk * std::pow(t11, 3)) +
                           (0.5 * CurrentAcceleration * std::pow(t11, 2)) +
                           (CurrentVelocity * t11);
        double v11_no_t2 = (CurrentVelocity + (CurrentAcceleration * t11) + 
                            (0.5 * Jerk * std::pow(t11, 2)));
        double a11_no_t2 = (CurrentAcceleration + (Jerk * t11));
        double x13_no_t2 = (v11_no_t2 * t13) +
                           (0.5 * a11_no_t2 * std::pow(t13, 2)) -
                           (0.1666667 * Jerk * std::pow(t13, 3));
        double v13_no_t2 = (v11_no_t2 + (a11_no_t2 * t13)) -
                           (0.5 * Jerk * std::pow(t13, 2));
        double a13_no_t2 = (a11_no_t2 - (Jerk * t13));

        double x21_no_t2 = (v13_no_t2 * t21) +
                            (0.5 * a13_no_t2 * std::pow(t21, 2)) -
                           (0.1666667 * Jerk * std::pow(t21, 3));
        double v21_no_t2 = (v13_no_t2 + (a13_no_t2 * t21) -
                            (0.5 * Jerk * std::pow(t21, 2)));
        double a21_no_t2 = (a13_no_t2 - (Jerk * t21));
        double x23_no_t2 = (v21_no_t2 * t23) +
                           (0.5 * a21_no_t2 * std::pow(t23, 2)) +
                           (0.1666667 * Jerk * std::pow(t23, 3));
        double v23_no_t2 = (v21_no_t2 + (a21_no_t2 * t23)) +
                           (0.5 * Jerk * std::pow(t23, 2));
        double a23_no_t2 = (a21_no_t2 + (Jerk * t23));

        double min_distance = (x11_no_t2 + x13_no_t2 + x21_no_t2 + x23_no_t2);


        if ((TargetDistance) <= min_distance)
        {
            //std::cout << "---------------------CASE 3a: WITHOUT t2, NOT enough to reach max acceleration both sides --------------------------" << std::endl;
            double t_calculated = std::cbrt((TargetDistance) /
                                            (2 * Jerk));
            x11 = (0.1666667 * Jerk * std::pow(t_calculated, 3)) +
                  (0.5 * CurrentAcceleration * std::pow(t_calculated, 2)) +
                  (CurrentVelocity * t_calculated);
            v11 = (CurrentVelocity +
                   (CurrentAcceleration * t_calculated) +
                   (0.5 * Jerk * std::pow(t_calculated, 2)));
            a11 = (CurrentAcceleration +
                   (Jerk * t_calculated));
            x12 = 0.0;
            v12 = v11;
            a12 = a11;
            x13 = (v11 * t_calculated) +
                  (0.5 * a11 * std::pow(t_calculated, 2)) -
                  (0.1666667 * Jerk * std::pow(t_calculated, 3));
            v13 = (v11 + (a11 * t_calculated)) -
                  (0.5 * Jerk * std::pow(t_calculated, 2));
            a13 = (a11 + (Jerk * t_calculated));

            x21 = (v13 * t_calculated) - (0.1666667 * Jerk * std::pow(t_calculated, 3));
            v21 = (v13 - (0.5 * Jerk * std::pow(t_calculated, 2)));
            a21 = -(Jerk * t_calculated);
            x22 = 0.0;
            v22 = v21;
            a22 = a21;
            x23 = (v22 * t_calculated) +
                  (0.5 * a22 * std::pow(t_calculated, 2)) +
                  (0.1666667 * Jerk * std::pow(t_calculated, 3));
            v23 = (v22 + (a22 * t_calculated)) +
                  (0.5 * Jerk * std::pow(t_calculated, 2));
            a23 = (a22 + (Jerk * t_calculated));
        }
        else
        {
            //std::cout << "---------------------CASE 3b: WITH t2 --------------------------" << std::endl;
            x11 = (0.1666667 * Jerk * std::pow(t11, 3)) +
                                           (0.5 * CurrentAcceleration * std::pow(t11, 2)) +
                                           (CurrentVelocity * t11);

            v11 = (CurrentVelocity +
                                            (CurrentAcceleration * t11) +
                                            (0.5 * Jerk * std::pow(t11, 2)));

            a11 = (CurrentAcceleration +
                                            (Jerk * t11));

            double c2 = (TargetDistance) -
                        x11 - (v11 * t13) -
                        (0.5 * a11 * std::pow(t13, 2)) +
                        (0.1666667 * Jerk * std::pow(t13, 3)) -
                        (Jerk * std::pow(t21, 3));

            a21 = -(Jerk * t21);

            bool result = newtonRaphson1(c2);

            if (!result)
            {
                //std::cout << "Newton-Raphson 1 did not converge!" << std::endl;
            }

            // t22 = result[0];
            // t12 = result[1];

            x12 = (v11 * t12) + (0.5 * a11 * std::pow(t12, 2));
            v12 = (v11 + (a11 * t12));
            a12 = a11;
            x13 = (v12 * t13) + (0.5 * a12 * std::pow(t13, 2)) -
                                           (0.1666667 * Jerk * std::pow(t13, 3));
            v13 = (v12 + (a12 * t13)) - (0.5 * Jerk * std::pow(t13, 2));
            a13 = (a12 - (Jerk * t13));
            //---------------------falling--------------------------
            x21 = (v13 * t21) - (0.1666667 * Jerk * std::pow(t21, 3));
            v21 = (v13 - (0.5 * Jerk * std::pow(t21, 2)));
            a21 = -(Jerk * t21);
            x22 = (v21 * t22) + (0.5 * a21 * std::pow(t22, 2));
            v22 = (v21 + (a21 * t22));
            a22 = a21;
            x23 = (v22 * t23) + (0.5 * a22 * std::pow(t23, 2)) +
                                           (0.1666667 * Jerk * std::pow(t23, 3));
            v23 = (v22 + (a22 * t23)) + (0.5 * Jerk * std::pow(t23, 2));
            a23 = (a22 + (Jerk * t23));
        }

    }
}

void Profiler::CalculatePositionProfile()
{

    // Case of maxAcc and maxVel value being reached
    t11 = (MaxAcceleration - CurrentAcceleration) / Jerk;

    t12 = ((MaxVelocity - CurrentVelocity) - (MaxAcceleration * t11)) / MaxAcceleration;

    t13 = (MaxAcceleration / Jerk);

    t21 = (MaxDeceleration) / Jerk;

    t22 = ((MaxVelocity - CurrentVelocity) - (MaxDeceleration * t21)) / MaxDeceleration;

    t23 = (MaxDeceleration / Jerk);

    if (t12 < 0.0)
    {
        t11 = std::sqrt((MaxVelocity - CurrentVelocity) / Jerk);
        t12 = 0.0;
        t13 = t11;
    }

    // DÜZELTİLMİŞ t22 < 0 DURUMU
    if (t22 < 0.0)
    {
        t21 = std::sqrt((MaxVelocity) / Jerk);
        t22 = 0.0;
        t23 = t21;
    }

    x11 = (0.1666667 * Jerk * std::pow(t11, 3)) +
          (0.5 * CurrentAcceleration * std::pow(t11, 2)) +
          (CurrentVelocity * t11);

    v11 = (CurrentVelocity +
           (CurrentAcceleration * t11) +
           (0.5 * Jerk * std::pow(t11, 2)));

    a11 = (CurrentAcceleration +
           (Jerk * t11));

    x12 = (v11 * t12) +
          (0.5 * a11 * std::pow(t12, 2));

    v12 = (v11 + (a11 * t12));

    a12 = a11;

    x13 = (v12 * t13) +
          (0.5 * a12 * std::pow(t13, 2)) -
          (0.1666667 * Jerk * std::pow(t13, 3));

    v13 = v12 + (a12 * t13) - (0.5 * Jerk * std::pow(t13, 2));

    a13 = a12 - (Jerk * t13);

    x21 = (MaxVelocity * t21) - (0.1666667 * Jerk * std::pow(t21, 3));

    v21 = (MaxVelocity - (0.5 * Jerk * std::pow(t21, 2)));

    a21 = -(Jerk * t21);

    x22 = (v21 * t22) + (0.5 * a21 * std::pow(t22, 2));

    v22 = (v21 + (a21 * t22));

    a22 = a21;

    x23 = (v22 * t23) +
          (0.5 * a22 * std::pow(t23, 2)) +
          (0.1666667 * Jerk * std::pow(t23, 3));

    v23 = (v22 + (a22 * t23)) +
          (0.5 * Jerk * std::pow(t23, 2));

    a23 = (a22 + (Jerk * t23));

    //std::cout << "full profile distance: " << (x11 + x12 + x13 + x21 + x22 + x23) << std::endl;

    if ((TargetDistance) > (x11 + x12 + x13 + x21 + x22 + x23))
    {
        return;
    }
    else
    {
        //std::cout << "Need to adjust profile to fit target distance" << std::endl;
        CalculateShorterProfile();
        if (x11 < 0.0 || x12 < 0.0 || x13 < 0.0 ||
            x21 < 0.0 || x22 < 0.0 || x23 < 0.0)
        {
            //std::cout << "Error in profile calculation, one of the segments is negative!" << std::endl;
        }
    }
    


}
