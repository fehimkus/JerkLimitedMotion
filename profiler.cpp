#include "profiler.h"

bool solveQuadratic(double a, double b, double c, double &x1, double &x2)
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

inline void solve2x2System(const double A[2][2], const double b[2], double x[2])
{
    const double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];

    if (std::abs(det) < 1e-12)
    {
        // Handle singular or near-singular matrix
        x[0] = x[1] = 0.0;
        return;
    }

    const double inv_det = 1.0 / det;
    x[0] = (A[1][1] * b[0] - A[0][1] * b[1]) * inv_det;
    x[1] = (A[0][0] * b[1] - A[1][0] * b[0]) * inv_det;
}

// Compute the system of equations (optimized)
inline void computeSystem1(const double x[2], double f[2],
                           double t11, double t13, double t21, double a11,
                           double V11, double J, double c)
{
    const double t22 = x[0];
    const double t12 = x[1];

    // First equation: t21² + t21*t22 = 0.5*t11² + t11*t12 + t11*t13 - 0.5*t13²
    f[0] = ((t21 * t21) + (t21 * t22)) - ((0.5 * t11 * t11) + (t11 * t12) + (t11 * t13) - (0.5 * t13 * t13));

    // Second equation: V11*t12 + 0.5*a11*t12² + a11*t12*t13 + 1.5*J*t21²*t22 + 0.5*J*t21*t22² = c
    f[1] = ((V11 * t12) + (0.5 * a11 * t12 * t12) + (a11 * t12 * t13) +
            (1.5 * J * t21 * t21 * t22) + (0.5 * J * t21 * t22 * t22)) -
           c;
}

inline void computeJacobian1(const double x[2], double J_mat[2][2],
                             double t11, double t21, double a11,
                             double V11, double J, double t13)
{
    const double t22 = x[0];
    const double t12 = x[1];

    // Partial derivatives of the first equation:
    // f1 = t21² + t21*t22 - (0.5*t11² + t11*t12 + t11*t13 - 0.5*t13²)
    J_mat[0][0] = t21;  // ∂f1/∂t22 = t21 (derivative of t21*t22)
    J_mat[0][1] = -t11; // ∂f1/∂t12 = -t11 (derivative of -t11*t12)

    // Partial derivatives of the second equation:
    // f2 = V11*t12 + 0.5*a11*t12² + a11*t12*t13 + 1.5*J*t21²*t22 + 0.5*J*t21*t22² - c
    J_mat[1][0] = 1.5 * J * t21 * t21 + J * t21 * t22; // ∂f2/∂t22
    J_mat[1][1] = V11 + a11 * t12 + a11 * t13;         // ∂f2/∂t12
}

std::vector<double> newtonRaphson1(double t11, double t13, double t21,
                                   double a11, double V11, double J,
                                   double c, double t22_initial,
                                   double t12_initial, double tol = 1e-6,
                                   int max_iter = 100)
{
    double x[2] = {t22_initial, t12_initial};
    double f[2];
    double J_mat[2][2];
    double dx[2];
    double minus_f[2];

    for (int iter = 0; iter < max_iter; ++iter)
    {
        computeSystem1(x, f, t11, t13, t21, a11, V11, J, c);

        // Check for convergence using squared L2 norm
        const double norm = f[0] * f[0] + f[1] * f[1];
        if (norm < tol * tol)
        {
            std::cout << "Converged after " << iter << " iterations." << std::endl;
            return {x[0], x[1]};
        }

        // Compute Jacobian
        computeJacobian1(x, J_mat, t11, t21, a11, V11, J, t13);

        // Solve J*dx = -f
        minus_f[0] = -f[0];
        minus_f[1] = -f[1];
        solve2x2System(J_mat, minus_f, dx);

        // Update solution
        x[0] += dx[0];
        x[1] += dx[1];
    }

    std::cout << "Warning: Newton-Raphson did not converge after " << max_iter << " iterations." << std::endl;
    return {x[0], x[1]};
}

// Compute the Jacobian matrix (optimized)

// Compute the system of equations (optimized)
inline void computeSystem2(const double x[2], double f[2],
                           double t11, double t13, double a11, double V11, double J, double c)
{
    const double t21 = x[0];
    const double t12 = x[1];

    // first equation
    f[0] = (t21 * t21) - ((t11 * t11 * 0.5) + (t11 * t12) + (t11 * t13) - (0.5 * t13 * t13));

    // second equation
    f[1] = (V11 * t12) + (0.5 * a11 * t12 * t12) + (a11 * t12 * t13) + (J * t21 * t21 * t21) - c;
}

// Solve 2x2 linear system directly (optimized for small system)
inline void computeJacobian2(const double x[2], double J[2][2],
                             double t11, double t13, double a11, double V11, double J_param)
{
    const double t21 = x[0];
    const double t12 = x[1];

    // Partial derivatives for first equation
    J[0][0] = 2.0f;  // df1/dt21
    J[0][1] = -1.0f; // df1/dt12

    // Partial derivatives for second equation
    J[1][0] = 3.0f * J_param * t21 * t21;         // df2/dt21
    J[1][1] = 2.0f * a11 * t12 + V11 + a11 * t13; // df2/dt12
}

// Optimized Newton-Raphson method for 2D system
std::vector<double> newtonRaphson2(double t11, double t13, double a11, double V11,
                                   double J, double c, double t21_initial,
                                   double t12_initial, double tol = 1e-6,
                                   int max_iter = 100)
{
    double x[2] = {t21_initial, t12_initial};
    double f[2];
    double J_mat[2][2];
    double dx[2];
    double minus_f[2];

    for (int iter = 0; iter < max_iter; ++iter)
    {
        computeSystem2(x, f, t11, t13, a11, V11, J, c);

        // Check for convergence using L2 norm
        const double norm = f[0] * f[0] + f[1] * f[1];
        if (norm < tol * tol)
        { // Compare squared norm to squared tolerance
            return {x[0], x[1]};
        }

        // Compute Jacobian
        computeJacobian2(x, J_mat, t11, t13, a11, V11, J);

        // Solve J*dx = -f
        minus_f[0] = -f[0];
        minus_f[1] = -f[1];
        solve2x2System(J_mat, minus_f, dx);

        // Update solution
        x[0] += dx[0];
        x[1] += dx[1];
    }

    return {x[0], x[1]};
}

inline void computeSystem3(const double x[2], double f[2],
                           double t21, double J, double Vcurr,
                           double acurr, double t1, double c)
{
    const double t11 = x[0];
    const double t22 = x[1];
    // First equation: // J*t21^2 + J*t21*t22 = Vcurr + acurr*t11 + 0.5*J*t11^2 + // acurr*t1 + 0.5*J*t1^2 + 0.5*acurr^2/J
    f[0] = (J * t21 * t21 + J * t21 * t22) -
           (Vcurr + acurr * t11 + 0.5 * J * t11 * t11 +
            acurr * t1 + 0.5 * J * t1 * t1 + 0.5 * acurr * acurr / J);
    // Second equation: // c = (acurr^2*t11/J) + 2.5*acurr*t11^2 + Vcurr*t11 + 1.33334*J*t11^3 + // 1.5*J*t21^2*t22 + 0.5*J*t21*t22^2
    f[1] = (acurr * acurr * t11 / J + 2.5 * acurr * t11 * t11 +
            Vcurr * t11 + 1.33334 * J * t11 * t11 * t11 +
            1.5 * J * t21 * t21 * t22 + 0.5 * J * t21 * t22 * t22) - c;
}

inline void computeJacobian3(const double x[2], double J_mat[2][2],
                             double t21, double J, double Vcurr,
                             double acurr, double t1)
{
    const double t11 = x[0];
    const double t22 = x[1];

    J_mat[0][0] = -acurr - J * t11;
    J_mat[0][1] =  J * t21;

    J_mat[1][0] = acurr * acurr / J + 5.0 * acurr * t11 +
                  Vcurr + 4.0 * J * t11 * t11;
    J_mat[1][1] = 1.5 * J * t21 * t21 + J * t21 * t22;
}

// Returns true if converged, false if failed
bool newtonRaphson3(double t21, double J, double Vcurr,
                    double acurr, double c,
                    double &t11, double &t22,
                    double tol = 1e-6, int max_iter = 100)
{
    // --- Security checks ---
    if (J <= 0.0 || t21 < 0.0 || t11 < 0.0 || t22 < 0.0)
        return false;

    double x[2] = {t11, t22};
    double f[2], J_mat[2][2], dx[2], minus_f[2];

    for (int iter = 0; iter < max_iter; ++iter)
    {
        computeSystem3(x, f, t21, J, Vcurr, acurr, t11, c);

        // Check convergence
        if (f[0]*f[0] + f[1]*f[1] < tol * tol)
        {
            t11 = std::max(0.0, x[0]);
            t22 = std::max(0.0, x[1]);
            return true;
        }

        computeJacobian3(x, J_mat, t21, J, Vcurr, acurr, t11);

        minus_f[0] = -f[0];
        minus_f[1] = -f[1];

        solve2x2System(J_mat, minus_f, dx);

        x[0] += dx[0];
        x[1] += dx[1];

        // Enforce constraints (time ≥ 0)
        if (x[0] < 0.0) x[0] = 0.0;
        if (x[1] < 0.0) x[1] = 0.0;
    }

    return false; // Not converged
}


void Profiler::CalculateShorterProfile()
{

    std::cout << "Target position is NOT enough to reach max velocity and max acceleration" << std::endl;
    //---------------------rising without t2
    if (t21 > t11)
    {

        double need_distance = (Jerk * std::pow(t21, 3));

        double x11_modified_t2 = (0.1666667 * Jerk * std::pow(t11, 3)) +
                                 (0.5 * CurrentAcceleration * std::pow(t11, 2)) +
                                 (CurrentVelocity * t11);

        double v11_modified_t2 = (CurrentVelocity +
                                  (CurrentAcceleration * t11) +
                                  (0.5 * Jerk * std::pow(t11, 2)));
        double a11_modified_t2 = (CurrentAcceleration +
                                  (Jerk * t11));

        double t12_1, t12_2;
        double c = (0.5 * a11_modified_t2 * std::pow(t13, 2) +
                    (0.1666667 * Jerk * std::pow(t13, 3)) - need_distance);

        bool solved = solveQuadratic(a11_modified_t2 * 0.5, 
            (v11_modified_t2 + (a11_modified_t2 * t13)), c, t12_1, t12_2);

        if (solved)
        {
            t12 = std::max(t12_1, t12_2);
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
        std::cout << "min_distance1: " << min_distance1 << std::endl;

        if ((TargetDistance) >= min_distance1)
        {
            std::cout << "Target position is enough to reach max acceleration both sides" << std::endl;
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

            std::vector<double> result = newtonRaphson1(
                t11, t13, t21,
                a11, v11, Jerk,
                c2, t22, t12);

            t22 = result[0];
            t12 = result[1];

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

            // std::cout << "min_distance2: " << min_distance2 << std::endl;

            if (TargetDistance >= min_distance2)
            {
                std::cout << "Target position is enough to reach max acceleration from one side" << std::endl;
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

                std::vector<double> result = newtonRaphson2(t11, t13,
                                                            a11, v11, Jerk,
                                                            c3, t21, t12);

                t21 = result[0];
                t12 = result[1];
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
                std::cout << "Target position is not enough to reach max acceleration from both side1" << std::endl;
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
        double need_distance = (Jerk * std::pow(t11, 3));

        double x21_modified_t2 = (0.1666667 * Jerk * std::pow(t21, 3)) +
                                 (0.5 * CurrentAcceleration * std::pow(t21, 2)) +
                                 (CurrentVelocity * t21);

        double v21_modified_t2 = (CurrentVelocity +
                                  (CurrentAcceleration * t21) +
                                  (0.5 * Jerk * std::pow(t21, 2)));
        double a21_modified_t2 = (CurrentAcceleration +
                                  (Jerk * t21));

        double t22_1, t22_2;
        double c = (0.5 * a21_modified_t2 * std::pow(t23, 2) +
                    (0.1666667 * Jerk * std::pow(t23, 3)) - need_distance);

        bool solved = solveQuadratic(a21_modified_t2 * 0.5, 
            (v21_modified_t2 + (a21_modified_t2 * t23)), c, t22_1, t22_2);

        if (solved)
        {
            t22 = std::max(t22_1, t22_2);
        }

        double x22_modified_t2 = (v21_modified_t2 * t22) +
                                 (0.5 * a21_modified_t2 * std::pow(t22, 2));

        double v22_modified_t2 = (v21_modified_t2 + (a21_modified_t2 * t22));
        double a22_modified_t2 = a21_modified_t2;

        double x23_modified_t2 = (v22_modified_t2 * t13) +
                                 (0.5 * a22_modified_t2 * std::pow(t13, 2)) -
                                 (0.1666667 * Jerk * std::pow(t13, 3));
        double v23_modified_t2 = (v22_modified_t2 + (a22_modified_t2 * t13)) -
                                 (0.5 * Jerk * std::pow(t13, 2));
        double a23_modified_t2 = (a22_modified_t2 + (Jerk * t13));

        double min_distance1 = (x21_modified_t2 + x22_modified_t2 + x23_modified_t2 + need_distance);

        if ((TargetDistance) >= min_distance1)
        {
            std::cout << "Target position is enough to reach max acceleration both sides" << std::endl;
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

            std::vector<double> result = newtonRaphson1(t11, t13, t21,
                                                        a11, v11, Jerk, c2,
                                                        t22, t12);

            t22 = result[0];
            t12 = result[1];

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
            double x21_no_t2 = (0.1666667 * Jerk * std::pow(t21, 3)) +
                               (0.5 * CurrentAcceleration * std::pow(t21, 2)) +
                               (CurrentVelocity * t21);
            double v21_no_t2 = (CurrentVelocity +
                                (CurrentAcceleration * t21) +
                                (0.5 * Jerk * std::pow(t21, 2)));
            double a21_no_t2 = (CurrentAcceleration +
                                (Jerk * t21));
            double x23_no_t2 = (v21_no_t2 * t13) +
                               (0.5 * a21_no_t2 * std::pow(t13, 2)) -
                               (0.1666667 * Jerk * std::pow(t13, 3));
            double v23_no_t2 = (v21_no_t2 + (a21_no_t2 * t13)) -
                               (0.5 * Jerk * std::pow(t13, 2));
            double a23_no_t2 = (a21_no_t2 + (Jerk * t13));
            double min_distance2 = 2 * (x21_no_t2 + x23_no_t2);

            std::cout << "min_distance2: " << min_distance2 << std::endl;

            if (TargetDistance >= min_distance2)
            {
                std::cout << "TODO: Target position is enough to reach max acceleration from one side" << std::endl;
                std::cout << "t11: " << t11 << ", t12: " << t12 << " t13: " << t13 << " t21: " << t21 << " t22: " << t22 << " t23: " << t23 << std::endl;

                double c3 = (TargetDistance) -
                            (0.1666667 * Jerk * std::pow(t21, 3)) -
                            (CurrentVelocity * CurrentAcceleration / Jerk) -
                            (0.5 * CurrentAcceleration * std::pow(t21, 3) / Jerk) -
                            (0.5 * CurrentAcceleration * std::pow(t21, 2) / Jerk);
                            
                bool result = newtonRaphson3(
                    t21, Jerk,
                    CurrentVelocity, CurrentAcceleration,
                    c3, t11, t22);
                if (!result)
                {
                    std::cout << "Newton-Raphson 3 did not converge!" << std::endl;
                }

                std::cout << "Calculated t11: " << t11 << ", t22: " << t22 << std::endl;
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
                std::cout << "Target position is not enough to reach max acceleration from both side2" << std::endl;
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
        }
    }
    else
    {
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
            std::cout << "Target position is not enough to reach max acceleration from both side2" << std::endl;
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
            std::cout << "Target position is enough to reach max acceleration both sides" << std::endl;
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

            std::vector<double> result = newtonRaphson1(t11, t13, t21,
                                                        a11, v11, Jerk, c2,
                                                        t22, t12);

            t22 = result[0];
            t12 = result[1];

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
        std::cout << "t12 < 0.0, adjusting profile" << std::endl;
        t12 = 0.0;
    }
    if (t22 < 0.0)
    {
        std::cout << "t22 < 0.0, adjusting profile" << std::endl;
        t22 = 0.0;
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

    v13 = (v12 + (a12 * t13)) -
          (0.5 * Jerk * std::pow(t13, 2));

    a13 = (a12 - (Jerk * t13));

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

    std::cout << "full profile distance: " << (x11 + x12 + x13 + x21 + x22 + x23) << std::endl;

    if ((TargetDistance) > (x11 + x12 + x13 + x21 + x22 + x23))
    {
        return;
    }
    else
    {
        std::cout << "Need to adjust profile to fit target distance" << std::endl;
        CalculateShorterProfile();
        if (x11 < 0.0 || x12 < 0.0 || x13 < 0.0 ||
            x21 < 0.0 || x22 < 0.0 || x23 < 0.0)
        {
            std::cout << "Error in profile calculation, one of the segments is negative!" << std::endl;
        }
    }
    
    // std::cout << "----------------------------  RESULT  ----------------------------" << std::endl;
    // std::cout << "t11: " << t11 << " t12: " << t12 << " t13: " << t13 << std::endl;
    // std::cout << "x11: " << x11 << " x12: " << x12 << " x13: " << x13 << std::endl;
    // std::cout << "v11: " << v11 << " v12: " << v12 << " v13: " << v13 << std::endl;
    // std::cout << "a11: " << a11 << " a12: " << a12 << " a13: " << a13 << std::endl;
    // std::cout << "t21: " << t21 << " t22: " << t22 << " t23: " << t23 << std::endl;
    // std::cout << "x21: " << x21 << " x22: " << x22 << " x23: " << x23 << std::endl;
    // std::cout << "v21: " << v21 << " v22: " << v22 << " v23: " << v23 << std::endl;
    // std::cout << "a21: " << a21 << " a22: " << a22 << " a23: " << a23 << std::endl;
    // std::cout << "itogo: " << x11 + x12 + x13 + x21 + x22 + x23 << std::endl;

}
