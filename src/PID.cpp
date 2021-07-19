#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID()
{}

PID::~PID()
{}

/**
 * Initialize PID coefficients (and errors, if needed)
 */
void PID::Init(double Kp_, double Ki_, double Kd_)
{
    this->Kp = Kp_;
    this->Ki = Ki_;
    this->Kd = Kd_;
}

/**
 * Update PID errors based on cte.
 */
void PID::UpdateError(double cte)
{
    // d_error is difference from old CTE (p_error) to the new CTE
    this->d_error = (cte - p_error);

    // Set p_error to the new CTE
    this->p_error = cte;

    // i_error is the sum of CTEs to this point
    this->i_error += cte;
}

/**
 * Calculate and return the total error
 */
double PID::TotalError()
{
    return -Kp * p_error - Kd * d_error - Ki * i_error;
}
