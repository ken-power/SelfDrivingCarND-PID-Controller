#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID()
{
    this->Kp = 0.0;
    this->Ki = 0.0;
    this->Kd = 0.0;
}

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
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
}

/**
 * Update PID errors based on cte.
 */
void PID::UpdateError(double cte)
{
    // Set p_error to the new CTE
    this->p_error = cte;

    // i_error is the sum of CTEs so far
    this->i_error += cte;

    // d_error is difference between the new CTE and old CTE (p_error)
    this->d_error = cte - p_error;
}

/**
 * Calculate and return the total error
 */
double PID::TotalError()
{
    return ((-Kp * p_error) - (Ki * i_error) - (Kd * d_error) );
}

double PID::P() const
{
    return p_error;
}

double PID::I() const
{
    return i_error;
}

double PID::D() const
{
    return d_error;
}
