
#include "PID.h"

PID::PID()
{
    double tolerance = 0.0001;
    this->twiddle = Twiddle(tolerance);
}

PID::~PID() = default;

void PID::Init(double Kp_, double Ki_, double Kd_)
{
    // Initial controller values
    this->Kp = Kp_;
    this->Ki = Ki_;
    this->Kd = Kd_;
    this->p[0] = Kp;
    this->p[1] = Kd;
    this->p[2] = Ki;

    // Error values
    this->p_error = 0.0;
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;

    double tolerance = 0.0001;
    this->twiddle = Twiddle(tolerance);
}

double PID::UpdateError(double cte)
{
    this->twiddle.iteration += 1;
    this->i_error += cte;
    this->d_error = cte - this->p_error;
    this->p_error = cte;

    this->steering_value = -p[0] * cte - p[1] * d_error - p[2] * i_error;

    return steering_value;
}


double PID::TotalError(double cte)
{
    return this->twiddle.TotalError(this->p, cte);
}

const double *PID::ParameterVector() const
{
    return p;
}

