#ifndef PID_H
#define PID_H

#include "twiddle.h"

class PID
{
public:
    /**
    * Constructor
    */
    PID();

    /**
    * Destructor.
    */
    virtual ~PID();

    /**
    * Initialize PID.
    */
    void Init(double Kp_, double Ki_, double Kd_);

    /**
    * Update the PID error variables given cross track error.
    */
    double UpdateError(double cte);

    /**
    * Calculate the total PID error.
    */
    double TotalError(double cte);

    const double *ParameterVector() const;

private:
    double p_error;
    double i_error;
    double d_error;

    double Kp;
    double Ki;
    double Kd;
    double p[3];

    double steering_value;

    Twiddle twiddle;
};

#endif /* PID_H */
