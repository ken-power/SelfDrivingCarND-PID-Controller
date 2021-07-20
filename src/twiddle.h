//
// Created by Ken Power on 20/07/2021.
//

#ifndef PID_TWIDDLE_H
#define PID_TWIDDLE_H

struct Twiddle
{
    Twiddle()
    {
        this->tolerance = 0.0001;
        this->twiddle_update = 1000;
        this->sum_dp = 0;
        this->best_error = 0;
        this->iteration = 0;
        this->param = 0;
        this->stage = 1;
        this->dp[0] = 0.1;
        this->dp[1] = 1.0;
        this->dp[2] = 0.00001;
        this->total_error = 0.0;
    }

    Twiddle(double tolerance)
    {
        this->tolerance = tolerance;
        Twiddle();
    }

    double TotalError(double *p, const double cte)
    {
        this->iteration += 1;
        total_error += cte * cte / this->iteration;

        // Activate twiddle
        if(this->iteration == this->twiddle_update)
        {
            this->best_error = total_error;
            this->Run(p);
            this->stage += 1;
        }

        if(this->iteration > this->twiddle_update and
           this->iteration % this->twiddle_update == 0)
        {
            this->Run(p);
            total_error = 0;
            this->iteration = 1000;
            this->stage += 1;

            if(this->stage > 3)
            {
                this->stage = 1;
                this->param += 1;

                if(this->param > 2)
                {
                    this->param = 0;
                }
            }
        }

        return total_error;
    }

    int iteration;

private:
    void Run(double *p)
    {
        this->sum_dp = 0;

        // Sum the P,I,D controller parameter changes
        for(int i = 0; i < 3; i++)
        {
            this->sum_dp += this->dp[i];
        }

        if(this->sum_dp > this->tolerance and this->stage == 1)
        {
            p[this->param] += this->dp[this->param];
            total_error = 0;
        }

        if(this->sum_dp > this->tolerance and this->stage == 2)
        {
            if(total_error < this->best_error)
            {
                this->best_error = total_error;
                this->dp[this->param] *= 1.1;
                total_error = 0;
            }
            else
            {
                p[this->param] -= 2 * dp[this->param];
                total_error = 0;
            }
        }

        if(this->sum_dp > this->tolerance and this->stage == 3)
        {
            if(total_error < this->best_error)
            {
                this->best_error = total_error;
                this->dp[this->param] *= 1.1;
                total_error = 0;
            }
            else
            {
                p[this->param] += this->dp[this->param];
                this->dp[this->param] *= 0.9;
                total_error = 0;
            }
        }
    }

private:
    double best_error;
    double tolerance;
    int twiddle_update;
    int param;
    int stage;
    double dp[3];
    double sum_dp;
    double total_error;
};

#endif //PID_TWIDDLE_H
