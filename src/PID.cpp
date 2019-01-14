#include "PID.h"


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */

  Kp = Kp_;
  Kd = Kd_;
  Ki = Ki_;

  
  p_error = 0;
  d_error = 0;
  i_error = 0;

  previous_cte = 0;

}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */

  p_error = cte;
  d_error = cte - previous_cte;
  i_error = i_error + cte;

  previous_cte = cte;

}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */

  double steering = - Kp * p_error - Kd * d_error - Ki * i_error;
   

  return steering;  
}