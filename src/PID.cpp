#include "PID.h"
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // Initialize PID coefficients
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  // Initialize PID errors
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  // DEBUG message
  std::cout << "Initialized: Kp = " << Kp << ", Ki = " << Ki <<  ", Kd = " << Kd << std::endl;
}

void PID::UpdateError(double cte) {
  // TODO: Update PID errors based on cte.
  d_error = cte - p_error;  // cte - prev_cte (which is the d_error before the update step)
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  // Calculate and return the total error
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}