#include "pid.h"

PID::PID(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), prev_error(0), total_error(0) {
}

double PID::get_output() const {
  return this->output;
}

void PID::set_output(double output_e) {
  this->output = output_e; // For now, later we might add another functionality for setting it like limits or whatever
}


double PID::compute(double error) {

  if (error == 0.0) {
    return error;
  }
  
  this->total_error += abs(error);

  double derivative = abs(error - this->prev_error);
  // if ((error > 0 && this->prev_error < 0) || (error < 0 && this->prev_error > 0)) {
  //   // Reset derivative term or take some action to smooth out the transition
  //   derivative = 0;
  // }
  
  double output_e = (
    error * this->kp +
    this->total_error * this->ki +
    derivative * this->kd
  );

  this->prev_error = error;
  this->set_output(output_e);

  return this->output;

}

void PID::reset() {
  this->prev_error  = 0;
  this->total_error = 0;
}