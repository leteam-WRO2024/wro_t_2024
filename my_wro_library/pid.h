#ifndef PID_CTRL_H
#define PID_CTRL_H

#include "Arduino.h"

class PID {
  public:
    PID(double kp, double ki, double kd);

    // main functions
    double compute(double error);
    void   reset();

    // getters & setters
    double get_output() const;
    
  private:
    double kp, ki, kd;

    double prev_error;
    double total_error;
    double output;
    void   set_output(double output_e);
    // double abs(double value);
};


#endif