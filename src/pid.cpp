#include "pid.hpp"

float PID::output(float value, float target, float time){
      auto error = target - value;
      auto TS = time - old_TS;
      this->old_TS += TS;
      error_int += error * TS;
      error_der = (error - error_old) / TS;
      this->error_old = error;
      return error * this->P + error_int * this->I + error_der * this->D;
    } 