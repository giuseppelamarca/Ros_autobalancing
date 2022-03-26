#ifndef __PID_HPP__
#define __PID_HPP__

class PID{
  float P, I, D;
  float out;
  float error_int, error_der, error_old;
  float old_TS;

  public:
    PID(float p, float i, float d): P(p), I(i), D(d){
      error_int = 0.0;
      error_der = 0.0;
      error_old = 0.0;
    } 

    float output(float value, float target, float time);

    void setP(float p){ this->P = p;}
    void setI(float i){ this->I = i;}
    void setD(float d){ this->D = d;}
};

#endif