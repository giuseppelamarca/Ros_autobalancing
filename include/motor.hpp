#ifndef __MOTOR__HPP__
#define __MOTOR__HPP__
#include "Arduino.h"
#include "pid.hpp"
#include "encoder.hpp"
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

#define MAX_MOTORS 5
#define SPEED_CONTROLLER    0
#define POSITION_CONTROLLER 1

#define MAX_VOLTAGE         6
#define POWER_SUPPLY        7.5
#define MOTOR_FREQ          1000
#define MOTOR_CHANNEL       0
#define MOTOR_RESOLUTION    8

template <int N>
class Motor: public Encoder<N>{

  int target_controller;
  float target_position, target_speed;
  int direction_pinA, direction_pinB, pwm_pin;
  float max_voltage, full_rot, power_supply;
  float TICKS_2_POS;
  bool current_sensor;

  // user need to overload this function 
  bool initCurrentSensor(){
    if (! ina219.begin())
        Serial.println("Failed to find INA219 chip");
    ina219.setCalibration_32V_1A();
  }

  float actual_speed;
  int last_cnt;    // last numbet of encode count to calculate speed
  long unsigned int last_time_ms;   // time im ms of the last time that the speed function was called
  float VELOCITY;   // constant (modified only in constructor) to pass from ticks/time to rpm

  void configurePin(int pin, int mode){
    pinMode(pin, mode);
  }

  int readPhaseA(){
      return digitalRead(this->phaseA);
  }

  void setPin(int pin, int value){
    digitalWrite(pin, value);
  }

  void setPwm(int pin, float value){
    if (value > 1)
      value = 1;
    ledcWrite(MOTOR_CHANNEL, value * 255);
  }

  void configurePwm(int pin, int channel, int freq, int resolution){
    configurePin(pin, OUTPUT);
        // configure LED PWM functionalitites
    ledcSetup(channel, freq, resolution);
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(pin, channel);    
  }

  public:
    Motor(int direction_pinA, int direction_pinB, int pwm_pin, int phaseA, int phaseB, float max_voltage, float full_rot, float power_supply, bool current_sensor): Encoder<N>(phaseA, phaseB),
      direction_pinA(direction_pinA), direction_pinB(direction_pinB), pwm_pin(pwm_pin),
      max_voltage(max_voltage), full_rot(full_rot), power_supply(power_supply), current_sensor(current_sensor)
    {

    } 

    void init(){
      configurePin(direction_pinA, OUTPUT);
      configurePin(direction_pinB, OUTPUT);
      configurePwm(pwm_pin, MOTOR_CHANNEL, MOTOR_FREQ, MOTOR_RESOLUTION);
      TICKS_2_POS = 1 / full_rot * PI;
      VELOCITY = 60/full_rot * 1000;  //60 is for rpm, 1000 is the milliseconds

      if (current_sensor){
        initCurrentSensor();
      }
    }


    void selectController(int controller){}
    void setPosition(float position){}
    void setSpeed(float speed){}
    void setVoltage(float voltage){
      if (voltage >= 0){
        if (voltage > max_voltage)
          voltage = max_voltage;
        setPin(direction_pinA, 1);
        setPin(direction_pinB, 0);
        setPwm(pwm_pin, voltage/power_supply);

      }
      else{
        if (voltage < -max_voltage)
          voltage = -max_voltage;
        setPin(direction_pinA, 0);
        setPin(direction_pinB, 1);
        setPwm(pwm_pin, -voltage/power_supply);       
      }
    }
    float getPosition(){ return this->getCnt() * TICKS_2_POS;}

    float getSpeed(unsigned long int time_ms){ 
      auto cnt = this->getCnt();
      this->actual_speed = (float)(cnt - last_cnt) / (float)(time_ms - last_time_ms) * VELOCITY;  
      last_cnt = cnt;
      last_time_ms = time_ms;
      return this->actual_speed;}

    float getCurrent(){ return ina219.getCurrent_mA();}
};

#endif