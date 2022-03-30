#ifndef __MAIN_HPP__
#define __MAIN_HPP__

#include <Arduino.h>
#include "util.hpp"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <array>
#include "ros_util.hpp"
#include "pid.hpp"


#define PHASE_A             4
#define PHASE_B             5
#define DIRECTION_PINA      12
#define DIRECTION_PINB      13
#define PWM_PIN             14



#define CW         1
#define CCW       -1

#define LED        2

#define FULL_TURN  1183.0
#define TS_VELOCITY_TASK   1000    // 1 s
#define TS_POSITION_TASK   250    // 0.5 s
#define TICKS_2_RPM  ( 60.0 / ((float)TS_VELOCITY_TASK / 1000.0) / FULL_TURN)
//#define TICKS_2_POS (1 / FULL_TURN * PI)

const char* ssid     = "********";
const char* password = "********";
// Set the rosserial socket server IP address
IPAddress server(10,3,184,80);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::Int16 l_wheel, r_wheel;
std_msgs::Float32 l_pos, r_pos; 

long int cnt = 0;
long int old_cnt = 0;
float left_velocity = 0.0;
int left_turns = 0;
float left_pos = 0.0;

#endif
