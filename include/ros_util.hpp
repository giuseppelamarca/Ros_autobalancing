#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <vector>
 #include <WiFi.h>


template<std::size_t SIZE_PUB, std::size_t SIZE_SUB, typename T>
void ros_init(IPAddress server, const uint16_t serverPort, ros::NodeHandle &nh, std::array<ros::Publisher, SIZE_PUB>&publisher, std::array<ros::Subscriber<T>, SIZE_SUB> &subscriber){
  // Set the connection to rosserial socket servera
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
 
  for (auto &pub : publisher) {
    nh.advertise(pub);
  }

  for (auto &sub : subscriber) {
    nh.subscribe(sub);
  }  
}