#include "main.hpp"
#include "encoder.hpp"
#include "motor.hpp"

auto pid{PID{0,0,0}};
Motor<1> motor{DIRECTION_PINA, DIRECTION_PINB, PWM_PIN, PHASE_A, PHASE_B, MAX_VOLTAGE, FULL_TURN, POWER_SUPPLY};

std::array<ros::Publisher, 4> pub_list{
                                        ros::Publisher{"left_wheel", &l_wheel},
                                        ros::Publisher{"right_wheel", &r_wheel},
                                        ros::Publisher{"l_pos", &l_pos},
                                        ros::Publisher{"r_pos", &r_pos}
                                      };

void setVelocity( const std_msgs::Float32& cmd_msg){
  Serial.print("Received: ");
  Serial.println(cmd_msg.data);
}

void setVoltage( const std_msgs::Float32& cmd_msg){
  Serial.print("Received: ");
  Serial.println(cmd_msg.data);
  motor.setVoltage(cmd_msg.data);
}

void setP(const std_msgs::Float32& cmd_msg){
  pid.setP(cmd_msg.data);
}

void setI(const std_msgs::Float32& cmd_msg){
  pid.setI(cmd_msg.data);
}

void setD(const std_msgs::Float32& cmd_msg){
  pid.setD(cmd_msg.data);
}

std::array<ros::Subscriber<std_msgs::Float32>, 5> sub_list{
                                        ros::Subscriber<std_msgs::Float32>{"setVoltage", setVoltage},
                                        ros::Subscriber<std_msgs::Float32>{"setVelocity", setVelocity},
                                        ros::Subscriber<std_msgs::Float32>{"setP", setP},
                                        ros::Subscriber<std_msgs::Float32>{"setI", setI},
                                        ros::Subscriber<std_msgs::Float32>{"setD", setD}
                                      };


float getTime(){
  return (float)millis()/1000.0;
}

void TaskDisplayVelocity(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
      Serial.println(motor.getSpeed(millis()));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TS_VELOCITY_TASK));
    }
}

void TaskDisplayPosition(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int led_status = 0;
  for (;;) {
      Serial.println(motor.getPosition());
      

      vTaskDelayUntil(&xLastWakeTime, TS_POSITION_TASK);
    }
}

void TaskPositionControl(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
      auto control = pid.output(l_pos.data, 0, millis());
      vTaskDelayUntil(&xLastWakeTime, TS_POSITION_TASK);
    }
}

void setup() {
  Serial.begin(115200);

  //wifi_connect(ssid, password);

  //ros_init(server, serverPort, nh, pub_list, sub_list);

  // Create task for FreeRTOS notification
  xTaskCreate(TaskDisplayVelocity, "Velocity Print", 2000, NULL, 1, NULL );
  //xTaskCreate(TaskDisplayPosition, "Position Print", 2000, NULL, 1, NULL );

  pinMode(LED, OUTPUT);
  motor.setVoltage(3);
}

void loop()
{
  //nh.spinOnce();
  auto volt  = Serial.parseFloat();
  if (volt != 0.0)
    motor.setVoltage(volt);
  
}