#include "main.hpp"
#include "encoder.hpp"
#include "motor.hpp"

#define PROFILE_SAMPLES  201
std::array<float, PROFILE_SAMPLES> sinProfile = {-1.2246467991473532e-16, -0.031410759078128236, -0.06279051952931358, -0.09410831331851435, -0.12533323356430454, -0.15643446504023098, -0.18738131458572457, -0.21814324139654276, -0.24868988716485482, -0.27899110603922955, -0.3090169943749475, -0.3387379202452913, -0.36812455268467814, -0.3971478906347806, -0.4257792915650729, -0.45399049973954686, -0.4817536741017152, -0.5090414157503714, -0.5358267949789967, -0.5620833778521308, -0.5877852522924732, -0.6129070536529764, -0.6374239897486899, -0.6613118653236518, -0.6845471059286888, -0.7071067811865476, -0.7289686274214114, -0.7501110696304597, -0.7705132427757895, -0.7901550123756905, -0.8090169943749475, -0.8270805742745617, -0.8443279255020152, -0.8607420270039439, -0.8763066800438637, -0.8910065241883679, -0.9048270524660195, -0.9177546256839813, -0.9297764858882515, -0.9408807689542256, -0.9510565162951536, -0.9602936856769431, -0.9685831611286312, -0.9759167619387474, -0.9822872507286887, -0.9876883405951378, -0.9921147013144779, -0.99556196460308, -0.9980267284282716, -0.9995065603657316, -1.0, -0.9995065603657316, -0.9980267284282716, -0.99556196460308, -0.9921147013144778, -0.9876883405951377, -0.9822872507286886, -0.9759167619387473, -0.9685831611286311, -0.960293685676943, -0.9510565162951535, -0.9408807689542253, -0.9297764858882513, -0.917754625683981, -0.9048270524660195, -0.8910065241883678, -0.8763066800438635, -0.8607420270039435, -0.8443279255020149, -0.8270805742745617, -0.8090169943749475, -0.7901550123756902, -0.7705132427757891, -0.7501110696304593, -0.7289686274214113, -0.7071067811865475, -0.6845471059286885, -0.6613118653236518, -0.6374239897486894, -0.6129070536529764, -0.5877852522924731, -0.5620833778521304, -0.5358267949789965, -0.509041415750371, -0.4817536741017151, -0.45399049973954636, -0.4257792915650724, -0.3971478906347805, -0.36812455268467764, -0.3387379202452912, -0.309016994374947, -0.278991106039229, -0.24868988716485468, -0.2181432413965422, -0.18738131458572446, -0.15643446504023042, -0.12533323356430398, -0.09410831331851423, -0.06279051952931301, -0.03141075907812812, 4.440892098500626e-16, 0.031410759078128556, 0.06279051952931346, 0.09410831331851467, 0.12533323356430442, 0.1564344650402313, 0.18738131458572488, 0.21814324139654262, 0.24868988716485513, 0.2789911060392294, 0.30901699437494784, 0.33873792024529165, 0.3681245526846784, 0.3971478906347809, 0.42577929156507277, 0.4539904997395472, 0.4817536741017155, 0.5090414157503718, 0.5358267949789969, 0.5620833778521307, 0.5877852522924735, 0.6129070536529767, 0.6374239897486901, 0.6613118653236522, 0.6845471059286888, 0.7071067811865478, 0.7289686274214117, 0.7501110696304599, 0.7705132427757895, 0.7901550123756904, 0.8090169943749475, 0.8270805742745622, 0.8443279255020153, 0.8607420270039438, 0.8763066800438637, 0.8910065241883678, 0.9048270524660198, 0.9177546256839814, 0.9297764858882516, 0.9408807689542256, 0.9510565162951535, 0.9602936856769433, 0.9685831611286312, 0.9759167619387474, 0.9822872507286887, 0.9876883405951378, 0.9921147013144779, 0.9955619646030801, 0.9980267284282716, 0.9995065603657316, 1.0, 0.9995065603657315, 0.9980267284282716, 0.99556196460308, 0.9921147013144778, 0.9876883405951378, 0.9822872507286886, 0.9759167619387473, 0.9685831611286311, 0.9602936856769431, 0.9510565162951536, 0.9408807689542252, 0.9297764858882512, 0.917754625683981, 0.9048270524660195, 0.8910065241883675, 0.8763066800438633, 0.8607420270039434, 0.844327925502015, 0.8270805742745617, 0.809016994374947, 0.79015501237569, 0.7705132427757889, 0.7501110696304594, 0.7289686274214114, 0.7071067811865469, 0.6845471059286882, 0.6613118653236515, 0.6374239897486895, 0.6129070536529764, 0.5877852522924725, 0.5620833778521301, 0.5358267949789962, 0.5090414157503711, 0.4817536741017152, 0.4539904997395461, 0.4257792915650721, 0.39714789063478023, 0.36812455268467775, 0.3387379202452913, 0.3090169943749467, 0.27899110603922866, 0.24868988716485438, 0.21814324139654231, 0.18738131458572457, 0.15643446504023012, 0.12533323356430365, 0.09410831331851391, 0.06279051952931314, 0.031410759078128236, 1.2246467991473532e-16};

auto pid{PID{0,0,0}};
Motor<1> motor{DIRECTION_PINA, DIRECTION_PINB, PWM_PIN, PHASE_A, PHASE_B, MAX_VOLTAGE, FULL_TURN, POWER_SUPPLY, true};

int data_sent = 0;

std::array<ros::Publisher, 10> pub_list{
                                        ros::Publisher{"left_wheel", &l_wheel},
                                        ros::Publisher{"right_wheel", &r_wheel},
                                        ros::Publisher{"l_pos", &l_pos},
                                        ros::Publisher{"r_pos", &r_pos},
                                        ros::Publisher{"l_voltage_input", &l_voltage_input}, 
                                        ros::Publisher{"r_voltage_input", &r_voltage_input}, 
                                        ros::Publisher{"l_velocity", &l_velocity}, 
                                        ros::Publisher{"r_velocity", &r_velocity}, 
                                        ros::Publisher{"l_current", &l_current}, 
                                        ros::Publisher{"r_current", &r_current}
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
  for (;;) {
      vTaskDelay(pdMS_TO_TICKS(TS_VELOCITY_TASK));
    }
}

void TaskDisplayPosition(void *pvParameters)
{
  int led_status = 0;
  for (;;) {
      //Serial.println(motor.getPosition());
      motor.setVoltage(sinProfile[led_status] * MAX_VOLTAGE);
      led_status++;
      if (led_status >= 201)
        led_status=0;
      

      vTaskDelay(TS_POSITION_TASK);
    }
}

void TaskPositionControl(void *pvParameters)
{
  for (;;) {
      auto control = pid.output(l_pos.data, 0, millis());
      vTaskDelay(TS_POSITION_TASK);
    }
}

std::array<float, RECORDED_DATA> current_data;
std::array<float, RECORDED_DATA> speed_data;
std::array<float, RECORDED_DATA> voltage_data;

void TaskMotorRecord(void *pvParameters)
{
  int led_status = 0;
  int record_data = 0;
  for (;;) {
      //Serial.println(motor.getPosition());
      auto voltage_ctrl = sinProfile[led_status] * MAX_VOLTAGE;
      motor.setVoltage(voltage_ctrl);
      led_status++;
      if (led_status >= PROFILE_SAMPLES)
        led_status=0;
      
        /*
        current_data[record_data] = motor.getCurrent()/1.0f;
        speed_data[record_data] = motor.getSpeed(millis());
        voltage_data[record_data] = voltage_ctrl;
        */
      l_current.data[record_data] = motor.getCurrent()/1.0f;
      l_velocity.data[record_data] = motor.getSpeed(millis());
      l_voltage_input.data[record_data] = voltage_ctrl;

      record_data++;
      if  (record_data >= RECORDED_DATA) { 
        record_data = 0;
        pub_list[4].publish(&l_voltage_input);
        pub_list[6].publish(&l_velocity);
        pub_list[8].publish(&l_current);
      }
      vTaskDelay(TS_CONTROL_TASK);
    }
}

void TaskPrintRecord(void *pvParameters)
{
  for (;;) {
      if (data_sent) {
        Serial.println("Current");
        for (auto data: current_data)
          Serial.println(data);

        Serial.println("Speed");
        for (auto data: speed_data)
          Serial.println(data);

        Serial.println("Voltage");
        for (auto data: voltage_data)
          Serial.println(data);

        data_sent = 0;
        Serial.println("FINE");
      }

      vTaskDelay(TS_PRINT_TASK);
    }
}

void setup() {
  Serial.begin(115200);

  wifi_connect(ssid, password);

  motor.init();
  ros_init(server, serverPort, nh, pub_list, sub_list);

  // Create task for FreeRTOS notification
  //xTaskCreate(TaskDisplayVelocity, "Velocity Print", 3000, NULL, 3, NULL );
  //xTaskCreate(TaskDisplayPosition, "Position Print", 2000, NULL, 2, NULL );
  xTaskCreate(TaskMotorRecord, "Motor Record", 3000, NULL, 3, NULL );
  //xTaskCreate(TaskPrintRecord, "Print Record", 3000, NULL, 1, NULL );

  pinMode(LED, OUTPUT);
  motor.setVoltage(3);
}

void loop()
{
  nh.spinOnce();
}