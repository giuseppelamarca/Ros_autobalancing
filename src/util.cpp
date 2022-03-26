 #include "util.hpp"
 #include <Arduino.h>
 #include <WiFi.h>

void wifi_connect(const char *ssid, const char *password){
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi  connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}