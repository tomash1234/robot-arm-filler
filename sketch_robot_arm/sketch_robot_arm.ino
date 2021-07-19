#include "Servo.h"
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>


#define LOCAL_PORT 5101
#define PACKET_SIZE 3

Servo servo_base;
Servo servo_arm;
Servo servo_small;


WiFiUDP Udp;

void connect_to_wifi(const char* ssid, const char* password){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  WiFi.begin(ssid, password); 
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(++i); Serial.print(' ');
    if(i > 10){
      Serial.println("Failed! No connection");  
      for(;;){}
      break;
    }
  }
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());
}

void setup() {
  servo_base.attach(D6, 800, 5800); 
  servo_arm.attach(D7, 800, 5800); 
  servo_small.attach(D5, 500, 2400);

  
  connect_to_wifi("TomasProjects", "TomasProjects");
  Udp.begin(LOCAL_PORT);
  
  servo_base.write(0);
  servo_arm.write(45);
  servo_small.write(0);  
}

void read_packets(){
  if(Udp.parsePacket()){ 
    uint8_t received[PACKET_SIZE];
    int len = Udp.read(received, PACKET_SIZE);
    Serial.println(len);
    if(len == PACKET_SIZE){
      servo_base.write(received[0]);
      servo_arm.write(received[1]);
      servo_small.write(received[2]);  
    }
  }
}

void loop() {
  read_packets();
}
