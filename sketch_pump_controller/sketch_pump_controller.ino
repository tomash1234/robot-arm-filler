#include <WiFiUdp.h>
#include <ESP8266WiFi.h>


#define LOCAL_PORT 5101
#define PACKET_SIZE 1

#define PIN_RELE D5

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
  Serial.begin(9600);
  pinMode(PIN_RELE, OUTPUT);
  
  connect_to_wifi("TomasProjects", "TomasProjects");
  Udp.begin(LOCAL_PORT);

  digitalWrite(PIN_RELE, HIGH);
}

void read_packets(){
  if(Udp.parsePacket()){ 
    Serial.println("Listen");
    uint8_t received[PACKET_SIZE];
    int len = Udp.read(received, PACKET_SIZE);
    Serial.println(len);
    if(len == PACKET_SIZE){
      if(received[0] > 128){
        digitalWrite(PIN_RELE, LOW);
      }else{
        digitalWrite(PIN_RELE, HIGH);
      }
    }
  }
}

void loop() {
  read_packets();
}
