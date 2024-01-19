#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#ifndef STASSID
#define STASSID "SmieLab_Net"
#define STAPSK  "smielab510"
#endif
#define MOTOR_NUM 11
#define PwnOffset 3   // For Install, the start pin of motors
/*
 * IO_2 of ESP_01s connect the SCL of driver board
 * IO_0 of ESP_01s connect the SDA of driver board
*/
#define SCL 2
#define SDA 0
/*
 * DSM44 --> [0, 180]
 * Reference pulse duration: [0.5ms, 2.5ms] --> [0, 180]
 * 50Hz --> 20ms
 * SERVOMIN = 4096*0.5/20 = 102.4 --> 102
 * SERVOMAX = 4096*2.5/20 = 512
*/
#define SERVOMIN  102 // This is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  512 // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMAX  580 // From Test
unsigned int localPort = 6666;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged\r\n";       // a string to send back

int motorInit[MOTOR_NUM+1] = {0};
int motorValues[MOTOR_NUM+1];

void streamSplit(char *streamString, const char* flag){
  int n = 0;
  char* result = NULL;

  result = strtok(streamString, flag);
  while(result != NULL){
    String temp = String(result);
    motorValues[n++] = temp.toInt();
    result = strtok(NULL, flag);
  }
}


WiFiUDP Udp;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("UDP server on port %d\n", localPort);
  Udp.begin(localPort);
  /************************************************************/
  Wire.begin(SDA, SCL);
  pwm.begin();
  pwm.setPWMFreq(60);  // This is the maximum PWM frequency
  for(int i = 0; i < MOTOR_NUM+1; i++){
    motorInit[i] = 90;
    motorValues[i] = 0;
  }
}

void loop() {
  // if there's data available, read a packet
  // Control data update and response the requirement
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // read the packet into packetBufffer
    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
    streamSplit(packetBuffer, ",");
    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
  /************************************************************/
  int i;
  for(i = 0; i < MOTOR_NUM; i++){
    int pwnPos = i + PwnOffset;
    int curAngle = (motorInit[i] + motorValues[i]);//%180;
    int pulse = map(curAngle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(pwnPos, 0, pulse);
  }
  delay(20);
}
