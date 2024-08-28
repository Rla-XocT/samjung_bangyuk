#include <ros.h> 
#include <Wire.h>
#include <Arduino.h>
#include <SensirionI2CSen5x.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32MultiArray.h>

#define DATA_LENGTH 4 // 실제로 보낼 데이터 길이

SensirionI2CSen5x sen5x;

const int16_t SEN50_ADDRESS = 0x69;

ros::NodeHandle nh;
std_msgs::Int32MultiArray SEN50_send;
ros::Publisher SEN50_SEND_Data("SEN50_topic", &SEN50_send);

int CONT_1 = 11;
int CONT_2 = 12;

int led_MOSFET1 = 5;
int led_MOSFET2 = 6;
int led_MOSFET3 = 7;
int sdl_num = 0;

unsigned long previousTime = 0;
unsigned long ledPreviousTime = 0; 
bool ledState = LOW;

int CONT_Log = 0;

void CM300_Mode(const std_msgs::Int8& msg) {
    int CONT_L = msg.data;
    
    if (CONT_L >= 1 && CONT_L <= 4) {
      CONT_Log = CONT_L;
      
      switch(CONT_Log) {
        case 1:
          digitalWrite(CONT_1, HIGH);
          digitalWrite(CONT_2, HIGH);
          break;
        case 2:
          digitalWrite(CONT_1, LOW);
          digitalWrite(CONT_2, HIGH);
          break;
        case 3:
          digitalWrite(CONT_1, HIGH);
          digitalWrite(CONT_2, LOW); 
          break;
        case 4:
          digitalWrite(CONT_1, LOW);
          digitalWrite(CONT_2, LOW);
          break;
      }
    }
  }


ros::Subscriber<std_msgs::Int8> control_level("CM300_topic", CM300_Mode);


void SEN50_data() { 
  uint16_t error;
  char errorMessage[256];

  delay(1000);

  // Read Measurement
  float massConcentrationPm1p0;
  float massConcentrationPm2p5;
  float massConcentrationPm4p0;
  float massConcentrationPm10p0;
  float ambientHumidity;
  float ambientTemperature;
  float vocIndex;
  float noxIndex;

  error = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);


  SEN50_send.data[0] = static_cast<int32_t>(massConcentrationPm1p0 * 1000); // 부동 소수점 변환 예시
  SEN50_send.data[1] = static_cast<int32_t>(massConcentrationPm2p5 * 1000);
  SEN50_send.data[2] = static_cast<int32_t>(massConcentrationPm4p0 * 1000);
  SEN50_send.data[3] = static_cast<int32_t>(massConcentrationPm10p0 * 1000);

  SEN50_SEND_Data.publish(&SEN50_send);

}

void led_control(int lpin, unsigned long onTime, unsigned long offTime) {
  unsigned long currentMillis = millis();
  
  if (ledState == LOW && currentMillis - ledPreviousTime >= offTime) {
    digitalWrite(lpin, HIGH);
    ledState = HIGH;
    ledPreviousTime = currentMillis;
  } 
  else if (ledState == HIGH && currentMillis - ledPreviousTime >= onTime) {
    digitalWrite(lpin, LOW);
    ledState = LOW;
    ledPreviousTime = currentMillis;
  }
}


void setup() {
  nh.initNode();
  nh.subscribe(control_level);
  nh.advertise(SEN50_SEND_Data);

  SEN50_send.data_length = 8; 
  SEN50_send.data = (int32_t *)malloc(sizeof(int32_t) * 8); 

  pinMode(CONT_1, OUTPUT);
  pinMode(CONT_2, OUTPUT);
  pinMode(led_MOSFET1, OUTPUT);
  pinMode(led_MOSFET2, OUTPUT);
  pinMode(led_MOSFET3, OUTPUT);
  digitalWrite(led_MOSFET1, LOW);
  digitalWrite(led_MOSFET2, LOW);
  digitalWrite(led_MOSFET3, LOW);
  
  Serial.begin(115200);
  while(!Serial);

  Wire.begin();
  sen5x.begin(Wire);

  sen5x.deviceReset();
  sen5x.setTemperatureOffsetSimple(0.0);
  sen5x.startMeasurement();

  Wire.beginTransmission(SEN50_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.endTransmission();
}

void loop() {
  unsigned long cur_time = millis();

  if (cur_time - previousTime >= 50) { // 100ms
    previousTime = cur_time;
    Serial.print("Valid input: ");
    sdl_num++;

    switch (sdl_num) {
      case 0:
        break;
      case 1:
        nh.spinOnce();  // ROS 
        break;
      case 2:
        SEN50_data();   
        break;
      case 3:
        led_control(led_MOSFET1, 1000, 200);  // LED 
        break;
      case 4:
        led_control(led_MOSFET2, 1000, 200);  // LED 
        break;
      case 5:
        led_control(led_MOSFET3, 1000, 200);  // LED 
        sdl_num = 0;  
        break;
    }
  }
}
