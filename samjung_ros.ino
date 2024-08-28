#include <ros.h> 
#include <Wire.h>
#include <Arduino.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>


const int16_t SEN50_ADDRESS = 0x69;

ros::NodeHandle nh;
std_msgs::Float32MultiArray SEN50_send;
ros::Publisher SEN50_SEND_Data("SEN50_topic", &SEN50_send);

int CONT_1 = 11;
int CONT_2 = 12;

int led_MOSFET1 = 5;
int led_MOSFET2 = 6;
int led_MOSFET3 = 7;
int sdl_num = 0;

// LED 제어를 위한 상태 및 시간 변수
unsigned long previousTime = 0;
unsigned long ledPreviousTime = 0; 
bool ledState = LOW;

int CONT_Log = 0;  // 마지막으로 유효한 입력을 저장

void CM300_Mode() {
  if (Serial.available() > 0) {
    int CONT_L = Serial.parseInt();
    Serial.print("Received input: ");
    Serial.println(CONT_L);
    
    if (CONT_L >= 1 && CONT_L <= 4) {
      CONT_Log = CONT_L;  // 유효한 입력만 저장
      Serial.print("Valid input: ");
      Serial.println(CONT_Log);
      
      switch(CONT_Log) {
        case 1:
          digitalWrite(CONT_1, HIGH);
          digitalWrite(CONT_2, HIGH);  // 강
          Serial.println("설정: 강");
          break;
        case 2:
          digitalWrite(CONT_1, LOW);
          digitalWrite(CONT_2, HIGH);  // 중
          Serial.println("설정: 중");
          break;
        case 3:
          digitalWrite(CONT_1, HIGH);
          digitalWrite(CONT_2, LOW);  // 약
          Serial.println("설정: 약");
          break;
        case 4:
          digitalWrite(CONT_1, LOW);
          digitalWrite(CONT_2, LOW);  // 끔
          Serial.println("설정: 꺼짐");
          break;
      }
    } else {
      Serial.println("Invalid input. Please enter a number between 1 and 4.");
    }
    
    while(Serial.available() > 0) {
      Serial.read();
    }
  }
}

ros::Subscriber<std_msgs::Int8> control_level("CM300_topic", CM300_Mode);


void SEN50_data() {    // 센서데이터 ROS로 받을거면 이렇게 할게 아니라 이 함수나 다른 함수에서 publish 해줘야함.
  uint16_t pm1p0, pm2p5, pm4p0, pm10p0;
  int16_t voc, nox, humidity, temperature;
  uint8_t data[24], counter;

  // Send command to read measurement data (0x03C4)
  Wire.beginTransmission(SEN50_ADDRESS);
  Wire.write(0x03);
  Wire.write(0xC4);
  Wire.endTransmission();
  // Wait 20 ms to allow the sensor to fill the internal buffer
  delay(20);

  // Read measurement data of SEN55, after two bytes a CRC follows
  Wire.requestFrom(SEN50_ADDRESS, 24);
  counter = 0;
  
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // PM1.0 to PM10 are unscaled unsigned integer values in ug / um3
  // VOC level is a signed int and scaled by a factor of 10 and needs to be divided by 10
  // humidity is a signed int and scaled by 100 and need to be divided by 100
  // temperature is a signed int and scaled by 200 and need to be divided by 200
  pm1p0 = (uint16_t)data[0] << 8 | data[1];
  pm2p5 = (uint16_t)data[3] << 8 | data[4];
  pm4p0 = (uint16_t)data[6] << 8 | data[7];
  pm10p0 = (uint16_t)data[9] << 8 | data[10];
  humidity = (uint16_t)data[12] << 8 | data[13];
  temperature = (uint16_t)data[15] << 8 | data[16];
  voc = (uint16_t)data[18] << 8 | data[19];
  nox = (uint16_t)data[21] << 8 | data[22];


  // 데이터를 Float32MultiArray 메시지에 할당
  SEN50_send.data[0] = float(pm1p0) / 10;
  SEN50_send.data[1] = float(pm2p5) / 10;
  SEN50_send.data[2] = float(pm4p0) / 10;
  SEN50_send.data[3] = float(pm10p0) / 10;
  SEN50_send.data[4] = float(voc) / 10;
  SEN50_send.data[5] = float(nox) / 10;
  SEN50_send.data[6] = float(humidity) / 100;
  SEN50_send.data[7] = float(temperature) / 200;

  SEN50_SEND_Data.publish(&SEN50_send);

  // Wait 1 s for next measurement
  delay(1000);

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

  SEN50_send.data_length = 8;  // 8개의 데이터를 보낼 것이므로
  SEN50_send.data = (float *)malloc(sizeof(float) * 8);
  
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
  delay(50);

  Wire.beginTransmission(SEN50_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.endTransmission();

  Serial.println("PM1.0\tPM2.5\tPM4.0\tPM10.0\tVOC_Index\tNOx_Index\tRH\tT");
  
  // Wait until sensor is ready, fan is initialized
  delay(1000);
}

void loop() {
  unsigned long cur_time = millis();

  if (cur_time - previousTime >= 100) { // 100ms마다 작업을 수행하도록 설정
    previousTime = cur_time;
    sdl_num++;

    switch (sdl_num) {
      case 0:
        nh.spinOnce();  // ROS 메시지 처리
        break;
      case 1:
        SEN50_data();   // 센서 데이터 읽기
        break;
      case 2:
        SEN50_SEND_Data.publish(&SEN50_send);
        break;
      case 3:
        led_control(led_MOSFET1, 1000, 200);  // LED 제어
        break;
      case 4:
        led_control(led_MOSFET2, 1000, 200);  // LED 제어
        break;
      case 5:
        led_control(led_MOSFET3, 1000, 200);  // LED 제어
        sdl_num = -1;  // 순환 리셋
        break;
    }
  }
}
