#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <MPU6050.h>  // MPU6050 library
#include <Wire.h>  
#include <esp_now.h>
#include <WiFi.h>
#include <TinyGPS++.h>

#define GPS_BAUDRATE 9600  // The default baudrate of NEO-6M is 9600

TinyGPSPlus gps;  // the TinyGPS++ object
#define TRIGGER_PIN  23
#define ECHO_PIN     18
#define buzzerPin 15 // Buzzer pin

// Motor A connections
int enA = 13;
int in1 = 26;
int in2 = 21;

// Motor B connections
int enB = 14;
int in3 = 32;
int in4 = 33;

long duration, distance;

/*void right_turn() {
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(500);
}

void left_turn() {
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(500);
}*/

uint8_t broadcastAddress[] = {0xBC, 0xDD, 0xC2, 0xD1, 0xD5, 0x60};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success\n" : "Delivery Fail\n");
}

void mpuTask(void *pvParameters) 
{
    MPU6050 mpu6050;
    mpu6050.initialize();  // Initialize sensor
    WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    vTaskDelete(NULL);
  }

  esp_now_peer_info_t peerInfo;
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  //addpeer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

    while (1) 
    {
        // Read sensor data
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        int16_t mpu_data[]={ax,ay,az,gx,gy,gz};
        uint8_t data[sizeof(mpu_data)];
        memcpy(data, mpu_data, sizeof(mpu_data));

        // Process sensor data (example: print to Serial Monitor)
        Serial.print("Acceleration (mg): ");
        Serial.print(ax); Serial.print(", ");
        Serial.print(ay); Serial.print(", ");
        Serial.print(az); Serial.print(", ");

        Serial.print("Rotation (degrees/s): ");
        Serial.print(gx); Serial.print(", ");
        Serial.print(gy); Serial.print(", ");
        Serial.println(gz);
        Serial.println();

        delay(2000);
        esp_err_t result = esp_now_send(broadcastAddress, data, sizeof(data));
   
       if (result == ESP_OK) 
       {
          Serial.println("Sent MPU6050 data with success");
      }
      else {
          Serial.println("Error sending the data");
  }
        // Delay to control task execution frequency
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1000ms delay
    }
}

void ultrasonicTask(void *para) {
  // Initialize ESP-NOW
  
  while(1){
  
  digitalWrite(TRIGGER_PIN, LOW);// Clear trigger pin
  delayMicroseconds(2);
  
  digitalWrite(TRIGGER_PIN, HIGH);// Trigger ultrasonic sensor
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);// Echo pulse duration

  distance = duration * 0.034 / 2;  // Speed of sound is 340 m/s(distance=time*speed)
  
  // Print distance to Serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm\n");
  
  delay(1000); 

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &distance, sizeof(distance));
   
  if (result == ESP_OK) {
    Serial.println("Sent ultrasonic data with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  vTaskDelay(pdMS_TO_TICKS(2000)); 
  }
}

void taskGPS(void *pvParameters) 
{
    while(1)
    {
    int16_t info[3];
  uint8_t data[sizeof(info)];
  
  if (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {
        Serial.print(F("- latitude: "));
        Serial.println(gps.location.lat());
        info[0]=gps.location.lat();
        Serial.print(F("- longitude: "));
        Serial.println(gps.location.lng());
        info[1]=gps.location.lng();
        Serial.print(F("- altitude: "));
        info[2]=gps.altitude.meters();
        if (gps.altitude.isValid())
          Serial.println(gps.altitude.meters());
        else
          Serial.println(F("INVALID"));
      } else {
        Serial.println(F("- location: INVALID"));
      }
      memcpy(data, info, sizeof(info));
      esp_err_t result = esp_now_send(broadcastAddress, data, sizeof(data));
   
       if (result == ESP_OK) {
          Serial.println("Sent GPS data with success");
      }
      else {
          Serial.println("Error sending the data");
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
    } 
  }   
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(GPS_BAUDRATE);
  Serial.println(F("ESP32 - GPS module"));
  Wire.begin();         // Initialize I2C communication
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  pinMode(buzzerPin, OUTPUT); // Buzzer pin*/
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  xTaskCreatePinnedToCore(taskGPS, "TaskGPS", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(ultrasonicTask,"Utrasonic Task",2048,NULL,2,NULL,1);
  xTaskCreatePinnedToCore(mpuTask,"MPU Task",2048,NULL,3,NULL,1);
}

void loop() {
  // Motor control code
  int speed;
  switch(distance) {
    case 0 ... 14: // distance < 15
      //right_turn();
      //left_turn(); // Turn left after right turn
      speed = 10;
      digitalWrite(buzzerPin, HIGH);
      break;
    case 15 ... 19: // distance < 20
      digitalWrite(buzzerPin, LOW);
      speed = 40;
      break;
    case 20 ... 100: // distance < 25
      digitalWrite(buzzerPin, LOW);
      speed = 80;
      break;
    default:
      digitalWrite(buzzerPin, LOW);
      speed = 100;
  }

  // Forward motion
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  delay(100);
}

