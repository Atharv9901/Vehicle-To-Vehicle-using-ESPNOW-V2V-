#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>  

//#define buzzerPin 13
// Motor A connections
int enA = 12;
int in1 = 27;
int in2 = 26;

// Motor B connections
int enB = 14;
int in3 = 32;
int in4 = 33;

long distance;

void right_turn() {
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
}


// Callback function to handle received data
void onDataReceived(const uint8_t *macAddr, const uint8_t *data, int dataLen) {
    // Check data length
    if (dataLen == sizeof(int16_t) * 6) {
      // Extract accelerometer data
      int16_t ax, ay, az,gx, gy, gz;
      memcpy(&ax, data, sizeof(int16_t));
      memcpy(&ay, data + sizeof(int16_t), sizeof(int16_t));
      memcpy(&az, data + 2*sizeof(int16_t), sizeof(int16_t));

      memcpy(&gx,  data + 3*sizeof(int16_t), sizeof(int16_t));
      memcpy(&gy,  data + 4*sizeof(int16_t), sizeof(int16_t));
      memcpy(&gz,  data + 5*sizeof(int16_t), sizeof(int16_t));      
      // Print accelerometer data
      Serial.println();
      Serial.print("Accelerometer Data: ");
      Serial.print("ax = ");
      Serial.print(ax);
      Serial.print(", ay = ");
      Serial.print(ay);
      Serial.print(", az = ");
      Serial.println(az);
      Serial.print("gx = ");
      Serial.print(gx);
      Serial.print(", gy = ");
      Serial.print(gy);
      Serial.print(", gz = ");
      Serial.println(gz);
    } else if (dataLen == sizeof(long)) {
      // Extract ultrasonic sensor data
      
      memcpy(&distance, data, sizeof(long));
      Serial.println();
      // Print ultrasonic sensor data
      Serial.print("Ultrasonic Distance: ");
      Serial.println(distance);
    }else if(dataLen=(sizeof(int16_t)*3)){

      int16_t lat,lon,alt;
      memcpy(&lat, data, sizeof(int16_t));
      memcpy(&lon, data + sizeof(int16_t), sizeof(int16_t));
      memcpy(&alt, data + 2*sizeof(int16_t), sizeof(int16_t));
      //print GPS Data
      Serial.println();
      Serial.print("GPS Data: ");
      Serial.print("Latitude = ");
      Serial.print(lat);
      Serial.print(", Longitude = ");
      Serial.print(lon);
      Serial.print(", Altitude = ");
      Serial.println(alt);
    }
    else {
      Serial.println("Invalid data length received");
    }
  
}

void setup() {
  Serial.begin(115200);

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
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
  // Register callback function for receiving data
  esp_now_register_recv_cb(onDataReceived);
}

void loop() {
  // Motor control code
  int speed;
  switch(distance) {
    case 0 ... 14: // distance < 15
      //right_turn();
      //left_turn(); // Turn left after right turn
      speed = 25;
      //digitalWrite(buzzerPin, HIGH);
      break;
    case 15 ... 19: // distance < 20
      //digitalWrite(buzzerPin, LOW);
      speed = 50;
      break;
    case 20 ... 24: // distance < 25
      //digitalWrite(buzzerPin, LOW);
      speed = 100;
      break;
    default:
      //digitalWrite(buzzerPin, LOW);
      speed = 75;
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

