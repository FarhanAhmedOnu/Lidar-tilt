#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
Servo tiltServo;

// Timing for 200Hz IMU output (5ms interval)
unsigned long lastImuSend = 0;
const unsigned long IMU_INTERVAL_US = 5000; 

void setup() {
  // 250000 baud gives us plenty of headroom for binary data
  Serial.begin(250000); 
  Wire.begin();
  
  if (!mpu.begin()) {
    while (1) { delay(10); } // Halt if IMU not found
  }
  
  // Optimal ranges for tilt estimation (4G is lower noise than 16G)
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G); 
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   
  
  tiltServo.attach(9);
  tiltServo.writeMicroseconds(1500); // Center the servo (0 degrees)
}

void loop() {
  // ---------------------------------------------------------
  // 1. NON-BLOCKING SERVO COMMAND PROCESSING
  // Listens for ASCII commands like "S1500\n" (1500 microseconds)
  // ---------------------------------------------------------
  static String servoCmd = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      if (servoCmd.startsWith("S")) {
        int us = servoCmd.substring(1).toInt();
        // Constrain to safe physical limits of the MG996R
        if (us >= 1000 && us <= 2000) {
          tiltServo.writeMicroseconds(us);
        }
      }
      servoCmd = ""; // Reset for next command
    } else if ((c >= '0' && c <= '9') || c == 'S') {
      servoCmd += c;
    }
  }

  // ---------------------------------------------------------
  // 2. HIGH-FREQUENCY IMU READING & BINARY TRANSMISSION
  // ---------------------------------------------------------
  unsigned long currentMicros = micros();
  if (currentMicros - lastImuSend >= IMU_INTERVAL_US) {
    lastImuSend = currentMicros; // Update timer
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // BINARY PAYLOAD (30 bytes total):
    // Header (2) + Timestamp (4) + Accel X,Y,Z (12) + Gyro X,Y,Z (12)
    uint8_t header[2] = {0xAA, 0x55};
    uint32_t timestamp = currentMicros;
    
    // Write raw bytes directly to serial (Extremely fast)
    Serial.write(header, 2);
    Serial.write((uint8_t*)&timestamp, 4);
    Serial.write((uint8_t*)&a.acceleration.x, 4);
    Serial.write((uint8_t*)&a.acceleration.y, 4);
    Serial.write((uint8_t*)&a.acceleration.z, 4);
    Serial.write((uint8_t*)&g.gyro.x, 4);
    Serial.write((uint8_t*)&g.gyro.y, 4);
    Serial.write((uint8_t*)&g.gyro.z, 4);
  }
}