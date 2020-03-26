#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "MS5837.h"

#define MOTOR_FL 2
#define MOTOR_FR 3
#define MOTOR_BL 4
#define MOTOR_BR 5
#define MOTOR_TA 6


Servo motor_fl, motor_fr, motor_bl, motor_br, motor_ta;
MS5837 depth_sensor;

void setup() {
  Wire.begin();
  Serial.begin(19200);

  pinMode(MOTOR_FL, OUTPUT);
  pinMode(MOTOR_FR, OUTPUT);
  pinMode(MOTOR_BL, OUTPUT);
  pinMode(MOTOR_BR, OUTPUT);
  pinMode(MOTOR_TA, OUTPUT);

  motor_fl.attach(MOTOR_FL);
  motor_fr.attach(MOTOR_FR);
  motor_bl.attach(MOTOR_BL);
  motor_br.attach(MOTOR_BR);
  motor_ta.attach(MOTOR_TA);

  motor_fl.writeMicroseconds(1500);
  motor_fr.writeMicroseconds(1500);
  motor_bl.writeMicroseconds(1500);
  motor_br.writeMicroseconds(1500);
  motor_ta.writeMicroseconds(1500);

  while (!depth_sensor.init()) {
    Serial.println("message:Init failed!");
    Serial.println("message:Are SDA/SCL connected correctly?");
    Serial.println("message:Blue Robotics Bar30: White=SDA, Green=SCL");
    delay(5000);
  }
  
  depth_sensor.setModel(MS5837::MS5837_02BA);
  depth_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop()
{
  // Reads the Serial input (If any) from the tx2, parses it and sends the signals to the motors
  if (Serial.available()) {
    int motor_fl_val = Serial.readStringUntil(',').toInt();
    int motor_fr_val = Serial.readStringUntil(',').toInt();
    int motor_bl_val = Serial.readStringUntil(',').toInt();
    int motor_br_val = Serial.readStringUntil(',').toInt();
    int motor_ta_val = Serial.readStringUntil('\n').toInt();

    motor_fl.writeMicroseconds(motor_fl_val);
    motor_fr.writeMicroseconds(motor_fr_val);
    motor_bl.writeMicroseconds(motor_bl_val);
    motor_br.writeMicroseconds(motor_br_val);
    motor_ta.writeMicroseconds(motor_ta_val);
    
    Serial.flush();
  }

  depth_sensor.read();
  Serial.print("pressure:"); 
  Serial.println(depth_sensor.pressure());
}