#include "StringSplitter.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

const int FLEX_PIN[4] = {20,21,22,23};

const int MOTOR[2] = {9,10};

const float VCC = 3.24;
const float R_DIV = 47000.0;

const float STRAIGHT_RESISTANCE[4] = {30000.0,31000.0,30000.0,30000.0}; // resistance when straight
const float BEND_RESISTANCE[4] = {70000.0,77000.0,70000.0,70000.0};

int motorPWM[2] = {0,0};

float bendAngle[4] = {0.0,0.0,0.0,0.0};

void setup(void) {
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  if (!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  bno.setExtCrystalUse(true);

  // Flex sensor outputs
  for(int i=0;i<4;i++){
    pinMode(FLEX_PIN[i],INPUT);
  }
  // Motor sensor outputs
  for(int i=0;i<2;i++){
    pinMode(MOTOR[i],INPUT);
  }
}

void loop(void) {
  /*
   * BNO-055 Sensor Euler angular data
   * euler.x()
   * euler.y()
   * euler.z()
   */

  if(Serial.available() > 0){
    String tempString = Serial.readStringUntil('\n');
    StringSplitter *splitter = new StringSplitter(tempString, ',', 2);
    motorPWM[0] = splitter->getItemAtIndex(0).toInt();
    motorPWM[1] = splitter->getItemAtIndex(1).toInt();
  }

  for(int i=0;i<2;i++) analogWrite(MOTOR[i], motorPWM[i]);
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  for(int i=0;i<4;i++){
    bendAngle[i] = bendSensor(i);
  }
  
  /*
   * Serial Handler
   */

  Serial.println(String(euler.x())+","+String(euler.y())+","+String(euler.z())+","+String(bendAngle[0])+","+String(bendAngle[1]));

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

float bendSensor(int num){
  int flexADC = analogRead(FLEX_PIN[num]);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  //Serial.println("Resistance: " + String(flexR) + " ohms");

  float bendAngle = map(flexR, STRAIGHT_RESISTANCE[num], BEND_RESISTANCE[num],
                     0, 90.0);
  //Serial.println("Bend: " + String(bendAngle) + " degrees");
  return bendAngle;
}
