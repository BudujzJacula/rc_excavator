#include <Arduino.h>
#include <SPI.h>
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"

// default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello, World!");
  pwm.setPWM(15, 0, 4096);
  delay(1000);
  pwm.setPWM(15, 4096, 0);
  delay(1000);
}