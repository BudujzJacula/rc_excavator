#include <Arduino.h>
#include <SPI.h>
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"

enum Actuators {
  LEFT_DRIVE = 1,
  RIGHT_DRIVE,
  SWING_MOTOR,
  ARM,
  DIPPER,
  BUCKET,
  THUMB,
  AUX,
};

// default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t motor_1_A = 0;
const uint8_t motor_1_B = 1;

const uint8_t motor_2_A = 2;
const uint8_t motor_2_B = 3;

const uint8_t motor_3_A = 4;
const uint8_t motor_3_B = 5;

const uint8_t motor_4_A = 6;
const uint8_t motor_4_B = 7;

const uint8_t motor_5_A = 8;
const uint8_t motor_5_B = 9;

const uint8_t motor_6_A = 10;
const uint8_t motor_6_B = 11;

const uint8_t motor_7_A = 12;
const uint8_t motor_7_B = 13;

const uint8_t motor_8_A = 14;
const uint8_t motor_8_B = 15;

void drive_motor(uint8_t pinA, uint8_t pinB, uint8_t direction, uint16_t speed) {
  if (direction == 0) {
    pwm.setPWM(pinA, speed, 0);
    pwm.setPWM(pinB, 0, 4096);
  }
  else {
    pwm.setPWM(pinA, 0, 4096);
    pwm.setPWM(pinB, speed, 0);
  }
}

void drive_axis(Actuators motor, uint8_t direction, uint16_t speed) {
  switch (motor)
  {
  case LEFT_DRIVE:
    drive_motor(motor_1_A, motor_1_B, direction, speed);
    Serial.println("LEFT_DRIVE");

    break;
  case RIGHT_DRIVE:
    drive_motor(motor_2_A, motor_2_B, direction, speed);

    break;
  case SWING_MOTOR:
    drive_motor(motor_3_A, motor_3_B, direction, speed);

    break;
  case ARM:
    drive_motor(motor_4_A, motor_4_B, direction, speed);

    break;
  case DIPPER:
    drive_motor(motor_5_A, motor_5_B, direction, speed);

    break;
  case BUCKET:
    drive_motor(motor_6_A, motor_6_B, direction, speed);

    break;
  case THUMB:
    drive_motor(motor_7_A, motor_7_B, direction, speed);

    break;
  case AUX:
    drive_motor(motor_8_A, motor_8_B, direction, speed);
    
    break;
  
  default:
    break;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello, World!");

  drive_axis(LEFT_DRIVE, 0, 2048);
  drive_axis(RIGHT_DRIVE, 0, 2048);
  drive_axis(SWING_MOTOR, 0, 2048);
  drive_axis(ARM, 0, 2048);
  drive_axis(DIPPER, 0, 2048);
  drive_axis(BUCKET, 0, 2048);
  drive_axis(THUMB, 0, 2048);
  drive_axis(AUX, 0, 2048);
  delay(1000);
  drive_axis(LEFT_DRIVE, 1, 2048);
  drive_axis(RIGHT_DRIVE, 1, 2048);
  drive_axis(SWING_MOTOR, 1, 2048);
  drive_axis(ARM, 1, 2048);
  drive_axis(DIPPER, 1, 2048);
  drive_axis(BUCKET, 1, 2048);
  drive_axis(THUMB, 1, 2048);
  drive_axis(AUX, 1, 2048);
  delay(1000);
}