#include <Arduino.h>
#include <SPI.h>
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include <Bluepad32.h>

#define RXD1 16
#define TXD1 17

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
ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

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

void drive_actuator(Actuators motor, uint8_t direction, uint16_t speed) {
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

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, l1:%1x, r1:%1x, b:%1x\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        //
        ctl->l1(),
        // ctl->l2(),
        ctl->r1(),
        // ctl->r2(),
        ctl->b()
    );
}

void processGamepad(ControllerPtr gamepad) {
  if (gamepad->a())
  {
    Serial.println("BUTTON A");
  }

  if (gamepad->b())
  {
    Serial.println("BUTTON B");
  }

  if (gamepad->x())
  {
    Serial.println("BUTTON X");
  }

  if (gamepad->y())
  {
    Serial.println("BUTTON Y");
  }
  
  char buf[256];
  snprintf(buf, sizeof(buf) - 1,
           "idx=%d, dpad: 0x%02x, buttons: 0x%04x, "
           "axis L: %4li, %4li, axis R: %4li, %4li, "
           "brake: %4ld, throttle: %4li, misc: 0x%02x, "
           "gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d, "
           "battery: %d",
           gamepad->index(),        // Gamepad Index
           gamepad->dpad(),         // DPad
           gamepad->buttons(),      // bitmask of pressed buttons
           gamepad->axisX(),        // (-511 - 512) left X Axis
           gamepad->axisY(),        // (-511 - 512) left Y axis
           gamepad->axisRX(),       // (-511 - 512) right X axis
           gamepad->axisRY(),       // (-511 - 512) right Y axis
           gamepad->brake(),        // (0 - 1023): brake button
           gamepad->throttle(),     // (0 - 1023): throttle (AKA gas) button
           gamepad->miscButtons(),  // bitmask of pressed "misc" buttons
           gamepad->gyroX(),        // Gyro X
           gamepad->gyroY(),        // Gyro Y
           gamepad->gyroZ(),        // Gyro Z
           gamepad->accelX(),       // Accelerometer X
           gamepad->accelY(),       // Accelerometer Y
           gamepad->accelZ(),       // Accelerometer Z
           gamepad->battery()       // 0=Unknown, 1=empty, 255=full
  );
  Serial.println(buf);
}

void setup() {
  HardwareSerial Serial1(2);
  Serial1.begin(115200);
  // Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  // pwm.begin();
  // pwm.setPWMFreq(1000);

  while (!Serial1)
  {
    ;
  }
  Serial1.println("SETUP");

  String fv = BP32.firmwareVersion();
  Serial.println("Firmware version installed: ");
  Serial.println(fv);

  const uint8_t* addr = BP32.localBdAddress();
  Serial.print("BD Address: ");
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(addr[i], HEX);
    if (i < 5)
      Serial.print(":");
  }
  Serial.println();

  BP32.setup(&onConnectedController, &onDisconnectedController);

  BP32.forgetBluetoothKeys();
}

void loop() {
  Serial1.println("Hello, World!");

  // This call fetches all the controllers' data.
  // Call this function in your main loop.

  /*
  BUG in Arduino IDE bluepad32 is a different library and thus this code doesnt compile and run from platformio
  BUG Because of that, the code development is continued as a separate project in Arduino IDE
  */
  bool dataUpdated = BP32.update();
  if (dataUpdated)
      processControllers();

  // BP32.update();

  // for (int i = 0; i < BP32_MAX_CONTROLLERS; i++)
  // {
  //   ControllerPtr myController = myControllers[i];
  //   Serial.println("Looping...");

  //   if (myController && myController->isConnected())
  //   {
  //     if (myController->isGamepad())
  //     {
  //       processGamepad(myController);
  //       Serial.println("Gamepad");
  //   }
  // }
  
  delay(150);

  // drive_actuator(LEFT_DRIVE, 0, 2048);
  // drive_actuator(RIGHT_DRIVE, 0, 2048);
  // drive_actuator(SWING_MOTOR, 0, 2048);
  // drive_actuator(ARM, 0, 2048);
  // drive_actuator(DIPPER, 0, 2048);
  // drive_actuator(BUCKET, 0, 2048);
  // drive_actuator(THUMB, 0, 2048);
  // drive_actuator(AUX, 0, 2048);
  // delay(1000);
  // drive_actuator(LEFT_DRIVE, 1, 2048);
  // drive_actuator(RIGHT_DRIVE, 1, 2048);
  // drive_actuator(SWING_MOTOR, 1, 2048);
  // drive_actuator(ARM, 1, 2048);
  // drive_actuator(DIPPER, 1, 2048);
  // drive_actuator(BUCKET, 1, 2048);
  // drive_actuator(THUMB, 1, 2048);
  // drive_actuator(AUX, 1, 2048);
  // delay(1000);
}
}