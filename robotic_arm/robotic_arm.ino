
#include <Arduino.h>
#include "RoboticArm.h"
#include "ServoMotor.h"

#include <SoftwareSerial.h>

// Create SoftwareSerial object for Bluetooth module at digital pins (RX, TX -> 10, 11)
// you can use any digital pins
SoftwareSerial Bluetooth(10, 11);

// Android app commands ID's, set to the same values in the android app
enum Command
{
  BUTTON_FORWARD_PRESSED = 1,
  BUTTON_FORWARD_RELEASED = 2,

  BUTTON_BACKWARD_PRESSED = 3,
  BUTTON_BACKWARD_RELEASED = 4,

  BUTTON_RIGHT_PRESSED = 5,
  BUTTON_RIGHT_RELEASED = 6,

  BUTTON_LEFT_PRESSED = 7,
  BUTTON_LEFT_RELEASED = 8,

  BUTTON_UP_PRESSED = 9,
  BUTTON_UP_RELEASED = 10,

  BUTTON_DOWN_PRESSED = 11,
  BUTTON_DOWN_RELEASED = 12,

  BUTTON_GRIPPER_OPEN_PRESSED = 13,
  BUTTON_GRIPPER_OPEN_RELEASED = 14,

  BUTTON_GRIPPER_CLOSE_PRESSED = 15,
  BUTTON_GRIPPER_CLOSE_RELEASED = 16
};

// an array containing the state for each of the (8) buttons
bool isButtonDown[8];

// Android app buttons ID's, used as index to the above array "isButtonDown"
enum ButtonID
{
  BUTTON_FORWARD = 0,
  BUTTON_BACKWARD = 1,
  BUTTON_RIGHT = 2,
  BUTTON_LEFT = 3,
  BUTTON_UP = 4,
  BUTTON_DOWN = 5,
  BUTTON_GRIPPER_OPEN = 6,
  BUTTON_GRIPPER_CLOSE = 7
};

// Robotic Arm object
RoboticArm roboticArm;

void setup()
{
  // initialize the Bluetooth module serial communication
  Bluetooth.begin(9600);

  // initialize the robotic arm with pin's numbers of servo motors on the PCA9685 Servo Driver
  // along with the lengths of the arms (in millimeters),
  // DO NOT CHANGE THESE VALUES
  roboticArm = RoboticArm(0, 1, 2, 3,  // servo motors pin numbers
                          140,         // L1 : Shoulder to elbow length in millimeters
                          140,         // L2 : Elbow to wrist length in millimeters
                          10);         // L3 : Length from wrist to hand PLUS base centre to shoulder in millimeters

  // calibrate the servo motors for (PWM-to-Angle) Conversions
  // DO NOT CHANGE THESE VALUES
  roboticArm.baseServo.setRanges(116, 540, 0, 180);
  roboticArm.shoulderServo.setRanges(106, 537, 0, 180);
  roboticArm.elbowServo.setRanges(341, 553, 90, 180);
  roboticArm.gripperServo.setRanges(120, 570, 0, 180);

  // set home position
  roboticArm.setBaseAngle(90);
  roboticArm.setShoulderAngle(90);
  roboticArm.setElbowAngle(180);
  roboticArm.setGripperAngle(90);
}

void loop()
{
  //-------------------------------------------------------
  // Read commands (Bytes) from the Bluetooth device
  //-------------------------------------------------------
  while(Bluetooth.available())
  {
    // read a signle byte
    int recivedData = Bluetooth.read();

    //----------------------------------------------------------------------------------------
    // set the state of each button depending on the recived command from the Android app
    //----------------------------------------------------------------------------------------

    if (recivedData == Command::BUTTON_FORWARD_PRESSED)  isButtonDown[ButtonID::BUTTON_FORWARD] = true;
    if (recivedData == Command::BUTTON_FORWARD_RELEASED) isButtonDown[ButtonID::BUTTON_FORWARD] = false;

    if (recivedData == Command::BUTTON_BACKWARD_PRESSED)  isButtonDown[ButtonID::BUTTON_BACKWARD] = true;
    if (recivedData == Command::BUTTON_BACKWARD_RELEASED) isButtonDown[ButtonID::BUTTON_BACKWARD] = false;

    if (recivedData == Command::BUTTON_RIGHT_PRESSED)  isButtonDown[ButtonID::BUTTON_RIGHT] = true;
    if (recivedData == Command::BUTTON_RIGHT_RELEASED) isButtonDown[ButtonID::BUTTON_RIGHT] = false;

    if (recivedData == Command::BUTTON_LEFT_PRESSED)  isButtonDown[ButtonID::BUTTON_LEFT] = true;
    if (recivedData == Command::BUTTON_LEFT_RELEASED) isButtonDown[ButtonID::BUTTON_LEFT] = false;

    if (recivedData == Command::BUTTON_UP_PRESSED)  isButtonDown[ButtonID::BUTTON_UP] = true;
    if (recivedData == Command::BUTTON_UP_RELEASED) isButtonDown[ButtonID::BUTTON_UP] = false;

    if (recivedData == Command::BUTTON_DOWN_PRESSED)  isButtonDown[ButtonID::BUTTON_DOWN] = true;
    if (recivedData == Command::BUTTON_DOWN_RELEASED) isButtonDown[ButtonID::BUTTON_DOWN] = false;

    if (recivedData == Command::BUTTON_GRIPPER_OPEN_PRESSED)  isButtonDown[ButtonID::BUTTON_GRIPPER_OPEN] = true;
    if (recivedData == Command::BUTTON_GRIPPER_OPEN_RELEASED) isButtonDown[ButtonID::BUTTON_GRIPPER_OPEN] = false;
    
    if (recivedData == Command::BUTTON_GRIPPER_CLOSE_PRESSED)  isButtonDown[ButtonID::BUTTON_GRIPPER_CLOSE] = true;
    if (recivedData == Command::BUTTON_GRIPPER_CLOSE_RELEASED) isButtonDown[ButtonID::BUTTON_GRIPPER_CLOSE] = false;
  }

  //-----------------------------------------------------------------
  // move the robotic arm depending on which buttons are held down
  //----------------------------------------------------------------

  if (isButtonDown[ButtonID::BUTTON_FORWARD])
  {
    roboticArm.moveSoulderByAngle(3);
  }
  if (isButtonDown[ButtonID::BUTTON_BACKWARD])
  {
    roboticArm.moveSoulderByAngle(-3);
  }

  if (isButtonDown[ButtonID::BUTTON_RIGHT])
  {
    roboticArm.moveBaseByAngle(-3);
  }
  if (isButtonDown[ButtonID::BUTTON_LEFT])
  {
    roboticArm.moveBaseByAngle(3);
  }

  if (isButtonDown[ButtonID::BUTTON_UP])
  {
    roboticArm.moveElbowByAngle(3);
  }
  if (isButtonDown[ButtonID::BUTTON_DOWN])
  {
    roboticArm.moveElbowByAngle(-3);
  }

  if (isButtonDown[ButtonID::BUTTON_GRIPPER_OPEN])
  {
    roboticArm.moveGripperByAngle(3);
  }
  if (isButtonDown[ButtonID::BUTTON_GRIPPER_CLOSE])
  {
    roboticArm.moveGripperByAngle(-3);
  }

  // sleep for (50 milliseconds), to give the servo motors a chance to move smoothly
  delay(50);
}


