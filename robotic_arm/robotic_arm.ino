
#include <Arduino.h>
#include <SoftwareSerial.h>

#include "RoboticArm.h"
#include "ServoMotor.h"
#include "ByteQueue.h"


// Android app commands ID's, set to the same values in the android app
enum Command
{
  SET_PWM = 1,
  SET_ANGLE = 2,
  BUTTON_PRESSED = 3,
  BUTTON_RELEASED = 4
};

// an array containing the state for each of the (8) buttons in android app
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

// SoftwareSerial object for Bluetooth module at digital pins (RX, TX -> 10, 11)
SoftwareSerial Bluetooth(10, 11);

// Robotic Arm object
RoboticArm roboticArm;

// a buffer used to store the incoming commands
ByteQueue commandQueue;
uint8_t commandBuffer[256];

// variable used to store the incoming command from the Android App
int8_t command = 0;

// flag used to indicate that we're ready to read the next command from the Commands Queue
bool isReady = true;

// a counter variable used to update the positions of the servo motors frequently
unsigned long timeServoUpdate;


//----------------------------
//  Setup
//----------------------------
void setup()
{
  // initialize the Bluetooth module serial communication
  Bluetooth.begin(9600);

  // initialize command queue
  commandQueue = ByteQueue(commandBuffer, sizeof(commandBuffer));

  // initialize the robotic arm with servo pins numbers on the PCA9685 Servo Driver
  roboticArm.begin(0, 1, 2, 3);

  // set initial position
  roboticArm.moveToCylindrical(90, 160, 50);

  timeServoUpdate = millis();
}

//----------------------------
//  Loop
//----------------------------
void loop()
{
  //----------------------------------------------------------------
  // Read (Bytes) from the Bluetooth device and store in the Queue
  //----------------------------------------------------------------
  while(Bluetooth.available() > 0 && commandQueue.free_space() > 0)
  {
    uint8_t data = Bluetooth.read();
    commandQueue.enqueue(data);
  }

  //-------------------------------------------------------
  // Read the next command from the Queue
  //-------------------------------------------------------
  if (isReady && commandQueue.size() > 0)
  {
    command = commandQueue.dequeue();

    isReady = false;
  }
  
  //----------------------------------------------------------------------------------------
  // set the state of each button depending on the recived command from the Android app,
  // the Android App sends this command whenever a (Joystick) button is pressed.
  //----------------------------------------------------------------------------------------
  if (command == Command::BUTTON_PRESSED)
  {
    // this command is expected to be sent with additional (1 byte), representing the ID of the button
    if (commandQueue.size() >= 1)
    {
      int8_t button_id = commandQueue.dequeue();

      isButtonDown[button_id] = true;

      isReady = true;
    }
  }
  else if (command == Command::BUTTON_RELEASED)
  {
    // this command is expected to be sent with additional (1 byte), representing the ID of the button
    if (commandQueue.size() >= 1)
    {
      int8_t button_id = commandQueue.dequeue();

      isButtonDown[button_id] = false;

      isReady = true;
    }
  }
  //-----------------------------------------------------------------------
  // set the PWM or Angle positions of the servo motors,
  // the Android App sends this command when (Sliders) are changed.
  //-----------------------------------------------------------------------
  else if (command == Command::SET_ANGLE)
  {
    // this command is expected to be sent with additional (8 bytes),
    // 1st (2 bytes) : 16-bit value of (Base) angle
    // 2nd (2 bytes) : 16-bit value of (Shoulder) angle
    // 3rd (2 bytes) : 16-bit value of (Elbow) angle
    // 4th (2 bytes) : 16-bit value of (Gripper) angle
    if (commandQueue.size() >= 8)
    {
      int16_t base_angle;
      int16_t shoulder_angle;
      int16_t elbow_angle;
      int16_t gripper_angle;

      commandQueue.dequeue((uint8_t*)&base_angle, sizeof(int16_t));
      commandQueue.dequeue((uint8_t*)&shoulder_angle, sizeof(int16_t));
      commandQueue.dequeue((uint8_t*)&elbow_angle, sizeof(int16_t));
      commandQueue.dequeue((uint8_t*)&gripper_angle, sizeof(int16_t));

      roboticArm.baseServo.setPositionAngle(float(base_angle));
      roboticArm.shoulderServo.setPositionAngle(float(shoulder_angle));
      roboticArm.elbowServo.setPositionAngle(float(elbow_angle));
      roboticArm.gripperServo.setPositionAngle(float(gripper_angle));

      isReady = true;
    }
  }
  else if (command == Command::SET_PWM)
  {
    // this command is expected to be sent with additional (8 bytes),
    // 1st (2 bytes) : 16-bit value of (Base) pwm
    // 2nd (2 bytes) : 16-bit value of (Shoulder) pwm
    // 3rd (2 bytes) : 16-bit value of (Elbow) pwm
    // 4th (2 bytes) : 16-bit value of (Gripper) pwm
    if (commandQueue.size() >= 8)
    {
      int16_t base_pwm;
      int16_t shoulder_pwm;
      int16_t elbow_pwm;
      int16_t gripper_pwm;

      commandQueue.dequeue((uint8_t*)&base_pwm, sizeof(int16_t));
      commandQueue.dequeue((uint8_t*)&shoulder_pwm, sizeof(int16_t));
      commandQueue.dequeue((uint8_t*)&elbow_pwm, sizeof(int16_t));
      commandQueue.dequeue((uint8_t*)&gripper_pwm, sizeof(int16_t));

      roboticArm.baseServo.setPositionPWM(base_pwm);
      roboticArm.shoulderServo.setPositionPWM(shoulder_pwm);
      roboticArm.elbowServo.setPositionPWM(elbow_pwm);
      roboticArm.gripperServo.setPositionPWM(gripper_pwm);

      isReady = true;
    }
  }

  //-----------------------------------------------------------------
  // move the robotic arm depending on which buttons are held down
  //---------------------------------------------------------------------------------------
  // Repeat every (100 milliseconds), to give the servo motors the chance to move smoothly
  //---------------------------------------------------------------------------------------
  if (millis() - timeServoUpdate >= 60)
  {
    if (isButtonDown[ButtonID::BUTTON_FORWARD])
    {
      roboticArm.moveByCylindrical(0, 5, 0);
    }
    if (isButtonDown[ButtonID::BUTTON_BACKWARD])
    {
      roboticArm.moveByCylindrical(0, -5, 0);
    }

    if (isButtonDown[ButtonID::BUTTON_RIGHT])
    {
      roboticArm.moveByCylindrical(-3, 0, 0);
    }
    if (isButtonDown[ButtonID::BUTTON_LEFT])
    {
      roboticArm.moveByCylindrical(3, 0, 0);
    }

    if (isButtonDown[ButtonID::BUTTON_UP])
    {
      roboticArm.moveByCylindrical(0, 0, 5);
    }
    if (isButtonDown[ButtonID::BUTTON_DOWN])
    {
      roboticArm.moveByCylindrical(0, 0, -5);
    }

    if (isButtonDown[ButtonID::BUTTON_GRIPPER_OPEN])
    {
      roboticArm.gripperServo.moveByAngle(5);
    }
    if (isButtonDown[ButtonID::BUTTON_GRIPPER_CLOSE])
    {
      roboticArm.gripperServo.moveByAngle(-5);
    }

    timeServoUpdate = millis();
  }
}


