/*
  Robotic Arm using Arduino & Bluetooth
  By : Ali Mustafa Kamel, 2024 Feb-July
*/


#include <Arduino.h>
#include <SoftwareSerial.h>

#include "RoboticArm.h"
#include "ServoMotor.h"
#include "ByteQueue.h"


// Android App commands ID's, set to the same values in the Android App
enum Command
{
  SET_POSITION = 1,
  BUTTON_PRESSED = 2,
  BUTTON_RELEASED = 3
};

// an array containing the state for each of the (8) buttons in Android App
bool isButtonDown[8];

// Android App buttons ID's, used as index to the above array "isButtonDown"
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
uint32_t timeServoUpdate;


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
  roboticArm.moveToCylindrical(90, 160, -25);
  roboticArm.setGripperAngle(90);

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
    uint8_t byte = Bluetooth.read();

    commandQueue.write(byte);
  }

  //-------------------------------------------------------
  // Read the next command from the Queue
  //-------------------------------------------------------
  if (isReady && commandQueue.size() > 0)
  {
    command = commandQueue.nextByte();

    isReady = false;
  }

  //--------------------------------
  // Execute The Recieved Commands
  //----------------------------------------------------------------------------------------
  // set the state of each button depending on the recived command from the Android App,
  // the Android App sends this command whenever a (Joystick) button is pressed.
  //----------------------------------------------------------------------------------------
  if (command == Command::BUTTON_PRESSED)
  {
    // this command is expected to be sent with additional (1 byte), representing the ID of the button
    if (commandQueue.size() >= 1)
    {
      int8_t button_id = commandQueue.nextByte();

      // button is held down
      isButtonDown[button_id] = true;

      // ready for the next command
      isReady = true;
    }
  }
  else if (command == Command::BUTTON_RELEASED)
  {
    // this command is expected to be sent with additional (1 byte), representing the ID of the button
    if (commandQueue.size() >= 1)
    {
      int8_t button_id = commandQueue.nextByte();

      // button is left up
      isButtonDown[button_id] = false;

      // ready for the next command
      isReady = true;
    }
  }
  //------------------------------------------------------------------------------------------
  // set the position of the end-effector of the Robotic Arm (in cylindrical coordinates),
  // the Android App sends this command when (Sliders) are changed.
  //------------------------------------------------------------------------------------------
  else if (command == Command::SET_POSITION)
  {
    // this command is expected to be sent with additional (8 bytes),
    // 1st (2 bytes) : 16-bit value of Base angle (Theta)
    // 2nd (2 bytes) : 16-bit value of Radius
    // 3rd (2 bytes) : 16-bit value of Height (Z)
    // 4th (2 bytes) : 16-bit value of (Gripper) angle
    if (commandQueue.size() >= 8)
    {
      int16_t theta = commandQueue.nextInt_2_Bytes();
      int16_t radius = commandQueue.nextInt_2_Bytes();
      int16_t z = commandQueue.nextInt_2_Bytes();
      int16_t gripper_angle = commandQueue.nextInt_2_Bytes();

      roboticArm.moveToCylindrical(theta, radius, z);
      roboticArm.setGripperAngle(gripper_angle);

      // ready for the next command
      isReady = true;
    }
  }

  //-----------------------------------------------------------------
  // move the robotic arm depending on which buttons are held down
  //---------------------------------------------------------------------------------------
  // Repeat every (60 milliseconds), to give the servo motors the chance to move smoothly
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
      roboticArm.moveByCylindrical(-2, 0, 0);
    }
    if (isButtonDown[ButtonID::BUTTON_LEFT])
    {
      roboticArm.moveByCylindrical(2, 0, 0);
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
      roboticArm.moveGripperByAngle(5);
    }
    if (isButtonDown[ButtonID::BUTTON_GRIPPER_CLOSE])
    {
      roboticArm.moveGripperByAngle(-5);
    }

    timeServoUpdate = millis();
  }
}


