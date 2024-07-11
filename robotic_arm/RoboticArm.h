#ifndef ROBOTIC_ARM_H
#define ROBOTIC_ARM_H

#include <Adafruit_PWMServoDriver.h>
#include "ServoMotor.h"

//-------------------------------------
//  Robotic Arm Structure
//-------------------------------------
class RoboticArm
{
private:

  // Current position of the end effector (x, y, z)
  float mX, mY, mZ;

  // PCA9685 Servo Driver Handle
  Adafruit_PWMServoDriver pwmDriver;

  /*---------------------------------------------------------------------------------
    Arm Lengths :
    L1 : Shoulder to elbow length in millimeters
    L2 : Elbow to wrist length in millimeters
    L3 : Length from wrist to hand PLUS base centre to shoulder in millimeters
  ----------------------------------------------------------------------------------*/
  float L1, L2, L3;

  /*-------------------------------------------------------------------------------------------
    flag indicating if the arm has just started,
    in that case, (when moving the arm) set the position immediatly,
    otherwise, there's a valid position in the position variables (mX, mY, mZ),
    so we can move the arm smoothly (using interpolation from "old" to "new" positions)
  -------------------------------------------------------------------------------------------*/
  bool firstMove;

public:

  /*--------------------------------
    Servo Motors objects
  --------------------------------*/
  ServoMotor baseServo;
  ServoMotor shoulderServo;
  ServoMotor elbowServo;
  ServoMotor gripperServo;

  RoboticArm(){}

  RoboticArm(int base_pin, int shoulder_pin, int elbow_pin, int gripper_pin, float _L1, float _L2, float _L3)
  {
    // initialize the Servo Driver
    pwmDriver = Adafruit_PWMServoDriver(0x40);
    pwmDriver.begin();
    pwmDriver.setPWMFreq(50);

    // initialize the servo motors
    baseServo = ServoMotor(base_pin, &pwmDriver);
    shoulderServo = ServoMotor(shoulder_pin, &pwmDriver);
    elbowServo = ServoMotor(elbow_pin, &pwmDriver);
    gripperServo = ServoMotor(gripper_pin, &pwmDriver);

    setArmLengths(_L1, _L2, _L3);
    firstMove = true;
  }

  //--------------------------------------------------------------------
  //  Move the end-effector directly to the given position (x, y, z)
  //--------------------------------------------------------------------
  void moveToPosition(float x, float y, float z);

  //----------------------------------------------------------------------------------
  //  Set the posistions of the joint's servo motors to the given angle (in degrees),
  //  Taking into account the physical limits of the Robotic Arm,
  //  And also updating the current position of the end-effector (x, y, z),
  //  THIS IS THE MOST SAFE FUNCTION TO USE
  //----------------------------------------------------------------------------------
  void setBaseAngle(float angle);
  void setShoulderAngle(float angle);
  void setElbowAngle(float angle);
  void setGripperAngle(float angle);

  //----------------------------------------------------------------------------------
  //  Move the posistions of the joint's servo motors by the given angle (in degrees),
  //  Taking into account the physical limits of the Robotic Arm,
  //  And also updating the current position of the end-effector (x, y, z),
  //  THIS IS THE MOST SAFE FUNCTION TO USE
  //----------------------------------------------------------------------------------
  void moveBaseByAngle(float angle);
  void moveSoulderByAngle(float angle);
  void moveElbowByAngle(float angle);
  void moveGripperByAngle(float angle);

  //--------------------------------------------------------------
  //  Set the lengths of the arms (L1, L2 and L3),
  //  These lengths are used by inverse kinematics calculations
  //--------------------------------------------------------------
  void setArmLengths(float _L1, float _L2, float _L3);

  //------------------------------------------------------------------------------
  // inverse kinematics, get joint's Angle's from the given position (x, y, z)
  //------------------------------------------------------------------------------
  bool solve(float x, float y, float z, float& a0, float& a1, float& a2);

  //---------------------------------------------------------------------------------------
  // forward kinematics, get end effector position (x, y, z) from the given joint's Angle's
  //---------------------------------------------------------------------------------------
  void unsolve(float a0, float a1, float a2, float& x, float& y, float& z);
};

//---------------------------------------------------------
//  Conversions of (degrees) to (radians) and vice versa
//---------------------------------------------------------
#define RAD_TO_DEG(x) ((x) * 180.0f / PI)
#define DEG_TO_RAD(x) ((x) * PI / 180.0f)

//-------------------------------------------------------
//  convert cartesian coordinates into polar coordinates
//-------------------------------------------------------
void cart2polar(float a, float b, float& r, float& theta);

//-------------------------------------------------------
//  convert polar coordinates into cartesian coordinates
//-------------------------------------------------------
void polar2cart(float r, float theta, float& a, float& b);

//--------------------------------------------------------------
//  Get angle (in radians) from a triangle using cosine rule
//--------------------------------------------------------------
bool cosangle(float opp, float adj1, float adj2, float& theta);

#endif  // ROBOTIC_ARM_H
