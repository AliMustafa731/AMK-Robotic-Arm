/*
  Robotic Arm using Arduino & Bluetooth
  By : Ali Mustafa Kamel, 2024 Feb-July
*/


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

  // Current position of the end-effector : Cartesian (x, y, z) and Cylindrical (Theta, Radius, z)
  float mX, mY, mZ;
  float Theta, Radius;

  // safe (min / max) limits of positions that can be reached (in Cylindrical coordinates) (found by experimentation)
  float min_theta, max_theta, min_radius, max_radius, min_Z, max_Z;

  /*---------------------------------------------------------------------------------
    Arm Lengths (in millimeters) :
    L1 : Shoulder to elbow length.
    L2 : Elbow to wrist length.
    L3 : Length from wrist to hand PLUS base centre to shoulder.
    L4 : Length from the top of the Gripper to the bottom.
  ----------------------------------------------------------------------------------*/
  float L1, L2, L3, L4;
  
  // PCA9685 Servo Driver Handle
  Adafruit_PWMServoDriver pwmDriver;
  
public:

  /*--------------------------------
    Servo Motors objects
  --------------------------------*/
  ServoMotor baseServo;
  ServoMotor shoulderServo;
  ServoMotor elbowServo;
  ServoMotor gripperServo;

  RoboticArm(){}

  //---------------------------------
  // Initialize the Robotic Arm
  //---------------------------------
  void begin(int base_pin, int shoulder_pin, int elbow_pin, int gripper_pin, float _L1 = 140, float _L2 = 140, float _L3 = 10, float _L4 = 150);

  //--------------------------------------------------------------------
  //  Move the end-effector directly to the given position (x, y, z)
  //--------------------------------------------------------------------
  void moveTo(float x, float y, float z);

  //----------------------------------------------------------------------------
  //  Move the end-effector from it's current position by the given (x, y, z)
  //----------------------------------------------------------------------------
  void moveBy(float x, float y, float z);

  //---------------------------------------------------------------------------------------
  //  Move the end-effector directly to the given position in Cylindrical coordinates
  //---------------------------------------------------------------------------------------
  void moveToCylindrical(float theta, float radius, float z);

  //--------------------------------------------------------------------------------------------
  //  Move the end-effector from it's current position by the given Cylindrical coordinates
  //--------------------------------------------------------------------------------------------
  void moveByCylindrical(float theta, float radius, float z);

  //--------------------------------------------------------------
  //  Set the lengths of the arms (L1, L2 and L3),
  //  These lengths are used by kinematics calculations
  //--------------------------------------------------------------
  void setArmLengths(float _L1, float _L2, float _L3, float _L4);

  //---------------------------------------------------------------------------------------------
  // set safe (min / max) limits of positions that can be reached (found by experimentation)
  //---------------------------------------------------------------------------------------------
  void setLimitsCylindrical(float min_theta, float max_theta, float min_radius, float max_radius, float min_z, float max_z);

  //------------------------------------------------------------------------------
  // inverse kinematics, get joint's Angle's from the given position (x, y, z)
  //------------------------------------------------------------------------------
  bool solve(float x, float y, float z, float& a0, float& a1, float& a2);

  //---------------------------------------------------------------------------------------
  // forward kinematics, get end effector position (x, y, z) from the given joint's Angle's
  //---------------------------------------------------------------------------------------
  void unsolve(float a0, float a1, float a2, float& x, float& y, float& z);

  //---------------------
  // Getters
  //---------------------
  float getX();
  float getY();
  float getZ();
  float getRadius();
  float getTheta();
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
