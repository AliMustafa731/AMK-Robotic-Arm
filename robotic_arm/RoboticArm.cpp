/*
  Robotic Arm using Arduino & Bluetooth
  By : Ali Mustafa Kamel, 2024 Feb-July
*/


#include "RoboticArm.h"

//---------------------------------
// Initialize the Robotic Arm
//---------------------------------
void RoboticArm::begin(int base_pin, int shoulder_pin, int elbow_pin, int gripper_pin, float _L1, float _L2, float _L3, float _L4)
{
  // initialize the PCA9685 Servo Driver
  pwmDriver = Adafruit_PWMServoDriver(0x40);
  pwmDriver.begin();
  pwmDriver.setPWMFreq(50);

  setArmLengths(_L1, _L2, _L3, _L4);

  // set the safe (min / max) limits of reachable  positions (in cylindrical coordinates) (found by experimentation)
  setLimitsCylindrical(0, 180, 110, 220, -100, -25);

  // initialize the servo motors
  // with calibration values of the servo motors for (PWM-to-Angle) Conversions (found by experimentation)
  // DO NOT CHANGE THESE VALUES
  baseServo.begin(base_pin, &pwmDriver, 116, 540, 0, 180);
  shoulderServo.begin(shoulder_pin, &pwmDriver, 537, 106, 0, 180);
  elbowServo.begin(elbow_pin, &pwmDriver, 341, 553, 90, 180);
  gripperServo.begin(gripper_pin, &pwmDriver, 120, 570, 0, 180);

  // set the safe (min / max) limits of the servo angles (found by experimentation)
  // DO NOT CHANGE THESE VALUES
  baseServo.setLimits(0, 180);
  shoulderServo.setLimits(40, 120);
  elbowServo.setLimits(90, 180);
  gripperServo.setLimits(70, 115);

  // set initial position
  moveToCylindrical(90, 160, 50);
}

//-----------------------------------------------------------------------
//  Move directly to the given position (in Cartesian coordinates)
//-----------------------------------------------------------------------
void RoboticArm::moveTo(float x, float y, float z)
{
  float radius, theta;

  // convert to polar coordinates
  cart2polar(x, y, radius, theta);
  
  theta = RAD_TO_DEG(theta);

  moveToCylindrical(theta, radius, z);
}

//----------------------------------------------------------------------------
//  Move the end-effector from it's current position by the given (x, y, z)
//----------------------------------------------------------------------------
void RoboticArm::moveBy(float x, float y, float z)
{
  moveTo(this->mX + x, this->mY + y, this->mZ + z);
}

//---------------------------------------------------------------------------------------
//  Move the end-effector directly to the given position in Cylindrical coordinates
//---------------------------------------------------------------------------------------
void RoboticArm::moveToCylindrical(float theta, float radius, float z)
{
  // safety limit
  theta = max(min_theta, min(theta, max_theta));
  radius = max(min_radius, min(radius, max_radius));
  z = max(min_Z, min(z, max_Z));

  float baseAngle, shoulderAngle, elbowAngle;
  float x, y;

  polar2cart(radius, DEG_TO_RAD(theta), x, y);

  if (solve(x, y, z, baseAngle, shoulderAngle, elbowAngle))
  {
    baseServo.setPositionAngle(baseAngle);
    shoulderServo.setPositionAngle(shoulderAngle);
    elbowServo.setPositionAngle(elbowAngle);
    this->mX = x;
    this->mY = y;
    this->mZ = z;
    this->Theta = theta;
    this->Radius = radius;
  }
}

//--------------------------------------------------------------------------------------------
//  Move the end-effector from it's current position by the given Cylindrical coordinates
//--------------------------------------------------------------------------------------------
void RoboticArm::moveByCylindrical(float theta, float radius, float z)
{
  moveToCylindrical(this->Theta + theta, this->Radius + radius, this->mZ + z);
}

//--------------------------------------------------------------
//  Set the lengths of the arms (L1, L2 and L3),
//  These lengths are used by inverse kinematics calculations
//--------------------------------------------------------------
void RoboticArm::setArmLengths(float _L1, float _L2, float _L3, float _L4)
{
  this->L1 = _L1;
  this->L2 = _L2;
  this->L3 = _L3;
  this->L4 = _L4;
}

//---------------------------------------------------------------------------------------------
// set safe (min / max) limits of positions that can be reached (found by experimentation)
//---------------------------------------------------------------------------------------------
void RoboticArm::setLimitsCylindrical(float min_theta, float max_theta, float min_radius, float max_radius, float min_z, float max_z)
{
  this->min_theta = min_theta;
  this->max_theta = max_theta;
  this->min_radius = min_radius;
  this->max_radius = max_radius;
  this->min_Z = min_z;
  this->max_Z = max_z;
}

//---------------------
// Getters
//---------------------
float RoboticArm::getX() { return this->mX; }
float RoboticArm::getY() { return this->mY; }
float RoboticArm::getZ() { return this->mZ; }
float RoboticArm::getRadius() { return this->Radius; }
float RoboticArm::getTheta() { return this->Theta; }

//------------------------------------------------------------------------------
// inverse kinematics, get joint's Angle's from the given position (x, y, z)
//------------------------------------------------------------------------------
bool RoboticArm::solve(float x, float y, float z, float& a0, float& a1, float& a2)
{
  // Account for the gripper height
  z += L4;

  // Solve top-down view
  float r, theta;
  cart2polar(x, y, r, theta);

  // Account for the wrist length
  r -= L3;

  // In arm plane, convert to polar
  float A, R;
  cart2polar(r, z, R, A);

  // Solve arm inner angles as required
  float B, C;
  if (!cosangle(L2, L1, R, B)) return false;
  if (!cosangle(R, L1, L2, C)) return false;

  // Solve for servo angles from horizontal
  a0 = RAD_TO_DEG(theta);
  a1 = RAD_TO_DEG(A + B);
  a2 = RAD_TO_DEG(A + B + C);

  return true;
}

//---------------------------------------------------------------------------------------
// forward kinematics, get end effector position (x, y, z) from the given joint's Angle's
//---------------------------------------------------------------------------------------
void RoboticArm::unsolve(float a0, float a1, float a2, float& x, float& y, float& z)
{
  a0 = DEG_TO_RAD(a0);
  a1 = DEG_TO_RAD(a1);
  a2 = DEG_TO_RAD(a2);

  // Calculate u,v coords for arm
  float u01, v01, u12, v12;
  polar2cart(L1, a1, u01, v01);
  polar2cart(L2, a2, u12, v12);

  // Add vectors
  float u, v;
  u = u01 + u12 + L3;
  v = v01 + v12;

  // Calculate in 3D space
  polar2cart(u, a0, x, y);
  z = v - L4;
}

//-------------------------------------------------------
//  convert cartesian coordinates into polar coordinates
//-------------------------------------------------------
void cart2polar(float a, float b, float& r, float& theta)
{
  // Determine magnitude of cartesian coords
  r = sqrt(a*a + b*b);

  // Don't try to calculate zero-magnitude vectors' angles
  if(r == 0) return;

  float c = a / r;
  float s = b / r;

  // Safety!
  if(s > 1) s = 1;
  if(c > 1) c = 1;
  if(s < -1) s = -1;
  if(c < -1) c = -1;

  // Calculate angle in 0..PI
  theta = acos(c);

  // Convert to full range
  if(s < 0) theta *= -1;
}

//-------------------------------------------------------
//  convert polar coordinates into cartesian coordinates
//-------------------------------------------------------
void polar2cart(float r, float theta, float& a, float& b)
{
  a = r * cos(theta);
  b = r * sin(theta);
}

//--------------------------------------------------------------
//  Get angle (in radians) from a triangle using cosine rule
//--------------------------------------------------------------
bool cosangle(float opp, float adj1, float adj2, float& theta)
{
  // Cosine rule:
  // C^2 = A^2 + B^2 - 2*A*B*cos(angle_AB)
  // cos(angle_AB) = (A^2 + B^2 - C^2)/(2*A*B)
  // C is opposite
  // A, B are adjacent
  float den = 2*adj1*adj2;

  if(den == 0) return false;
  float c = (adj1*adj1 + adj2*adj2 - opp*opp)/den;

  if(c>1 || c<-1) return false;

  theta = acos(c);

  return true;
}

