
#include "RoboticArm.h"

//---------------------------------
// Initialize the Robotic Arm
//---------------------------------
void RoboticArm::begin(int base_pin, int shoulder_pin, int elbow_pin, int gripper_pin, float _L1, float _L2, float _L3, float _L4)
{
  // initialize the Servo Driver
  pwmDriver = Adafruit_PWMServoDriver(0x40);
  pwmDriver.begin();
  pwmDriver.setPWMFreq(50);

  setArmLengths(_L1, _L2, _L3, _L4);

  // initialize the servo motors
  baseServo.begin(base_pin, &pwmDriver);
  shoulderServo.begin(shoulder_pin, &pwmDriver);
  elbowServo.begin(elbow_pin, &pwmDriver);
  gripperServo.begin(gripper_pin, &pwmDriver);

  // calibrate the servo motors for (PWM-to-Angle) Conversions (found by experimentation)
  // DO NOT CHANGE THESE VALUES
  baseServo.setRanges(116, 540, 0, 180);
  shoulderServo.setRanges(537, 106, 0, 180);
  elbowServo.setRanges(341, 553, 90, 180);
  gripperServo.setRanges(120, 570, 0, 180);

  // set the safe (max / min) limits of the servo angles (found by experimentation)
  // DO NOT CHANGE THESE VALUES
  baseServo.setLimits(0, 180);
  shoulderServo.setLimits(40, 120);
  elbowServo.setLimits(90, 180);
  gripperServo.setLimits(70, 115);

  // set initial position
  moveToCylindrical(90, 160, 50);
}

//-------------------------------------------------
//  Move directly to the given position
//-------------------------------------------------
void RoboticArm::moveTo(float x, float y, float z)
{
  float baseAngle, shoulderAngle, elbowAngle;

  if (solve(x, y, z, baseAngle, shoulderAngle, elbowAngle))
  {
    baseServo.setPositionAngle(baseAngle);
    shoulderServo.setPositionAngle(shoulderAngle);
    elbowServo.setPositionAngle(elbowAngle);
    this->mX = x;
    this->mY = y;
    this->mZ = z;
  }
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
  theta = max(0, min(theta, 180));
  radius = max(110, min(radius, 220));
  z = max(-100, min(z, -25));

  float x, y;

  polar2cart(radius, DEG_TO_RAD(theta), x, y);

  float baseAngle, shoulderAngle, elbowAngle;

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

