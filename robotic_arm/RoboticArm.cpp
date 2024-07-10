
#include "RoboticArm.h"

//-------------------------------------------------
//  Move directly to the given position
//-------------------------------------------------
void RoboticArm::moveDirectlyTo(float x, float y, float z)
{
  float baseAngle, shoulderAngle, elbowAngle;

  if (solve(x, y, z, baseAngle, shoulderAngle, elbowAngle))
  {
    setBaseAngle(baseAngle);
    setShoulderAngle(shoulderAngle);
    setElbowAngle(elbowAngle);
    this->mX = x;
    this->mY = y;
    this->mZ = z;
  }
}

//--------------------------------------------------------------
//  Move smoothly to the given position (using interpolation)
//--------------------------------------------------------------
void RoboticArm::moveSmoothlyTo(float x, float y, float z, int step_size, int delay_time)
{
  if(this->firstMove)
  {
    this->firstMove = false;
    moveDirectlyTo(x, y, z);
    delay(100);
    return;
  }

  float x0 = this->mX;
  float y0 = this->mY;
  float z0 = this->mZ;
  float dist = sqrt((x0 - x)*(x0 - x) + (y0 - y)*(y0 - y) + (z0 - z)*(z0 - z));
      
  if(dist > step_size)
  {
    for (int i = 0; i < dist; i += step_size)
    {
      moveDirectlyTo(x0 + (x - x0)*i / dist, y0 + (y - y0) * i / dist, z0 + (z - z0) * i / dist);
      delay(delay_time);
    }
  }
  moveDirectlyTo(x, y, z);
  delay(delay_time);
}

//----------------------------------------------------------------------------------
//  Set the posistions of the joint's servo motors to the given angle (in degrees),
//  Taking into account the physical limits of the Robotic Arm,
//  And also updating the current position of the end-effector (x, y, z),
//  THIS IS THE MOST SAFE FUNCTION TO USE
//----------------------------------------------------------------------------------
void RoboticArm::setBaseAngle(float angle)
{
  baseServo.setPositionAngle(angle);

  // update position of the end-effector (x, y, z)
  unsolve(baseServo.getAngle(), shoulderServo.getAngle(), elbowServo.getAngle(), mX, mY, mZ);
}

void RoboticArm::setShoulderAngle(float angle)
{
  shoulderServo.setPositionAngle(angle);

  // update position of the end-effector (x, y, z)
  unsolve(baseServo.getAngle(), shoulderServo.getAngle(), elbowServo.getAngle(), mX, mY, mZ);
}

void RoboticArm::setElbowAngle(float angle)
{
  elbowServo.setPositionAngle(angle);

  // update position of the end-effector (x, y, z)
  unsolve(baseServo.getAngle(), shoulderServo.getAngle(), elbowServo.getAngle(), mX, mY, mZ);
}

void RoboticArm::setGripperAngle(float angle)
{
  gripperServo.setPositionAngle(angle);
}

//----------------------------------------------------------------------------------
//  Move the posistions of the joint's servo motors by the given angle (in degrees),
//  Taking into account the physical limits of the Robotic Arm,
//  And also updating the current position of the end-effector (x, y, z),
//  THIS IS THE MOST SAFE FUNCTION TO USE
//----------------------------------------------------------------------------------
void RoboticArm::moveBaseByAngle(float angle)
{
  setBaseAngle(baseServo.getAngle() + angle);
}

void RoboticArm::moveSoulderByAngle(float angle)
{
  setShoulderAngle(shoulderServo.getAngle() + angle);
}

void RoboticArm::moveElbowByAngle(float angle)
{
  setElbowAngle(elbowServo.getAngle() + angle);
}

void RoboticArm::moveGripperByAngle(float angle)
{
  setGripperAngle(gripperServo.getAngle() + angle);
}

//--------------------------------------------------------------
//  Set the lengths of the arms (L1, L2 and L3),
//  These lengths are used by inverse kinematics calculations
//--------------------------------------------------------------
void RoboticArm::setArmLengths(float _L1, float _L2, float _L3)
{
  this->L1 = _L1;
  this->L2 = _L2;
  this->L3 = _L3;
}

//------------------------------------------------------------------------------
// inverse kinematics, get joint's Angle's from the given position (x, y, z)
//------------------------------------------------------------------------------
bool RoboticArm::solve(float x, float y, float z, float& a0, float& a1, float& a2)
{
  // Solve top-down view
  float r, theta;
  cart2polar(x, y, r, theta);

  // Account for the wrist length!
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
  z = v;
}

//---------------------------------
//  Utility Functions
//---------------------------------

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

  if(den==0) return false;
  float c = (adj1*adj1 + adj2*adj2 - opp*opp)/den;

  if(c>1 || c<-1) return false;

  theta = acos(c);

  return true;
}

