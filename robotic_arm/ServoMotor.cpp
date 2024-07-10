
#include "ServoMotor.h"

//-------------------------------------------------
//  Set the ranges for (PWM-to-Angle) Conversions
//-------------------------------------------------
void ServoMotor::setRanges(int pwm_min, int pwm_max, float angle_min, float angle_max)
{
  float pwm_range = pwm_max - pwm_min;
  float angle_range = angle_max - angle_min;

  this->gain = pwm_range / angle_range;
  this->zero = pwm_min - this->gain * angle_min;
}

//----------------------------------------------------------------------
// set the position of the servo from the given PWM (in microseconds)
//----------------------------------------------------------------------
void ServoMotor::setPositionPWM(int _pwm)
{
  this->pwm = _pwm;
  this->angle = (float(this->pwm) - this->zero) / this->gain;

  this->pwmDriver->setPWM(this->pinNumber, 0, this->pwm);
}

//----------------------------------------------------------------------
// set the position of the servo from the given Angle (in degrees)
//----------------------------------------------------------------------
void ServoMotor::setPositionAngle(float _angle)
{
  if (_angle > 180)
  {
    _angle = 180;
  }
  if (_angle < 0)
  {
    _angle = 0;
  }

  this->angle = _angle;
  this->pwm = int(0.5f + this->zero + this->gain * this->angle);

  this->pwmDriver->setPWM(this->pinNumber, 0, this->pwm);
}

//----------------------------------------------------------------------
// move from current position by the given PWM (in microseconds)
//----------------------------------------------------------------------
void ServoMotor::moveByPWM(int _pwm)
{
  this->setPositionPWM(this->pwm + _pwm);
}

//----------------------------------------------------------------------
// move from current position by the given Angle (in degrees)
//----------------------------------------------------------------------
void ServoMotor::moveByAngle(float _angle)
{
  this->setPositionAngle(this->angle + _angle);
}

//-----------------------------------------------
//  Setters and Getters
//-----------------------------------------------
void ServoMotor::setPin(int _pinNumber)
{
  this->pinNumber = _pinNumber;
}

int ServoMotor::getPin()
{
  return this->pinNumber;
}

float ServoMotor::getAngle()
{
  return this->angle;
}

int ServoMotor::getPWM()
{
  return this->pwm;
}

