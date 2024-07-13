
#include "ServoMotor.h"

//---------------------------------
// Initialize the Servo Motor
//---------------------------------
void ServoMotor::begin(int _pinNumber, Adafruit_PWMServoDriver* _pwmDriver, int pwm_min, int pwm_max, float angle_min, float angle_max)
{
  this->pinNumber = _pinNumber;
  this->pwmDriver = _pwmDriver;
  this->angle = 0;
  this->pwm = 0;

  setRanges(pwm_min, pwm_max, angle_min, angle_max);
  setLimits(0, 180);
}

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

//-------------------------------------------------
// set the safe (min / max) limits of the servo
//-------------------------------------------------
void ServoMotor::setLimits(float _min_angle, float _max_angle)
{
  this->min_angle = _min_angle;
  this->max_angle = _max_angle;
}

//----------------------------------------------------------------------
// set the position of the servo from the given PWM (in microseconds)
//----------------------------------------------------------------------
void ServoMotor::setPositionPWM(int _pwm)
{
  this->angle = (float(_pwm) - this->zero) / this->gain;
  
  this->angle = max(min_angle, min(angle, max_angle));
  this->pwm = int(0.5f + this->zero + this->gain * this->angle);

  this->pwmDriver->setPWM(this->pinNumber, 0, this->pwm);
}

//----------------------------------------------------------------------
// set the position of the servo from the given Angle (in degrees)
//----------------------------------------------------------------------
void ServoMotor::setPositionAngle(float _angle)
{
  _angle = max(min_angle, min(_angle, max_angle));

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
void ServoMotor::setPin(int _pinNumber) { this->pinNumber = _pinNumber; }
float ServoMotor::getAngle() { return this->angle; }
int ServoMotor::getPin() { return this->pinNumber; }
int ServoMotor::getPWM() { return this->pwm; }

