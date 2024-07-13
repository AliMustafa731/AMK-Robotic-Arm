/*
  Robotic Arm using Arduino & Bluetooth
  By : Ali Mustafa Kamel, 2024 Feb-July
*/


#include "ServoMotor.h"

//---------------------------------
// Initialize the Servo Motor
//---------------------------------
void ServoMotor::begin(int _pinNumber, Adafruit_PWMServoDriver* _pwmDriver, float _min_pwm, float _max_pwm, float _min_angle, float _max_angle)
{
  this->pinNumber = _pinNumber;
  this->pwmDriver = _pwmDriver;
  this->angle = 0;
  this->pwm = 0;

  setRanges(_min_pwm, _max_pwm, _min_angle, _max_angle);
  setLimits(-45, 225);
}

//-------------------------------------------------
//  Set the ranges for (PWM-to-Angle) Conversions
//-------------------------------------------------
void ServoMotor::setRanges(float _min_pwm, float _max_pwm, float _min_angle, float _max_angle)
{
  this->min_pwm = _min_pwm;
  this->max_pwm = _max_pwm;
  this->min_angle = _min_angle;
  this->max_angle = _max_angle;
}

//-------------------------------------------------
// set the safe (min / max) limits of the servo
//-------------------------------------------------
void ServoMotor::setLimits(float _min_sweep_angle, float _max_sweep_angle)
{
  this->min_sweep_angle = _min_sweep_angle;
  this->max_sweep_angle = _max_sweep_angle;
}

//----------------------------------------------------------------------
// set the position of the servo from the given PWM (in microseconds)
//----------------------------------------------------------------------
void ServoMotor::setPositionPWM(int _pwm)
{
  // convert PWM to Angle
  this->angle = min_angle + (float(_pwm) - min_pwm) * (max_angle - min_angle) / (max_pwm - min_pwm);
  
  // limit the angle
  this->angle = max(min_sweep_angle, min(angle, max_sweep_angle));

  // convert Angle back to PWM after limiting
  this->pwm = int(min_pwm + (this->angle - min_angle) * (max_pwm - min_pwm) / (max_angle - min_angle));

  // set PWM position of the servo on the PCA9685 Servo Driver
  this->pwmDriver->setPWM(this->pinNumber, 0, this->pwm);
}

//----------------------------------------------------------------------
// set the position of the servo from the given Angle (in degrees)
//----------------------------------------------------------------------
void ServoMotor::setPositionAngle(float _angle)
{
  // limit the angle
  this->angle = max(min_sweep_angle, min(_angle, max_sweep_angle));

  // convert Angle to PWM
  this->pwm = int(min_pwm + (this->angle - min_angle) * (max_pwm - min_pwm) / (max_angle - min_angle));

  // set PWM position of the servo on the PCA9685 Servo Driver
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

