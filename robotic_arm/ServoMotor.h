#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Adafruit_PWMServoDriver.h>

//---------------------------------------------------------------------
//  Servo Motor Structure to hold information (pin number, position)
//---------------------------------------------------------------------
class ServoMotor
{
private:

  int pwm;  // PWM position (in microseconds)
  float angle;  // Angle position (in degrees)
  float gain;   // PWM per radian
  float zero;   // Theoretical PWM for zero Angle

  // PCA9685 Servo Driver Handle
  Adafruit_PWMServoDriver* pwmDriver;
  int pinNumber;

public:

  ServoMotor(){}

  ServoMotor(int _pinNumber, Adafruit_PWMServoDriver* _pwmDriver, int pwm_min = 0, int pwm_max = 0, float angle_min = 0, float angle_max = 0)
  {
    this->pinNumber = _pinNumber;
    this->pwmDriver = _pwmDriver;
    this->angle = 0;
    this->pwm = 0;
    setRanges(pwm_min, pwm_max, angle_min, angle_max);
  }

  //-------------------------------------------------
  //  Set the ranges for (PWM-to-Angle) Conversions
  //-------------------------------------------------
  void setRanges(int pwm_min, int pwm_max, float angle_min, float angle_max);

  //----------------------------------------------------------------------
  // set the position of the servo from the given PWM (in microseconds)
  //----------------------------------------------------------------------
  void setPositionPWM(int _pwm);

  //----------------------------------------------------------------------
  // set the position of the servo from the given Angle (in degrees)
  //----------------------------------------------------------------------
  void setPositionAngle(float _angle);

  //----------------------------------------------------------------------
  // move from current position by the given PWM (in microseconds)
  //----------------------------------------------------------------------
  void moveByPWM(int _pwm);

  //----------------------------------------------------------------------
  // move from current position by the given Angle (in degrees)
  //----------------------------------------------------------------------
  void moveByAngle(float _angle);

  //-----------------------------------------------
  //  Setters and Getters
  //-----------------------------------------------
  void setPin(int _pinNumber);
  int getPin();
  float getAngle();
  int getPWM();
};

#endif  // SERVO_MOTOR_H
