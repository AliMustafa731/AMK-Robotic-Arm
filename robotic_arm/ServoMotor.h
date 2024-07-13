/*
  Robotic Arm using Arduino & Bluetooth
  By : Ali Mustafa Kamel, 2024 Feb-July
*/


#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Adafruit_PWMServoDriver.h>

//---------------------------------------------------------------------
//  Servo Motor Structure to hold information (pin number, position)
//---------------------------------------------------------------------
class ServoMotor
{
private:

  int pwm;      // PWM position (in microseconds)
  float angle;  // Angle position (in degrees)
  
  // PWM-to-Angle conversion ranges
  float min_pwm, max_pwm;
  float min_angle, max_angle;

  // safe (min / max) limits of angles that can be reached (found by experimentation)
  float min_sweep_angle, max_sweep_angle;

  // PCA9685 Servo Driver Handle
  Adafruit_PWMServoDriver* pwmDriver;
  
  // pin number of this servo on the PCA9685 Driver (from 0 to 15)
  int pinNumber;

public:

  ServoMotor(){}
  
  //---------------------------------
  // Initialize the Servo Motor
  //---------------------------------
  void begin(int _pinNumber, Adafruit_PWMServoDriver* _pwmDriver, float _min_pwm, float _max_pwm, float _min_angle, float _max_angle);

  //-------------------------------------------------
  // set the ranges for (PWM-to-Angle) Conversions
  //-------------------------------------------------
  void setRanges(float _min_pwm, float _max_pwm, float _min_angle, float _max_angle);

  //-------------------------------------------------
  // set the safe (min / max) limits of the servo
  //-------------------------------------------------
  void setLimits(float _min_sweep_angle, float _max_sweep_angle);

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
  float getAngle();
  int getPin();
  int getPWM();
};

#endif  // SERVO_MOTOR_H
