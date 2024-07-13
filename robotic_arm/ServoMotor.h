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
  float gain;   // PWM per Degree
  float zero;   // Theoretical PWM for zero Angle

  // safe (min / max) limits of angles that can be reached (found by experimentation)
  float min_angle, max_angle;

  // PCA9685 Servo Driver Handle
  Adafruit_PWMServoDriver* pwmDriver;
  
  // pin number of this servo on the PCA9685 Driver (from 0 to 15)
  int pinNumber;

public:

  ServoMotor(){}
  
  //---------------------------------
  // Initialize the Servo Motor
  //---------------------------------
  void begin(int _pinNumber, Adafruit_PWMServoDriver* _pwmDriver, int pwm_min = 0, int pwm_max = 0, float angle_min = 0, float angle_max = 0);

  //-------------------------------------------------
  // set the ranges for (PWM-to-Angle) Conversions
  //-------------------------------------------------
  void setRanges(int pwm_min, int pwm_max, float angle_min, float angle_max);

  //-------------------------------------------------
  // set the safe (min / max) limits of the servo
  //-------------------------------------------------
  void setLimits(float _min_angle, float _max_angle);

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
