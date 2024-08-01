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

  // PCA9685 Servo Driver Handle
  Adafruit_PWMServoDriver* pwmDriver;
  
  // pin number of this servo on the PCA9685 Driver (from 0 to 15)
  int pinNumber;

  int pwm;      // PWM position (pulse length in (1 out of 4096) of the pwmDriver frequenncy period)
  float angle;  // Angle position (in degrees)
  
  // PWM-to-Angle conversion ranges
  float min_pwm, max_pwm;
  float min_angle, max_angle;

public:

  ServoMotor(){}
  
  //---------------------------------
  // Initialize the Servo Motor
  //---------------------------------
  void begin(int _pinNumber, Adafruit_PWMServoDriver* _pwmDriver, float _min_pwm = 100, float _max_pwm = 500, float _min_angle = 0, float _max_angle = 180);

  //-------------------------------------------------
  // set the ranges for (PWM-to-Angle) Conversions
  //-------------------------------------------------
  void setRanges(float _min_pwm, float _max_pwm, float _min_angle, float _max_angle);

  //--------------------------------------------------------------------------------------------
  // set the position of the servo from the given PWM (1 out of 4096 of frequenncy period)
  //--------------------------------------------------------------------------------------------
  void setPositionPWM(int _pwm);

  //----------------------------------------------------------------------
  // set the position of the servo from the given Angle (in degrees)
  //----------------------------------------------------------------------
  void setPositionAngle(float _angle);

  //-----------------------------------------------
  //  Setters and Getters
  //-----------------------------------------------
  void setPin(int _pinNumber);
  float getAngle();
  int getPin();
  int getPWM();
};

#endif  // SERVO_MOTOR_H
