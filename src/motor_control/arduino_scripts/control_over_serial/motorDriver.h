#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>

class motorDriver
{
    int in1, in2, pwm = 127;
    int encoder1, encoderTicks = 0, lastEncoderTicks = 0;
    double lastTime;
    int motorVelocity = 0;
public:
    int encoderTicksPerSecond = 0;

    motorDriver(bool isLeftSide, bool isTopMotor)
    {
        // Determines if it is talking to the left motor controller or the right one
        if(isLeftSide)
        {
        // Determines if it is talking to top motor controller or bottom
        if(isTopMotor)
        {
            in1 = INB1;
            in2 = INB2;
            pwm = PWMB;
        }
        else
        {
            in1 = INA1;
            in2 = INA2;
            pwm = PWMA;
        }
        }
        else
        {
        // Determines if it is talking to top motor controller or bottom
        if(isTopMotor)
        {
            in1 = INA3;
            in2 = INA4;
            pwm = PWMD;
        }
        else
        {
            in1 = INB3;
            in2 = INB4;
            pwm = PWMC;
        }
        }

        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        pinMode(pwm, OUTPUT);

        digitalWrite(in1, 0);
        digitalWrite(in2, 0);
        analogWrite(pwm, 0);
    }

  void move(int velocityToMove);
  void stopMotor();
  void initEncoders(int encoderPin1, int encoderPin2);
  void resetEncoder();
  void incrementEncoder();
  int getEncoderTicksPerSecond(bool wasEncoderReset);
}

#endif