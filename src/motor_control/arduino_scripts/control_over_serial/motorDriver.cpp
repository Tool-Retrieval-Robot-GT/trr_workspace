#include "motorDriver.h"

void motorDriver::move(int velocityToMove = 0)
{
    if(velocityToMove != 0)
    {
        // If a new velocity is input then 
        motorVelocity = velocityToMove;
    }

    if(motorVelocity > 0)
    {
      digitalWrite(in1, 1);
      digitalWrite(in2, 0);
      analogWrite(pwm, velocity);
    }
    else
    {
      digitalWrite(in1, 0);
      digitalWrite(in2, 1);
      analogWrite(pwm, abs(velocity));
    }
}

void motorDriver::stopMotor()
{
    digitalWrite(in1, 0);
    digitalWrite(in2, 0);
    analogWrite(pwm, 0);
}

void motorDriver::initEncoders(int encoderPin1, int encoderPin2)
{
    pinMode(encoderPin1, INPUT);
    pinMode(encoderPin2, INPUT);
    encoder1 = encoderPin1;
    encoder2 = encoderPin2;
    lastTime = millis();
    attachInterrupt(encoderPin1, motorDriver::incrementEncoder, rising);
}

void motorDriver::incrementEncoder()
{
    if(digitalRead(encoder1) == HIGH)
    {
        encoderTicks++;
    }
    else
    {
        encoderTicks--;
    }
}

void motorDriver::resetEncoder()
{
    encoderTicks = 0;
}

void motorDriver::getEncoderTicksPerSecond(bool& wasEncoderReset)
{
    if(wasEncoderReset == true)
    {
        encoderTicksPerSecond = encoderTicks / ((millis() - lastTime) * 0.001);
        lastTime = millis();
        wasEncoderReset = false;
    }
    else
    {
        encoderTicksPerSecond = (encoderTicks - lastEncoderTicks) / ((millis() - lastTime) * 0.001);
        lastEncoderTicks = encoderTicks;
        lastTime = millis();
    }
    return encoderTicksPerSecond;
}