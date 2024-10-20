#include <Arduino.h>
#include <PID_v1.h>

//Declare the pins on the ARDUINO
constexpr int DIGITAL0 = 1,DIGITAL1 = 2;    // Doubles as TX and RX respectively
constexpr int DIGITAL2 = 5, DIGITAL3 = 6, DIGITAL4 = 7, DIGITAL5 = 8;

// Declare the motor driver pins
constexpr int INA1 = 3, INA2 = 4, PWMA = 5, INB1 = 6, INB2 = 7, PWMB = 8;
constexpr int PIDINL, PIDINR, PIDOUTL, PIDOUTR;

// Get the Encoder inputs
constexpr int ENCODER1, ENCODER2, ENCODER3, ENCODER4, ENCODER5, ENCODER6, ENCODER7, ENCODER8; // Assign pins when found

volatile int interruptFlag = false;
volatile int encoderCount1 = 0, encoderCount2 = 0, encoderCount3 = 0, encoderCount4 = 0;

// Parameters for the PID:
double setPointLF, inputLF, outputLF, setPointLB, inputLB, outputLB; // Left two motors
double setPointRF, inputRF, outputRF, setPointRB, inputRB, outputRB; // Right two motors
double kpLF = 2, kiLF = 5, kdLF = 1, kpLB = 2, kiLB = 5, kdLB = 1;
double kpRF = 2, kiRF = 5, kdRF = 1, kpRB = 2, kiRB = 5, kdRB = 1;

PID leftFrontPID(&inputLF, &outputLF, &setPointLF, kpLF, kiLF, kdLF, DIRECT);
PID leftBackPID(&inputLB, &outputLB, &setPointLB, kpLB, kiLB, kdLB, DIRECT);
PID rightFrontPID(&inputRF, &outputRF, &setPointRF, kpRF, kiRF, kdRF, DIRECT);
PID rightBackPID(&inputRB, &outputRB, &setPointRB, kpRB, kiRB, kdRB, DIRECT);

void fault_detected()
{
  interruptFlag = true;
}

class motorDriver
{
  int in1, in2, pwm;
public:
  int encoderCount;
  motorDriver(bool isLeftSide, bool isTopMotor)
  {
    // Determines if it is talking to the left motor controller or the right one
    if(isLeftSide)
    {
      // Determines if it is talking to top motor controller or bottom
      if(isTopMotor)
      {
        in1 = DIGITAL3;
        in2 = DIGITAL4;
        pwm = DIGITAL5;
      }
      else
      {
        in1 = DIGITAL0;
        in2 = DIGITAL1;
        pwm = DIGITAL2;
      }
    }
    else
    {
      // When the second motor driver is hooked up add the right pin outs
    }

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(pwm, OUTPUT);

    digitalWrite(in1, 0);
    digitalWrite(in2, 0);
    digitalWrite(pwm, 0);
  }
  inline void moveForeward()
  {
    // Make sure backwards is off before turning forewards on
    digitalWrite(in2, 0);
    digitalWrite(in1, 1);
  }
  inline void moveBackward()
  {
    // Make sure forewards is off before turning backwards on
    digitalWrite(in1, 0);
    digitalWrite(in2, 1);
  }
  inline void changeSpeed(int speed)
  {
    // Divide speed by 255 so max value is 1 aka High
    digitalWrite(pwm, speed / 255);
  }
  inline void pidMoveMotor(int calculatedValue)
  {
    changeSpeed(abs(calculatedValue));
    if(calculatedValue > 0)
    {
      moveForeward();
    }
    else
    {
      moveBackward();
    }
  }
};

void setup()
{
  Serial.begin(9600);

  pinMode(ENCODER1, INPUT);
  pinMode(ENCODER2, INPUT);
  pinMode(ENCODER3, INPUT);
  pinMode(ENCODER4, INPUT);
  pinMode(ENCODER5, INPUT);
  pinMode(ENCODER6, INPUT);
  pinMode(ENCODER7, INPUT);
  pinMode(ENCODER8, INPUT);

  int intr1 = digitalPinToInterrupt(2), intr2 = digitalPinToInterrupt(3);
  if(intr1 == -1)
  {
    Serial.write("Pin 2 is not an interrupt pin\n");
  }
  else if (intr2 == -1)
  {
    Serial.write("Pin 3 is not an interrupt pin\n");
  }
  else
  {
    attachInterrupt(2, fault_detected, LOW);
  }
  setPointLF = 100;
  setPointLB = 100;
  setPointRF = 100;
  setPointRB = 100;
  leftFrontPID.SetMode(AUTOMATIC);
  leftBackPID.SetMode(AUTOMATIC);
  rightFrontPID.SetMode(AUTOMATIC);
  rightBackPID.SetMode(AUTOMATIC);
  attachInterrupt(ENCODER1, updateEncoderCount1, RISING);
  attachInterrupt(ENCODER3, updateEncoderCount2, RISING);
  attachInterrupt(ENCODER5, updateEncoderCount3, RISING);
  attachInterrupt(ENCODER7, updateEncoderCount4, RISING);
}

void loop()
{
  // Make the motor objects
  motorDriver frontLeftMotor(true, true);
  motorDriver backLeftMotor(true, false);
  motorDriver frontRightMotor(false, true);
  motorDriver backRightMotor(false, false);

  // Constantly check the interrupts
  if(interruptFlag == true)
  {
    frontLeftMotor.changeSpeed(0);
    frontRightMotor.changeSpeed(0);
    backLeftMotor.changeSpeed(0);
    backRightMotor.changeSpeed(0);
  }
  
  int isManual = 0;
  if(isManual == 0)
  {
    while(Serial.available() == 0){}
  }

  int isManual = Serial.parseInt();
  int incomingByte = Serial.read();
  printf("%i\n", isManual);

  if(isManual != 1)
  {
    while(Serial.available() > 0)
    {
      int isShutDown = Serial.parseInt();
      if (isShutDown == 1)
      {
        isManual = false;
        break;
      }
    }
    int speedToChangeTo = 0;
    while(Serial.available() == 0) {}
    speedToChangeTo = Serial.parseInt();
    incomingByte = Serial.read();
  
    // This code only allows the motors to go forwards and backwards at the moment
    setPointLF = speedToChangeTo;
    setPointLB = speedToChangeTo;
    setPointRF = speedToChangeTo;
    setPointRB = speedToChangeTo;

    inputLF = encoderCount1;
    inputLB = encoderCount2;
    inputRF = encoderCount3;
    inputRB = encoderCount4;

    // Have PID calculate the speed
    leftFrontPID.Compute();
    frontLeftMotor.pidMoveMotor(outputLF);
    leftBackPID.Compute();
    backLeftMotor.pidMoveMotor(outputLB);
    rightFrontPID.Compute();
    frontRightMotor.pidMoveMotor(outputRF);
    rightBackPID.Compute();
    backRightMotor.pidMoveMotor(outputRB);
  }
  else
  {
    int motorToChange = Serial.parseInt();
    incomingByte = Serial.read();
    printf("%i\n", motorToChange);
    
    while(Serial.available() == 0) {}
    
    int functionToCall = Serial.parseInt();
    incomingByte = Serial.read();
    printf("%i\n", functionToCall);
    
    int speedToChangeTo = 0;
    while(Serial.available() == 0) {}
    speedToChangeTo = Serial.parseInt();
    incomingByte = Serial.read();

    switch(motorToChange)
    {
      case(0):
        switch(functionToCall)
        {
          case(0):
            frontLeftMotor.moveForeward();
            break;
          case(1):
            frontLeftMotor.moveBackward();
            break;
          case(2):
            frontLeftMotor.changeSpeed(speedToChangeTo);
            break;
          default:
            break;
        }
        break;
      case(1):
        switch(functionToCall)
        {
          case(0):
            frontRightMotor.moveForeward();
            break;
          case(1):
            frontRightMotor.moveBackward();
            break;
          case(2):
            frontRightMotor.changeSpeed(speedToChangeTo);
            break;
          default:
            break;
        }
        break;
      case(2):
        switch(functionToCall)
        {
          case(0):
            backLeftMotor.moveForeward();
            break;
          case(1):
            backLeftMotor.moveBackward();
            break;
          case(2):
            backLeftMotor.changeSpeed(speedToChangeTo);
            break;
          default:
            break;
        }
        break;
      case(3):
        switch(functionToCall)
        {
          case(0):
            backRightMotor.moveForeward();
            break;
          case(1):
            backRightMotor.moveBackward();
            break;
          case(2):
            backRightMotor.changeSpeed(speedToChangeTo);
            break;
          default:
            break;
        }
        break;
      default:
        break;
    }
  }
}

void updateEncoderCount1()
{
  if(digitalRead(ENCODER1) == HIGH)
  {
    encoderCount1++;
  }
  else
  {
    encoderCount1--;
  }
}

void updateEncoderCount2()
{
  if(digitalRead(ENCODER3) == HIGH)
  {
    encoderCount2++;
  }
  else
  {
    encoderCount2--;
  }
}

void updateEncoderCount3()
{
  if(digitalRead(ENCODER5) == HIGH)
  {
    encoderCount3++;
  }
  else
  {
    encoderCount3--;
  }
}

void updateEncoderCount4()
{
  if(digitalRead(ENCODER7) == HIGH)
  {
    encoderCount4++;
  }
  else
  {
    encoderCount4--;
  }
}