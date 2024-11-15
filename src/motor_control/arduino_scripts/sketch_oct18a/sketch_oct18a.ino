#include <Arduino.h>
#include <PID_v1.h>

// Declare the motor driver pins
constexpr int INA1 = A0, INA2 = A1, PWMA = 5, INB1 = 4, INB2 = 7, PWMB = 6;
constexpr int INA3 = 8, INA4 = 12, PWMC = 9, INB3 = 10, INB4 = 13, PWMD = 11;
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

void updateEncoderCount1();
void updateEncoderCount2();
void updateEncoderCount3();
void updateEncoderCount4();

void fault_detected()
{
  //interruptFlag = true; // Uncomment when interrupts are tested on motor controls
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

 /*
  * Drives the motor foreward for one second
  */
  inline void moveForeward()
  {
    // Make sure backwards is off before turning forewards on
    digitalWrite(in2, 0);
    digitalWrite(in1, 1);
    delay(1000); // Move for one second
  }

 /*
  * Drives the motor backwards for one second
  */
  inline void moveBackward()
  {
    // Make sure forewards is off before turning backwards on
    digitalWrite(in1, 0);
    digitalWrite(in2, 1);
    delay(1000); // Move for one second
  }

 /*
  * Changes the speed of the PWM signal
  */
  inline void changeSpeed(int speed)
  {
    analogWrite(pwm, speed);
  }

 /*
  * Adjusts the motors to account for a speed and a direction
  * Note: This code is still in testing / writing
  */
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
    attachInterrupt(3, fault_detected, LOW);
  }

  // This code relates to PID control: 

  pinMode(ENCODER1, INPUT);
  pinMode(ENCODER2, INPUT);
  pinMode(ENCODER3, INPUT);
  pinMode(ENCODER4, INPUT);
  pinMode(ENCODER5, INPUT);
  pinMode(ENCODER6, INPUT);
  pinMode(ENCODER7, INPUT);
  pinMode(ENCODER8, INPUT);

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
  // Make the motor objects and set them to half power
  motorDriver frontLeftMotor(true, true);
  frontLeftMotor.changeSpeed(127);
  motorDriver backLeftMotor(true, false);
  backLeftMotor.changeSpeed(127);
  motorDriver frontRightMotor(false, true);
  frontRightMotor.changeSpeed(127);
  motorDriver backRightMotor(false, false);
  backRightMotor.changeSpeed(127);

  // Constantly check the interrupts
  if(interruptFlag == true)
  {
    frontLeftMotor.changeSpeed(0);
    frontRightMotor.changeSpeed(0);
    backLeftMotor.changeSpeed(0);
    backRightMotor.changeSpeed(0);
  }
  
  char isManual = 0;
  if(isManual == '0')
  {
    while(Serial.available() == 0){}
  }

  isManual = Serial.read();

  if(isManual == '0')
  {
     // Allow user to break the PID loop if needed
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
    int incomingByte = Serial.read();
  
    // This code relates to PID control:

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
    // Get the motor the user wants to move
    while(Serial.available() == 0) {}
    
    char motorToChange = Serial.read();

    // Get the function the user wants to call
    while(Serial.available() == 0) {}
    char functionToCall = Serial.read();

    // If the user wants to change speed
    char speedToChangeTo = 0;
    if(functionToCall == '3')
    {
      while(Serial.available() == 0) {}
      speedToChangeTo = Serial.read();
    }
    
    /*
     * The first switch statement handles the motor the user called
     * The nested switch statement handles the function the user has called
     */
    switch(motorToChange)
    {
      case '1':
        switch(functionToCall)
        {
          case '1':
            frontLeftMotor.moveForeward();
            break;
          case '2':
            frontLeftMotor.moveBackward();
            break;
          case '3':
            frontLeftMotor.changeSpeed(speedToChangeTo);
            break;
          default:
            break;
        }
        break;
      case '2':
        switch(functionToCall)
        {
          case '1':
            frontRightMotor.moveForeward();
            break;
          case '2':
            frontRightMotor.moveBackward();
            break;
          case '3':
            frontRightMotor.changeSpeed(speedToChangeTo);
            break;
          default:
            break;
        }
        break;
      case '3':
        switch(functionToCall)
        {
          case '1':
            backLeftMotor.moveForeward();
            break;
          case '2':
            backLeftMotor.moveBackward();
            break;
          case '3':
            backLeftMotor.changeSpeed(speedToChangeTo);
            break;
          default:
            break;
        }
        break;
      case '4':
        switch(functionToCall)
        {
          case '1':
            backRightMotor.moveForeward();
            break;
          case '2':
            backRightMotor.moveBackward();
            break;
          case '3':
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

/* 
 *These functions are attached to the interruprs to determine 
 *whether the motor is moving clockwise or counter clockwise 
 */
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
