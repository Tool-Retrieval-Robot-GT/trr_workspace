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

void getMotorValues(int* motorPointer, String motorValString);
void sendEncoderValues();
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
  int in1, in2, pwm = 127;
public:
  bool needReset = false;
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
   * Moves that motor based on the velocity determined from twist messages
   */
  inline void move(int velocity)
  {
    if(velocity > 0)
    {
      digitalWrite(in1, 1);
      digitalWrite(in2, 0);
      analogWrite(pwm, velocity);
      delay(10); //Drive for 10 ms as that's the time between nav2 messages
    }
    else
    {
      digitalWrite(in1, 0);
      digitalWrite(in2, 1);
      analogWrite(pwm, abs(velocity));
      delay(10); //Drive for 10 ms as that's the time between nav2 messages
    }
  }

  inline void stopMotor()
  {
    digitalWrite(in1, 0);
    digitalWrite(in2, 0);
    analogWrite(pwm, 0);
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
  // Initialize the motors
  motorDriver frontLeftMotor(true, true);
  motorDriver backLeftMotor(true, false);
  motorDriver frontRightMotor(false, true);
  motorDriver backRightMotor(false, false);

  int motorVelocities[2] {0};
  int* motorVelPointer = motorVelocities;
  
  // Constantly check the interrupts
  if(interruptFlag == true)
  {
    frontLeftMotor.stopMotor();
    frontRightMotor.stopMotor();
    backLeftMotor.stopMotor();
    backRightMotor.stopMotor();
  }
  else
  {
    // Wait for an input
    while(Serial.available() == 0){}
    String input = Serial.readString();

    // The controller messages will be of form e or m so determine which one to use
    char mode = input[0];
    if(mode == 'm')
    {
      input.remove(0, 2); // Remove the mode and space that follows

      //Get the motor values
      getMotorValues(motorVelPointer, input);
      
      // Set the motor speeds
      frontLeftMotor.move(motorVelocities[0]);
      backLeftMotor.move(motorVelocities[0]);
      frontRightMotor.move(motorVelocities[1]);
      backRightMotor.move(motorVelocities[1]);
    }
    else if(mode == 'e')
    {
      sendEncoderValues();
    }
    else
    {
      Serial.println("INVALID COMMAND");
    }
  }
}

void getMotorValues(int* motorPointer, String motorValString)
{
  // Iterate through the two motor velocities
  for(int motorCount = 0; motorCount < 2; motorCount++)
  {
    int stringIndex = 0;
    String motorValAsString = "";
    
    // Loop until it sees the next space
    while(!isSpace(motorValString[stringIndex]))
    {
      motorValAsString += motorValString[stringIndex];
      stringIndex++;
    }
    // Assign the value (as an int) to the buffer location
    *(motorPointer + motorCount) = motorValAsString.toInt();
  }

  // Response message to controller manager
  Serial.println("OK");
}

void sendEncoderValues()
{
  Serial.print(encoderCount1);
  Serial.print(" ");
  Serial.println(encoderCount2);
}

/* 
 *These functions are attached to the interrupts to determine 
 *whether the motor is moving clockwise or counter clockwise 
 */
void updateEncoderCount1()
{
  // If ENCODER1 is high by the time it gets to ENCODER0 then it's moving clockwise
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
