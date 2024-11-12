#include "motorDriver.h"
#include <PID_v1.h>

// Declare the motor driver pins
constexpr int INA1 = A0, INA2 = A1, PWMA = 5, INB1 = 4, INB2 = 7, PWMB = 6;
constexpr int INA3 = 8, INA4 = 12, PWMC = 9, INB3 = 10, INB4 = 13, PWMD = 11;

// Get the Encoder inputs
// Front Left:
constexpr int ENCODER1, ENCODER2;
// Back Left
constexpr int ENCODER3 = 2, ENCODER4 = 3;
// Front Right:
constexpr int ENCODER5, ENCODER6;
// Back Right
constexpr int ENCODER7 = 17, ENCODER8 = 18;

// Volatiles
volatile bool interruptFlag = false;

bool usePID = true, encoderReset = false;

// Parameters for the PID:
// Left two motors:
double setPointLF, inputLF, outputLF;
double setPointLB, inputLB, outputLB;

//PID control parameters: 
double kpLF = 2, kiLF = 1, kdLF = 1;
double kpLB = 2, kiLB = 1, kdLB = 1;

PID leftFrontPID(&inputLF, &outputLF, &setPointLF, kpLF, kiLF, kdLF, DIRECT);
PID leftBackPID(&inputLB, &outputLB, &setPointLB, kpLB, kiLB, kdLB, DIRECT);

motorDriver frontLeftMotor(true, true);
motorDriver backLeftMotor(true, false);

void getMotorValues(int* motorPointer, String motorValString);
void setSetPoints(String setPointString);
void sendEncoderValues();

void fault_detected()
{
  //interruptFlag = true; // Uncomment when interrupts are tested on motor controls
}

void setup()
{
  Serial.begin(57600);

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

  leftFrontPID.SetMode(AUTOMATIC);
  leftBackPID.SetMode(AUTOMATIC);

  leftFrontMotor.initEncoders();
  leftBackMotor.initEncoders();
}

void loop()
{
  int motorVelocities[2] {0};
  int* motorVelPointer = motorVelocities;
  
  // Constantly check the interrupts
  if(interruptFlag == true)
  {
    frontLeftMotor.stopMotor();
    backLeftMotor.stopMotor();
  }
  else
  {
    // if input is received handle it
    if(Serial.available() > 0)
    {
      String input = Serial.readString();
    
      // The controller messages will be of form e or m so determine which one to use
      char mode = input[0];
      switch(mode)
      {
        case('m'):
          usePID = true; // Let PID do the work
          input.remove(0, 2);
          setSetPoints(input);
          break;
          
        case('o'):
          usePID = false; // Want to set the PWM manually
          input.remove(0, 2); // Remove the mode and space that follows
  
          //Get the motor values
          getMotorValues(motorVelPointer, input);
          
          break;
          
        case('e'):
          sendEncoderValues();
          break;
        
        case('r'):
          frontLeftMotor.resetEncoder()
          backLeftMotor.resetEncoder();
          break;
  
        default:
          Serial.println("Invalid Input");
      }
    }

    if(usePID == false)
    {
      // Turn PID off
      setPointLF = 0;
      setPointLB = 0;
      setPointRF = 0;
      setPointRB = 0;
      
      // Manually set the motor speeds
      frontLeftMotor.move(motorVelocities[0]);
      backLeftMotor.move(motorVelocities[0]);
    }
    else
    {
      inputLF = frontLeftMotor.getEncoderTicksPerSecond();
      inputLB = frontBackMotor.getEncoderTicksPerSecond();

      // Compute the new PWM frequency
      leftFrontPID.Compute();
      rightFrontPID.Compute();

      Serial.println(outputRF);

      // Set the new PWM frequency
      frontLeftMotor.move(outputLF);
      frontRightMotor.move(outputRF);
      backLeftMotor.move(outputLB);
      backRightMotor.move(outputRB);
    }
  }
}

void getMotorValues(int* motorPointer, String motorValString)
{
  int stringIndex = 0;
  // Iterate through the two motor velocities
  for(int motorCount = 0; motorCount < 2; motorCount++)
  {
    String motorValAsString = "";
    
    // Loop until it sees the next space
    while(!isSpace(motorValString[stringIndex]))
    {
      motorValAsString += motorValString[stringIndex];
      stringIndex++;
    }
    // Assign the value (as an int) to the buffer location
    *(motorPointer + motorCount) = motorValAsString.toInt();
    stringIndex++;
  }

  // Response message to controller manager
  Serial.println("OK");
}

void setSetPoints(String setPointString)
{
  int stringIndex = 0;
  int setPointBuffer[2] {0};
  int* setPointBufferPointer = setPointBuffer;
  // Iterate through the two motor velocities
  for(int motorCount = 0; motorCount < 2; motorCount++)
  {
    String setPointAsString = "";
    
    // Loop until it sees the next space
    while(!isSpace(setPointString[stringIndex]))
    {
      setPointAsString += setPointString[stringIndex];
      stringIndex++;
    }
    // Assign the value (as an int) to the buffer location
    *(setPointBufferPointer + motorCount) = setPointAsString.toInt();
    stringIndex++;
  }

  // Set the set points for the encoders
  setPointLB = (double)(*setPointBufferPointer);
  setPointLF = (double)(*setPointBufferPointer);
  setPointRB = (double)(*(setPointBufferPointer + 1));
  setPointRF = (double)(*(setPointBufferPointer + 1));

  // Response message to controller manager
  Serial.println("OK");
}

void sendEncoderValues()
{
  Serial.print(backLeftEncoderCount); // Send back left encoder count
  Serial.print(" ");
  Serial.println(backRightEncoderCount); // Send back right encoder count
}
