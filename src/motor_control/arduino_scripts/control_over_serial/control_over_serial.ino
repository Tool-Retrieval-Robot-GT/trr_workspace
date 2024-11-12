#include <Arduino.h>
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
volatile int frontLeftEncoderCount = 0, backLeftEncoderCount = 0, frontRightEncoderCount = 0, backRightEncoderCount = 0;

int lastEncoderCount[4];
unsigned long lastTimes[4];

bool usePID = true, encoderReset = false;

// Parameters for the PID:
// Left two motors:
double setPointLF, inputLF, outputLF;
double setPointLB, inputLB, outputLB;

// Right two motors:
double setPointRF, inputRF, outputRF;
double setPointRB, inputRB, outputRB;

//PID control parameters: 
double kpLF = 2, kiLF = 1, kdLF = 1;
double kpLB = 2, kiLB = 1, kdLB = 1;
double kpRF = 2, kiRF = 1, kdRF = 1;
double kpRB = 2, kiRB = 1, kdRB = 1;

PID leftFrontPID(&inputLF, &outputLF, &setPointLF, kpLF, kiLF, kdLF, DIRECT);
PID leftBackPID(&inputLB, &outputLB, &setPointLB, kpLB, kiLB, kdLB, DIRECT);
PID rightFrontPID(&inputRF, &outputRF, &setPointRF, kpRF, kiRF, kdRF, DIRECT);
PID rightBackPID(&inputRB, &outputRB, &setPointRB, kpRB, kiRB, kdRB, DIRECT);

void getMotorValues(int* motorPointer, String motorValString);
void setSetPoints(String setPointString);
void sendEncoderValues();
void updatefrontLeftEncoderCount();
void updatebackLeftEncoderCount();
void updatefrontRightEncoderCount();
void updatebackRightEncoderCount();

void fault_detected()
{
  //interruptFlag = true; // Uncomment when interrupts are tested on motor controls
}

class motorDriver
{
  int in1, in2, pwm = 127;
  
public:
  bool needReset = false;
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
    }
    else
    {
      digitalWrite(in1, 0);
      digitalWrite(in2, 1);
      analogWrite(pwm, abs(velocity));
    }

    delay(10);
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

  // This code relates to PID control: 

  pinMode(ENCODER1, INPUT);
  pinMode(ENCODER2, INPUT);
  pinMode(ENCODER3, INPUT);
  pinMode(ENCODER4, INPUT);
  pinMode(ENCODER5, INPUT);
  pinMode(ENCODER6, INPUT);
  pinMode(ENCODER7, INPUT);
  pinMode(ENCODER8, INPUT);

  leftFrontPID.SetMode(AUTOMATIC);
  leftBackPID.SetMode(AUTOMATIC);
  rightFrontPID.SetMode(AUTOMATIC);
  rightBackPID.SetMode(AUTOMATIC);
  
  attachInterrupt(ENCODER1, updatefrontLeftEncoderCount, RISING);
  attachInterrupt(ENCODER3, updatebackLeftEncoderCount, RISING);
  attachInterrupt(ENCODER5, updatefrontRightEncoderCount, RISING);
  attachInterrupt(ENCODER7, updatebackRightEncoderCount, RISING);

  // Assign start time for each encoder
  for(int i = 0; i < 4; i++)
  {
    lastTimes[i] = millis();
  }
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
          frontLeftEncoderCount = 0;
          backLeftEncoderCount = 0;
          frontRightEncoderCount = 0;
          backRightEncoderCount = 0;
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
      frontRightMotor.move(motorVelocities[1]);
      backRightMotor.move(motorVelocities[1]);
    }
    else
    {
      int changeInEncoder[4];

      // If ROS2 calls for a reset then there's no need to compute the difference 
      if(encoderReset == false)
      {
        changeInEncoder[0] = (frontLeftEncoderCount - lastEncoderCount[0]);
        changeInEncoder[1] = (frontRightEncoderCount - lastEncoderCount[1]);
        changeInEncoder[2] = (backLeftEncoderCount - lastEncoderCount[2]);
        changeInEncoder[3] = (backRightEncoderCount - lastEncoderCount[3]);
      }
      else
      {
        changeInEncoder[0] = frontLeftEncoderCount;
        changeInEncoder[1] = frontRightEncoderCount;
        changeInEncoder[2] = backLeftEncoderCount;
        changeInEncoder[3] = backRightEncoderCount;

        encoderReset = false;
      }
      
      // Setpoint provided by ros_arduino_bridge is in ticks/second so the input to the PID must be the same
      inputLF = changeInEncoder[0] / ((millis() - lastTimes[0]) * 0.001);
      lastTimes[0] = millis();
      lastEncoderCount[0] = frontLeftEncoderCount;

      inputRF = changeInEncoder[1] / ((millis() - lastTimes[1]) * 0.001);
      lastTimes[1] = millis();
      lastEncoderCount[1] = frontRightEncoderCount;

      inputLB = changeInEncoder[2] / ((millis() - lastTimes[2]) * 0.001);
      lastTimes[2] = millis();
      lastEncoderCount[2] = backLeftEncoderCount;

      inputRB = changeInEncoder[4] / ((millis() - lastTimes[3]) * 0.001);
      lastTimes[3] = millis();
      lastEncoderCount[3] = frontLeftEncoderCount;

      // Compute the new PWM frequency
      leftFrontPID.Compute();
      leftBackPID.Compute();
      rightFrontPID.Compute();
      rightBackPID.Compute();

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

/* 
 *These functions are attached to the interrupts to determine 
 *whether the motor is moving clockwise or counter clockwise 
 */
void updatefrontLeftEncoderCount()
{
  // If ENCODER1 is high by the time it gets to ENCODER0 then it's moving clockwise
  if(digitalRead(ENCODER1) == HIGH)
  {
    frontLeftEncoderCount++;
  }
  else
  {
    frontLeftEncoderCount--;
  }
}

void updatebackLeftEncoderCount()
{
  if(digitalRead(ENCODER3) == HIGH)
  {
    backLeftEncoderCount++;
  }
  else
  {
    backLeftEncoderCount--;
  }
}

void updatefrontRightEncoderCount()
{
  if(digitalRead(ENCODER5) == HIGH)
  {
    frontRightEncoderCount++;
  }
  else
  {
    frontRightEncoderCount--;
  }
}

void updatebackRightEncoderCount()
{
  if(digitalRead(ENCODER7) == HIGH)
  {
    backRightEncoderCount++;
  }
  else
  {
    backRightEncoderCount--;
  }
}
