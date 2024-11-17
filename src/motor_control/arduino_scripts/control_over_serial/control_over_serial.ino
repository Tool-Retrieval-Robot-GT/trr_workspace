#include <Arduino.h>
#include <PID_v1.h>
#include <PinChangeInterrupt.h>

// Left BACKLEFTPWM =3 , LEFT FRONTLEFTPWM = 10, RIGHT FRONTLEFTINPUT1 = 2, 

// Declare the motor driver pins 785 436
constexpr int FRONTLEFTINPUT1 = 4, FRONTLEFTINPUT2 = 3, FRONTLEFTPWM = 6;
constexpr int BACKLEFTINPUT1 = 7, BACKLEFTINPUT2 = 8, BACKLEFTPWM = 5;

constexpr int FRONTRIGHTINPUT1 = A0, FRONTRIGHTINPUT2 = A1, FRONTRIGHTPWM = 10;
constexpr int BACKRIGHTINPUT1 = 12, BACKRIGHTINPUT2 = 11, BACKRIGHTPWM = 9;

constexpr int GEARRATIO = 50;
constexpr int ENCODERTPR = 11;  // Encoders claim to go 11 ticks per rotation with 1 gear
constexpr int ENCODERTRUETPR = GEARRATIO * ENCODERTPR;  // True ticks per rotation is 11 * gears

// Get the Encoder inputs
// Front Left:
constexpr int ENCODER1, ENCODER2;
// Back Left
constexpr int ENCODER3 = A2, ENCODER4 = A6;
// Front Right:
constexpr int ENCODER5, ENCODER6;
// Back Right
constexpr int ENCODER7 = A4, ENCODER8 = A5;

// Encoder Book-keeping:
int frontLeftEncoderCount = 0, backLeftEncoderCount = 0, frontRightEncoderCount = 0, backRightEncoderCount = 0;
int lastEncoderCount[4];
unsigned long lastTimes[4];

// PID / Motor Book-keeping:
bool usePID = true, encoderReset = false;
int motorVelocities[2] {0};
int* motorVelPointer = motorVelocities;

// Parameters for the PID:
// Left two motors:
double setPointLF = 0, inputLF = 0, outputLF = 0;
double setPointLB = 0, inputLB = 0, outputLB = 0;

// Right two motors:
double setPointRF = 0, inputRF = 0, outputRF = 0;
double setPointRB = 0, inputRB = 0, outputRB = 0;

//PID control parameters: 
double kpLF = 50, kiLF = 0.5, kdLF = 0.5;
double kpLB = 50, kiLB = 0.5, kdLB = 0.5;
double kpRF = 50, kiRF = 0.5, kdRF = 0.5;
double kpRB = 50, kiRB = 0.5, kdRB = 0.5;

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
      // Pins are flipped for left side so backwards --> forwards
      if(isTopMotor)
      {
        in1 = FRONTLEFTINPUT2;
        in2 = FRONTLEFTINPUT1;
        pwm = FRONTLEFTPWM;
      }
      else
      {
        in1 = BACKLEFTINPUT2;
        in2 = BACKLEFTINPUT1;
        pwm = BACKLEFTPWM;
      }
    }
    else
    {
      // Determines if it is talking to top motor controller or bottom
      if(isTopMotor)
      {
        in1 = FRONTRIGHTINPUT1;
        in2 = FRONTRIGHTINPUT2;
        pwm = FRONTRIGHTPWM;
      }
      else
      {
        in1 = BACKRIGHTINPUT1;
        in2 = BACKRIGHTINPUT2;
        pwm = BACKRIGHTPWM;
      }
    }

    /*
     * Sets output pins
     * Named in1/2 due to it being an input from the arduino to output to the controller
     */
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

// Initialize the motors
motorDriver frontLeftMotor(true, true);
motorDriver backLeftMotor(true, false);
motorDriver frontRightMotor(false, true);
motorDriver backRightMotor(false, false);

void setup()
{
  Serial.begin(57600);

  // This code relates to PID control:
  
  // Assign all of the encoder pins as inputs 
  pinMode(BACKRIGHTINPUT1, OUTPUT);
  pinMode(BACKRIGHTINPUT2, OUTPUT);
  pinMode(BACKLEFTINPUT1, OUTPUT);
  pinMode(BACKLEFTINPUT2, OUTPUT);
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

  // Attach digital pins as interrupts to properly read the encoders
  attachPCINT(digitalPinToPCINT(ENCODER1), updatefrontLeftEncoderCount, RISING);
  attachPCINT(digitalPinToPCINT(ENCODER3), updatebackLeftEncoderCount, RISING);
  attachPCINT(digitalPinToPCINT(ENCODER5), updatefrontRightEncoderCount, RISING);
  attachPCINT(digitalPinToPCINT(ENCODER7), updatebackRightEncoderCount, RISING);

  // Assign start time for each encoder
  for(int i = 0; i < 4; i++)
  {
    lastTimes[i] = millis();
  }
}

void loop()
{
  // Keep moving the motors until a command is sent
  while (Serial.available() == 0)
  {
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
      if(changeInEncoder[0] == 0)
      {
        inputLF = 0;
      }
      else
      {
        inputLF = changeInEncoder[2] / ((millis() - lastTimes[0]) * 0.001) / ENCODERTRUETPR;
        lastTimes[0] = millis();
        lastEncoderCount[0] = backLeftEncoderCount;
      }

      if(changeInEncoder[1] == 0)
      {
        inputRF = 0;
      }
      else
      {
        inputRF = changeInEncoder[3] / ((millis() - lastTimes[1]) * 0.001) / ENCODERTRUETPR;
        lastTimes[1] = millis();
        lastEncoderCount[1] = backRightEncoderCount;
      }

      if(changeInEncoder[2] == 0)
      {
        inputLB = 0;
      }
      else
      {
        inputLB = (changeInEncoder[2] / ((millis() - lastTimes[2]) * 0.001)) / ENCODERTRUETPR;
        lastTimes[2] = millis();
        lastEncoderCount[2] = backLeftEncoderCount;
      }

      if(changeInEncoder[3] == 0)
      {
        inputRB = 0;
      }
      else
      {
        inputRB = changeInEncoder[3] / ((millis() - lastTimes[3]) * 0.001) / ENCODERTRUETPR;
        lastTimes[3] = millis();
        lastEncoderCount[3] = backRightEncoderCount;
      }

      inputLF = inputLB;
      inputRF = inputRB;

      // Compute the new PWM frequency
      leftFrontPID.Compute();
      leftBackPID.Compute();
      rightFrontPID.Compute();
      rightBackPID.Compute();

      // Set the new PWM frequency
      frontLeftMotor.move(outputLF);
      frontRightMotor.move(outputRF);
      backLeftMotor.move(outputLB);
      backRightMotor.move(outputRB);
    }

    Serial.println("Encoders: ");
    Serial.println(backRightEncoderCount);
    Serial.println("Outputs: ");
    Serial.println(outputLF);
    Serial.println(outputLB);
    Serial.println(outputRF);
    Serial.println(outputRB);
    Serial.print("\n");
  }

  // When the input gets sent:
  String input = Serial.readString();

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

      encoderReset = true;
      break;

    default:
      Serial.println("Invalid Input");
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
  // If ENCODER1 is high by the time it gets to ENCODER2 then it's moving clockwise
  if(digitalRead(ENCODER1) == HIGH && digitalRead(ENCODER2) == LOW)
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
  if(digitalRead(ENCODER3) == HIGH && digitalRead(ENCODER4) == LOW)
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
  if(digitalRead(ENCODER5) == HIGH && digitalRead(ENCODER6) == LOW)
  {
    frontRightEncoderCount--;
  }
  else
  {
    frontRightEncoderCount++;
  }
}

void updatebackRightEncoderCount()
{
  if(digitalRead(ENCODER7) == HIGH && digitalRead(ENCODER8) == LOW)
  {
    backRightEncoderCount--;
  }
  else
  {
    backRightEncoderCount++;
  }
}
