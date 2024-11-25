#include <Arduino.h>
#include <PID_v1.h>
#include <PinChangeInterrupt.h>

// Declare the motor driver pins
constexpr int FRONTLEFTINPUT1 = 7, FRONTLEFTINPUT2 = 8, FRONTLEFTPWM = 5;
constexpr int BACKLEFTINPUT1 = 7, BACKLEFTINPUT2 = 8, BACKLEFTPWM = 5;

constexpr int FRONTRIGHTINPUT1 = 12, FRONTRIGHTINPUT2 = 11, FRONTRIGHTPWM = 9;
constexpr int BACKRIGHTINPUT1 = 12, BACKRIGHTINPUT2 = 11, BACKRIGHTPWM = 9;

// Encoder Uses:
constexpr int PIDRATE = 30; // In Hz
const int PIDINTERVAL = 1000 / PIDRATE;
unsigned long nextPIDInterval = PIDINTERVAL;

// Get the Encoder inputs
// Back Left
constexpr int ENCODERABL = 4, ENCODERBBL = 3;
// Back Right
constexpr int ENCODERABR = 6, ENCODERBBR = 2;

// Encoder Book-keeping:
int backLeftEncoderCount = 0, backRightEncoderCount = 0;
int lastEncoderCount[2];

// PID / Motor Book-keeping:
bool usePID = true, encoderReset = false;
int motorVelocities[2] {0};
int* motorVelPointer = motorVelocities;

// Parameters for the PID:
double setPointLB = 0, inputLB = 0, outputLB = 0;
double setPointRB = 0, inputRB = 0, outputRB = 0;

//PID control parameters: 
double kpLB = 5, kiLB = 1, kdLB = 0;
double kpRB = 5, kiRB = 1, kdRB = 0;

PID leftBackPID(&inputLB, &outputLB, &setPointLB, kpLB, kiLB, kdLB, DIRECT);
PID rightBackPID(&inputRB, &outputRB, &setPointRB, kpRB, kiRB, kdRB, DIRECT);

void getMotorValues(int* motorPointer, String motorValString);
void setSetPoints(String setPointString);
void sendEncoderValues();
void updatefrontLeftEncoderCount();
void updatebackLeftEncoderCount();
void updatefrontRightEncoderCount();
void updatebackRightEncoderCount();

char mode, lastMode;

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
      if(velocity > 255)
      {
        velocity = 255;
      }
      digitalWrite(in1, 1);
      digitalWrite(in2, 0);
      analogWrite(pwm, velocity);
    }
    else
    {
      if(abs(velocity) > 255)
      {
        velocity = 255;
      }
      digitalWrite(in1, 0);
      digitalWrite(in2, 1);
      analogWrite(pwm, abs(velocity));
    }
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
  pinMode(ENCODERABL, INPUT);
  pinMode(ENCODERBBL, INPUT);
  pinMode(ENCODERABR, INPUT);
  pinMode(ENCODERBBR, INPUT);
  pinMode(A5, OUTPUT);

  leftBackPID.SetMode(AUTOMATIC);
  rightBackPID.SetMode(AUTOMATIC);
  //leftBackPID.SetOutputLimits(-255, 255);
  //rightBackPID.SetOutputLimits(-255, 255);

  // Attach digital pins as interrupts to properly read the encoders
  //attachPCINT(digitalPinToPCINT(ENCODERABL), updatebackLeftEncoderCount, RISING);
  //attachPCINT(digitalPinToPCINT(ENCODERABR), updatebackRightEncoderCount, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODERBBL), updatebackLeftEncoderCount, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODERBBR), updatebackRightEncoderCount, RISING);
}

void loop()
{
  // Keep moving the motors until a command is sent
  while (Serial.available() > 0)
  { 
    // When the input gets sent:
    String input = Serial.readString();
  
    mode = input[0];
    String tempString = "";
    tempString += input[0];
    tempString += input[1];
    if(tempString.toInt() == 13)
    {
      break;
    }
    Serial.print(tempString);
    Serial.println(tempString.toInt());
    
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
        backLeftEncoderCount = 0;
        backRightEncoderCount = 0;
  
        encoderReset = true;
        break;
  
      default:
        Serial.println("Invalid Command");
    }
  }

  if(usePID == false)
  {
    // Turn PID off
    setPointLB = 0;
    setPointRB = 0;
    
    // Manually set the motor speeds
    frontLeftMotor.move(motorVelocities[0]);
    backLeftMotor.move(motorVelocities[0]);
    frontRightMotor.move(motorVelocities[1]);
    backRightMotor.move(motorVelocities[1]);
  }
  else if(millis() > nextPIDInterval);
  {
    int changeInEncoder[2];

    // If ROS2 calls for a reset then there's no need to compute the difference 
    if(encoderReset == false)
    {
      changeInEncoder[0] = (backLeftEncoderCount - lastEncoderCount[0]);
      changeInEncoder[1] = (backRightEncoderCount - lastEncoderCount[1]);
    }
    else
    {
      changeInEncoder[0] = backLeftEncoderCount;
      changeInEncoder[1] = backRightEncoderCount;
      encoderReset = false;
    }
    
    // Setpoint provided by ros_arduino_bridge is in ticks/second so the input to the PID must be the same
    if(changeInEncoder[0] != 0)
    {
      inputLB = changeInEncoder[0];
      lastEncoderCount[0] = backLeftEncoderCount;
    }
    else
    {
      inputLB = 0;
    }

    if(changeInEncoder[1] != 0)
    {
      inputRB = changeInEncoder[1];
      lastEncoderCount[1] = backRightEncoderCount;
    }
    else
    {

      inputRB = 0;
    }

    // Compute the new PWM frequency
    leftBackPID.Compute();
    rightBackPID.Compute();

//    Serial.println("Change: ");
//    Serial.println(changeInEncoder[0]);
//    Serial.println(changeInEncoder[1]);
//
//    Serial.println("Encoders: ");
//    Serial.println(backLeftEncoderCount);
//    Serial.println(backRightEncoderCount);
//
//    Serial.println("Set points: ");
//    Serial.println(setPointLB);
//    Serial.println(setPointRB);
//
//    Serial.println("Inputs: ");
//    Serial.println(inputLB);
//    Serial.println(inputRB);
//
//    Serial.println("Outputs: ");
//    Serial.println(outputLB);
//    Serial.println(outputRB);

    // Set the new PWM frequency
    frontLeftMotor.move(outputLB);
    frontRightMotor.move(outputRB);
    backLeftMotor.move(outputLB);
    backRightMotor.move(outputRB);

    nextPIDInterval += PIDINTERVAL;
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
  setPointRB = (double)(*(setPointBufferPointer + 1));
  if(setPointLB == 0 && setPointRB == 0)
  { 
    frontLeftMotor.move(0);
    frontRightMotor.move(0);
    backLeftMotor.move(0);
    backRightMotor.move(0);
    //usePID = false;
  }

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

void updatebackLeftEncoderCount()
{
  if(digitalRead(ENCODERABL) == LOW)
  {
    backLeftEncoderCount--;
  }
  else
  {
    backLeftEncoderCount++;
  }
}

void updatebackRightEncoderCount()
{
  if(digitalRead(ENCODERABR) == LOW)
  {
    backRightEncoderCount++;
  }
  else
  {
    backRightEncoderCount--;
  }
}
