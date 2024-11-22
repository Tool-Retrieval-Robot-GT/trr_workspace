#include <Arduino.h>
#include <PID_v1.h>

// Pin related info:
// Motors:
constexpr int LEFTINPUT1 = 7, LEFTINPUT2 = 8, LEFTPWM = 5;
constexpr int RIGHTINPUT1 = 12, RIGHTINPUT2 = 11, RIGHTPWM = 9;
// Encoders:
constexpr int LEFTENCODERA = 4, LEFTENCODERB = 3;
constexpr int RIGHTENCODERA = 6, RIGHTENCODERB = 2;

// Conversions:
constexpr int GEARRATIO = 50;
constexpr float WHEELRADIUS = 0.04; // In meters
constexpr float TICKSPERREV = 11;
constexpr double GAINFACTOR = 87.39339563;
float lastTime[2] {0};

// PID Control:
constexpr int TIMEOUT = 2000;
long lastCommandRecv = TIMEOUT;
constexpr int PIDRATE = 30; // In Hz
const int PIDINTERVAL = 1000 / PIDRATE;
//unsigned long nextPIDInterval = PIDINTERVAL;
bool timeout = false;

// Encoder Book-keeping:
int leftEncoderTicks = 0, rightEncoderTicks = 0;
int lastEncoderTicks[2] {0};

// PID / Motor Book-keeping:
bool usePID = true, encoderReset = false;
int motorVelocities[2] {0};

// Parameters for the PID:
double setPointLeft = 0, inputLeft = 0, outputLeft = 0;
double setPointRight = 0, inputRight = 0, outputRight = 0;

//PID control parameters: 
double kpL = 1.5, kiL = 1.0, kdL = 0.0;
double kpR = 1.5, kiR = 1.0, kdR = 0.0;

PID leftPID(&inputLeft, &outputLeft, &setPointLeft, kpL, kiL, kdL, DIRECT);
PID rightPID(&inputRight, &outputRight, &setPointRight, kpR, kiR, kdR, DIRECT);

// Functions to use
void launch(char functionToCall, String ticksPerSecondMotor1 = "", String ticksPerSecondMotor2 = "");
void updatePID();
void changeSetpoints(String setpoint1Str, String setpoint2Str);
void sendEncoderCounts();
void tickLeft();
void tickRight();

class motorDriver
{
  int in1, in2, pwm;
public:
  motorDriver(bool isLeftSide)
  {
    // Determines if it is talking to the left motor controller or the right one
    if(isLeftSide)
    {
      // Pins are flipped for left side so backwards --> forwards
      in1 = LEFTINPUT2;
      in2 = LEFTINPUT1;
      pwm = LEFTPWM;
    }
    else
    {
      in1 = RIGHTINPUT1;
      in2 = RIGHTINPUT2;
      pwm = RIGHTPWM;
    }
    
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(pwm, OUTPUT);
  }

  /*
   * Moves that motor based on a PWM input
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
};

// Initialize the motors
motorDriver leftMotor(true);
motorDriver rightMotor(false);

// Loop variables:
String arg1 = "", arg2 = "";
char mode, currentChar;
int spaceCount = 0;

void setup()
{
  Serial.begin(57600);
  
  // Assign all of the output pins
  pinMode(RIGHTINPUT1, OUTPUT);
  pinMode(RIGHTINPUT2, OUTPUT);
  pinMode(LEFTINPUT1, OUTPUT);
  pinMode(LEFTINPUT2, OUTPUT);

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  //leftPID.SetSampleTime(PIDINTERVAL);
  //rightPID.SetSampleTime(PIDINTERVAL);
  leftPID.SetOutputLimits(-255, 255);
  rightPID.SetOutputLimits(-255, 255);

  attachInterrupt(digitalPinToInterrupt(LEFTENCODERB), tickLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHTENCODERB), tickRight, RISING);

  lastTime[0] = millis();
  lastTime[1] = millis();
}

void loop() {
  while(Serial.available() > 0)
  {
    currentChar = Serial.read();

    // Once \r is reached send the command
    if(currentChar == 13)
    {
      if(mode == 'm' || mode == 'o')
      {
        launch(mode, arg1, arg2);
        
        // Reset the amount of spaces and arguments for next usage
        spaceCount = 0;
        arg1 = "";
        arg2 = "";
      }
      else
      {
        launch(mode);
      }
    }
    else if(currentChar == ' ')
    {
       spaceCount++;
    }
    else
    {
      // If no spaces have been found yet then it is the mode set character
      // If one space has been found then it is argument 1 and if two spaces found then it is argument 2
      if(spaceCount == 0)
      {
        mode = currentChar;
      }
      else if(spaceCount == 1)
      {
        arg1 += currentChar;
      }
      else if(spaceCount == 2)
      {
        arg2 += currentChar;
      }
    }
  }

  if(usePID == true)
  {
    updatePID();
  }
  else if (usePID == false)
  {
    leftMotor.move(motorVelocities[0]);
    rightMotor.move(motorVelocities[1]);
  }

  if((millis() - lastCommandRecv) > TIMEOUT)
  {
    timeout = true;
    leftMotor.move(0);
    rightMotor.move(0);
    usePID = false;
    motorVelocities[0] = 0;
    motorVelocities[1] = 0;
  }
}

void launch(char functionToCall, String ticksPerSecondMotor1 = "", String ticksPerSecondMotor2 = "")
{
  switch(functionToCall)
  {
    case('m'):
      lastCommandRecv = millis();
      changeSetpoints(ticksPerSecondMotor1, ticksPerSecondMotor2);
      if(timeout == true)
      {
        timeout = false;
      }
      Serial.println("OK");
      break;
      
    case('e'):
      sendEncoderCounts();
      break;

    case('o'):
      lastCommandRecv = millis();
      motorVelocities[0] = ticksPerSecondMotor1.toInt();
      motorVelocities[1] = ticksPerSecondMotor2.toInt();
      usePID = false;
      if(timeout == true)
      {
        timeout = false;
      }
      Serial.println("OK");
      break;

    case('r'):
      leftEncoderTicks = 0;
      rightEncoderTicks = 0;
      break;

    default:
      Serial.println("Invalid Input");
      break;
  }
}

void updatePID()
{
  float inputTicksLeft = leftEncoderTicks - lastEncoderTicks[0], inputTicksRight = rightEncoderTicks - lastEncoderTicks[1]; // Difference in encoder ticks
  float deltaTLeft = (millis() - lastTime[0]) * 0.001, deltaTRight = (millis() - lastTime[1]) * 0.001; // Want time to be in seconds
  lastTime[0] = millis();
  lastTime[1] = millis();
  lastEncoderTicks[0] = leftEncoderTicks;
  lastEncoderTicks[1] = rightEncoderTicks;

  /* To convert encoder ticks to velocity:
   *  
   *  Change in Ticks     Wheel Circumference
   *  ---------------- * ---------------------- * Gear Ratio
   *  Change in Time      Ticks Per Revolution
   *  
   */
  if(inputTicksLeft != 0)
  {
    float velLeft = ((float)inputTicksLeft / (TICKSPERREV * GEARRATIO)) * ((2 * PI* WHEELRADIUS) / deltaTLeft);
    inputLeft = (double)((velLeft / ((2 * PI) / (GEARRATIO * TICKSPERREV))) / (float)PIDRATE) * GAINFACTOR;
  }
  else
  {
    inputLeft = 0;
  }

  if(inputTicksRight != 0)
  {
    float velRight = ((float)inputTicksRight / (TICKSPERREV * GEARRATIO)) * ((2 * PI* WHEELRADIUS) / deltaTRight);
    inputRight = (double)((velRight / ((2 * PI) / (GEARRATIO * TICKSPERREV))) / (float)PIDRATE) * GAINFACTOR;
  }
  else
  {
    inputRight = 0;
  }

  leftPID.Compute();
  rightPID.Compute();

  Serial.println("Outputs Before Gain: ");
  Serial.println(outputLeft);
  Serial.println(outputRight);
  
  Serial.println("Inputs: ");
  Serial.println(inputLeft);
  Serial.println(inputRight);

  Serial.println("Setpoints: ");
  Serial.println(setPointLeft);
  Serial.println(setPointRight);

  leftMotor.move(outputLeft);
  rightMotor.move(outputRight);
}

void changeSetpoints(String setpoint1Str, String setpoint2Str)
{
  /*
   * Note: Setpoint appears to be given in units of ((m/s)/rad)/Hz
   */
  setPointLeft = (double)setpoint1Str.toFloat() * GAINFACTOR;
  setPointRight = (double)setpoint2Str.toFloat() * GAINFACTOR;

  // If the PID set point is to stop the motor then manually stop
  if(setPointLeft == 0 && setPointRight == 0)
  {
    motorVelocities[0] = 0;
    motorVelocities[1] = 0;
    usePID = false;
  }
  else
  {
    usePID = true;
  }
}

void sendEncoderCounts()
{
  Serial.println(leftEncoderTicks);
  Serial.println(" ");
  Serial.println(rightEncoderTicks);
}

void tickLeft()
{
  if(digitalRead(LEFTENCODERA) == LOW)
  {
    leftEncoderTicks--;
  }
  else
  {
    leftEncoderTicks++;
  }
}

void tickRight()
{
  if(digitalRead(RIGHTENCODERA) == LOW)
  {
    rightEncoderTicks++;
  }
  else
  {
    rightEncoderTicks--;
  }
}
