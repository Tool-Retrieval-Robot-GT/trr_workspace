#include <PinChangeInterrupt.h>

constexpr int MOTOROUT = 10;      // Motor PWM out
constexpr int DIRECTION = 11;     // Direction pin out

constexpr int TOPSWITCH = 5;      // Top sensor switch
constexpr int BOTTOMSWITCH = 6;   // Bottom sensor switch

constexpr int ENCODER1 = 8;       // First encoder pin
constexpr int ENCODER2 = 9;       // Second encoder pin

constexpr int MOTORSPEED = 100;   // Set the motor speed

volatile int encoderCount = 0;    // Encoder count
int bottomToTop = -1;             // Number of encoder ticks from bottom to top of forklift. Gets set after homing.

void updateEncoderCount();
void homingProcedure();
void toPos();
void topReached();
void bottomReached();

// Setup before main loop.
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(TOPSWITCH, INPUT);
  pinMode(ENCODER1, INPUT);
  pinMode(ENCODER2, INPUT);

  pinMode(MOTOROUT, OUTPUT);
  pinMode(DIRECTION, OUTPUT);

  attachPCINT(digitalPinToPCINT(ENCODER1), updateEncoderCount, RISING);

  attachPCINT(digitalPinToPCINT(TOPSWITCH), topReached, RISING);
  attachPCINT(digitalPinToPCINT(BOTTOMSWITCH), bottomReached, RISING);
}

// Main program loop.
void loop() {
  // Reenabling the end interrupts.
  enablePCINT(digitalPinToPCINT(TOPSWITCH));
  enablePCINT(digitalPinToPCINT(BOTTOMSWITCH));

  // Wait for a serial command.
  digitalWrite(LED_BUILTIN, HIGH);
  while (Serial.available() == 0);
  digitalWrite(LED_BUILTIN, LOW);

  // Read the serial command.
  String input = Serial.readString();
  char func = input[0];

  // Interpret the serial command.
  switch (func) {
    // "h" runs the homing procedure.
    case 'h':
      homingProcedure();
      break;
    // "p [0.0-1.0]" sets the fork position.
    case 'p':
      int spaceIndex = input.indexOf(' ');
      String numberStr = input.substring(spaceIndex + 1);
      float val = numberStr.toFloat();
      toPos(val);
      break;
    default: 
      break;
  }
  Serial.println("done");
}

// Update encoders on interrupt.
void updateEncoderCount() {
  // If ENCODER1 is high by the time it gets to ENCODER2 then it's moving clockwise.
  if (digitalRead(ENCODER1) == HIGH && digitalRead(ENCODER2) == LOW) {
    encoderCount++;
  } else {
    encoderCount--;
  }

  if (bottomToTop != -1 && (encoderCount > bottomToTop + 100 || encoderCount < -100)) {
    analogWrite(MOTOROUT, 0);
  }
}

// Runs the homing procedure
void homingProcedure() {
  // Lower the forklift as far as possible.
  digitalWrite(DIRECTION, 0);
  analogWrite(MOTOROUT, MOTORSPEED);
  while (digitalRead(BOTTOMSWITCH) == LOW);
  encoderCount = 0;

  // Raise the forklift as high as possible. Count the amount of ticks.
  digitalWrite(DIRECTION, 1);
  analogWrite(MOTOROUT, MOTORSPEED);
  while (digitalRead(TOPSWITCH) == LOW);
  bottomToTop = encoderCount;
}

// Moves the forklift to specified position.
// 1.0: top, 0.5: middle, 0.0: bottom
void toPos(float val) {
  // Doesn't run if forklift isn't homed or is provided an invalid value
  if (bottomToTop == -1 || val < 0.0 || val > 1.0) {
    return;
  }

  // Calculate the target amount of ticks and move the forks there
  int target = round(bottomToTop * val);

  // If the current position is within 20 encoder ticks, don't do anything
  if (abs(encoderCount - target) <= 20) {
    return;
  }
  
  int direction = encoderCount < target ? 1 : 0;
  digitalWrite(DIRECTION, direction);
  analogWrite(MOTOROUT, MOTORSPEED);
  if (direction == 0) {
    while (encoderCount > target);
  } else {
    while (encoderCount < target);
  }
  analogWrite(MOTOROUT, 0);
}

// Triggers when the top of the forklift is reached
void topReached() {
  analogWrite(MOTOROUT, 0);
  disablePCINT(digitalPinToPCINT(TOPSWITCH));
  enablePCINT(digitalPinToPCINT(BOTTOMSWITCH));
}

// Triggers when the bottom of the forklift is reached
void bottomReached() {
  analogWrite(MOTOROUT, 0);
  disablePCINT(digitalPinToPCINT(BOTTOMSWITCH));
  enablePCINT(digitalPinToPCINT(TOPSWITCH));
}
