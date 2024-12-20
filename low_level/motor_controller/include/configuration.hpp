#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#define VERSION "1.2.6"

// -----------------------------------------------------------------------------
// ---------------------------| Motor GPIO Pins |-------------------------------
// -----------------------------------------------------------------------------

// GPIO pin configuration for the right motor.
#define GPIO_MOTOR_RIGHT_EN         9   // Must be a PWM pin
#define GPIO_MOTOR_RIGHT_IN1        12
#define GPIO_MOTOR_RIGHT_IN2        11
#define GPIO_MOTOR_RIGHT_ENCODER_A  3  // Must be a hardware interrupt pin
#define GPIO_MOTOR_RIGHT_ENCODER_B  4

// GPIO pin configuration for the left motor.
#define GPIO_MOTOR_LEFT_EN          5   // Must be a PWM pin
#define GPIO_MOTOR_LEFT_IN1         7
#define GPIO_MOTOR_LEFT_IN2         8
#define GPIO_MOTOR_LEFT_ENCODER_A   2  // Must be a hardware interrupt pin
#define GPIO_MOTOR_LEFT_ENCODER_B   6

// -----------------------------------------------------------------------------
// ----------------------------| Motor Configuration |--------------------------
// -----------------------------------------------------------------------------

// Motor and encoder configuration settings.
#define WHEEL_RADIUS 0.04  // Wheel radius in meters
#define ENCODER_TICKS_PER_REVOLUTION 550
#define DIST_BETWEEN_WHEELS 0.47  // Distance between wheels in meters

// PID gains
#define MOTOR_DRIVER_PID_KP 200.0
#define MOTOR_DRIVER_PID_KI 70.0
#define MOTOR_DRIVER_PID_KD 30.0

#define MOTOR_RUN_FREQUENCY 20  // Motor run frequency (in Hz)

#define MOTOR_MAX_VELOCITY 1.0  // Maximum velocity in m/s

// -----------------------------------------------------------------------------
// ----------------------------| Serial Configuration |-------------------------
// -----------------------------------------------------------------------------

// If you change this and want to use platformIO monitor, you must also change
// the baud rate in platformio.ini
#define SERIAL_BAUD_RATE 9600

#endif  // !CONFIGURATION_HPP
