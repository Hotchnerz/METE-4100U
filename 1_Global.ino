/*
SARbot Phase C - Group 6 Code
*/
#include <CytronMotorDriver.h>
#include <Servo.h>

// declare all variables and pin assignments
Servo arm;

// defining quadrature encoders A & B from left and right motors 
#define L_ENCODER_A 9 // LEFT_MOTOR_CHANNEL_A
#define L_ENCODER_B 10 // LEFT_MOTOR_CHANNEL_B
#define R_ENCODER_A 11 // RIGHT_MOTOR_CHANNEL_A
#define R_ENCODER_B 12 // RIGHT_MOTOR_CHANNEL_B

// defining Cytron motor driver control pins for left and right motors
#define L_ENABLE 5 // LEFT_MOTOR_ENABLE (PWM)
#define L_MOT_DIR 4 // LEFT_MOTOR_DIRECTION
#define R_ENABLE 6 // RIGHT_MOTOR_ENABLE (PWM)
#define R_MOT_DIR 7 // RIGHT_MOTOR_DIRECTION

// motor driver used pins 3,4,5,6, due pwm pins = 2-13
// from testing, 1478 ticks is ~1 full wheel rotation
CytronMD left_motor(PWM_DIR, L_ENABLE, L_MOT_DIR);  // PWM 1 = Pin 7, DIR 1 = Pin 8.
CytronMD right_motor(PWM_DIR, R_ENABLE, R_MOT_DIR); // PWM 2 = Pin 5, DIR 2 = Pin 6.

// INPUTS
double left_motor_corr; // correction signal for fwd movement
double right_motor_corr;
const double left_motor_base_speed = 93; // base motor speed absolute pwm value
const double right_motor_base_speed = 73;
double left_motor_speed = 0; // final calculated speed
double right_motor_speed = 0;

// for PID calculations
long prevT = 0;
float eprev = 0;
float eIntegral = 0;

volatile long encoderLeftPos = 0;
volatile long encoderRightPos = 0;

int calcPWML_test = 0;
int calcPWMR_test = 0;
int targDisttL = 0;
int targDistR = 0;

// PID constants
float KpL = 4.5;
float KiL = 0;
float KdL = 0.2;

float KpR = 4;
float KiR = 0;
float KdR = 0.01;

bool flag = true;
int currentSensorState;
int lastSensorState;
unsigned long readTimer;
unsigned long currentMillis;
unsigned long prevTime = 0;

// infrared sensor, where ir_state = 0 when detected
int front_ir = 51;
int front_ir_state = 1;
int bot_ir = 8;
int bot_ir_state = 1;
int left_ir = 53;
int left_ir_state = 1;
int right_ir = 2;
int right_ir_state = 0;
int left_diagonal_ir = 23;
int left_diagonal_ir_state = 1;
int right_diagonal_ir = 3;
int right_diagonal_ir_state = 1;

bool miner = false;

//Debugging LED variables
int rightLED = 52;
int leftLED = 22; 
