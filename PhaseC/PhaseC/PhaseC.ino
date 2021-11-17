/*
   SARbot Phase B-C - Group 6 Code
*/
#include <CytronMotorDriver.h> // motor driver library 
#include <Encoder.h> // encoder library 
#include <PID_v1.h> // PID library 
#include <Servo.h>

Servo arm;
// declare variables
// all digital pins on the due are interrupt pins
#define left_encoder_channel_A 9 // for best performance attach all the encoder channels to interrupt pins 
#define left_encoder_channel_B 10
#define right_encoder_channel_A 11
#define right_encoder_channel_B 12
Encoder left_wheel_encoder(left_encoder_channel_A, left_encoder_channel_B);
Encoder right_wheel_encoder(right_encoder_channel_A, right_encoder_channel_B);

// motor driver used pins 3,4,5,6, due pwm pins = 2-13
// from testing, 5800 ticks is ~1 full wheel rotation
CytronMD left_motor(PWM_DIR, 5, 4);  // PWM 1 = Pin 7, DIR 1 = Pin 8.
CytronMD right_motor(PWM_DIR, 6, 7); // PWM 2 = Pin 5, DIR 2 = Pin 6.

bool flag = true;
// infrared sensor
int front_ir = 51;
int front_ir_state = 1;
int bot_ir = 8;
int bot_ir_state = 1;
int left_ir = 53;
int left_ir_state = 1;
int right_ir = 2;
int right_ir_state = 1;
int left_diagonal_ir = 23;
int left_diagonal_ir_state = 1;
int right_diagonal_ir = 3;
int right_diagonal_ir_state = 1;

// Maze Mapping Variables
const int rows = 6;
const int columns = 4;
int floor1[rows][columns];
const int x_start = 2;
const int y_start = 2;
const int x_ramp = 2;
const int y_ramp = 5;
int start_point[2] = {x_start, y_start};

// distance control variables
double setpoint_dist = 0; // 6.5 cm
/*double kp = 2.6;
  double ki = 0;
  double kd = 0;//0.1; */
// inputs
double avg_left_dist; //average calculated ir distance
double avg_right_dist;
// output is pwm speed control
double left_motor_corr; // conrrection signal
double right_motor_corr;
const double motor_base_speed = 60; // base motor speed absolute pwm value
double left_motor_speed; // final calculated speed
double right_motor_speed;
