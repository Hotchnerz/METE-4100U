/*
   SARbot Code Phase A
*/
#include <CytronMotorDriver.h> // motor driver library 
#include <Encoder.h> // encoder library 

// declare variables
// all digital pins on the due are interrupt pins
#define left_encoder_channel_A 9 // for best performance attach all the encoder channels to interrupt pins 
#define left_encoder_channel_B 10
#define right_encoder_channel_A 11
#define right_encoder_channel_B 12
Encoder left_wheel_encoder(left_encoder_channel_A, left_encoder_channel_B);
Encoder right_wheel_encoder(right_encoder_channel_A, right_encoder_channel_B);
float oldPosition  = -999;

int left_motor_speed = 0;
int right_motor_speed = 0;

// motor driver used pins 3,4,5,6, due pwm pins = 2-13
// from testing, 5800 ticks is ~1 full wheel rotation
CytronMD left_motor(PWM_DIR, 5, 4);  // PWM 1 = Pin 7, DIR 1 = Pin 8.
CytronMD right_motor(PWM_DIR, 6, 7); // PWM 2 = Pin 5, DIR 2 = Pin 6.

// infrared sensors
int front_ir = 22;
int left_ir = 24;
int right_ir = 26;
int left_diagonal_ir = 28;
int right_diagonal_ir = 30;
int front_ir_state = 1;
int left_ir_state = 1;
int right_ir_state = 1;
int left_diagonal_ir_state = 1;
int right_diagonal_ir_state = 1;

// function to count the wheel rotations
float countRotations(float encoder_position) {
  float rotations = encoder_position / 5800;
  return rotations;
}

// interrupt service routines
void front_state_isr() {
  front_ir_state = digitalRead(front_ir);
}
void left_state_isr() {
  left_ir_state = digitalRead(left_ir);
}
void right_state_isr() {
  right_ir_state = digitalRead(right_ir);
}
void left_diagonal_state_isr() {
  left_diagonal_ir_state = digitalRead(left_diagonal_ir);
}
void right_diagonal_state_isr() {
  right_diagonal_ir_state = digitalRead(right_diagonal_ir);
}

// correct speed function
void correctSpeed() {
  if ((right_diagonal_ir_state == 0) && (left_diagonal_ir_state == 1)){
    right_motor_speed += 0.11;
  } else if ((left_diagonal_ir_state == 0) && (right_diagonal_ir_state == 1)) {
    left_motor_speed += 0.11;
  } else {
    left_motor_speed = -50;
    right_motor_speed = -50;
  }
}

// left turn 90 deg
void turnLeft() {
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_enc = left_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc);
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0;
  float right_rot = 0;
  while ((left_rot <= 0.15) && (right_rot <= 0.15)) {
    correctSpeed();
    float left_enc = left_wheel_encoder.read();
    float right_enc = right_wheel_encoder.read();
    left_rot = countRotations(left_enc) - initial_left_rot;
    right_rot = countRotations(right_enc) - initial_right_rot;
    left_motor.setSpeed(left_motor_speed);
    right_motor.setSpeed(right_motor_speed);
  }
  left_rot = 0;
  right_rot = 0;
  while ((left_rot <= 0.50) && (right_rot >= -0.50)) {
    float left_enc = left_wheel_encoder.read();
    float right_enc = right_wheel_encoder.read();
    left_rot = initial_left_rot - countRotations(left_enc);
    right_rot = initial_right_rot - countRotations(right_enc);
    left_motor.setSpeed(-50);
    right_motor.setSpeed(50);
  }
  //left_motor.setSpeed(0);
  //right_motor.setSpeed(0);
}

// uTurn 
void uTurn() {
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_enc = left_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc);
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0;
  float right_rot = 0;
  while ((left_rot <= 0.8) && (right_rot >= -0.8)) {
    float left_enc = left_wheel_encoder.read();
    float right_enc = right_wheel_encoder.read();
    left_rot = initial_left_rot - countRotations(left_enc);
    right_rot = initial_right_rot - countRotations(right_enc);
    left_motor.setSpeed(-50);
    right_motor.setSpeed(50);
  }
}

// turn right 90 deg
void turnRight() {
  float initial_left_enc = left_wheel_encoder.read();
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc);
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0;
  float right_rot = 0;
  while ((left_rot <= 0.15) && (right_rot <= 0.15)) {
    correctSpeed();
    float left_enc = left_wheel_encoder.read();
    float right_enc = right_wheel_encoder.read();
    left_rot = countRotations(left_enc) - initial_left_rot;
    right_rot = countRotations(right_enc) - initial_right_rot;
    left_motor.setSpeed(left_motor_speed);
    right_motor.setSpeed(right_motor_speed);
  }
  left_rot = 0;
  right_rot = 0;
  while ((left_rot >= -0.50) && (right_rot <= 0.5
  0)) {
    float left_enc = left_wheel_encoder.read();
    float right_enc = right_wheel_encoder.read();
    left_rot = initial_left_rot - countRotations(left_enc);
    right_rot = initial_right_rot - countRotations(right_enc);
    left_motor.setSpeed(50);
    right_motor.setSpeed(-50);
  }
  left_rot = 0;
  right_rot = 0;
  while ((left_rot <= 1.6) && (right_rot <= 1.6)) {
    correctSpeed();
    float left_enc = left_wheel_encoder.read();
    float right_enc = right_wheel_encoder.read();
    left_rot = countRotations(left_enc) - initial_left_rot;
    right_rot = countRotations(right_enc) - initial_right_rot;
    left_motor.setSpeed(left_motor_speed);
    right_motor.setSpeed(right_motor_speed);
  }
  //left_motor.setSpeed(0);
  //right_motor.setSpeed(0);
}

// forward 1 cell step
void fwdStep() {
  float initial_left_enc = left_wheel_encoder.read();
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc);
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0;
  float right_rot = 0;
  while ((left_rot <= 0.93) && (right_rot <= 0.93)) {
    correctSpeed();
    if ((front_ir_state == 0) || (right_ir_state == 1)) {
      break;
    }
    float left_enc = left_wheel_encoder.read();
    float right_enc = right_wheel_encoder.read();
    left_rot = countRotations(left_enc) - initial_left_rot;
    right_rot = countRotations(right_enc) - initial_right_rot;
    left_motor.setSpeed(left_motor_speed);
    right_motor.setSpeed(right_motor_speed);
  }
  //left_motor.setSpeed(0);
  //right_motor.setSpeed(0);
}


// follow wall
void wallFollow() {
  if ((front_ir_state == 1) && (right_ir_state == 0)) {
    fwdStep();
  } else if (right_ir_state == 1) {
    turnRight();
  } else if ((front_ir_state == 0) && (right_ir_state == 0) && (left_ir_state == 0)) {
    uTurn(); 
  } else if ((front_ir_state == 0) && (right_ir_state == 0)) {
    turnLeft();
  }
}

void setup() {
  Serial.begin(9600);
  // initialize ir pins and interrupt pins
  pinMode(front_ir, INPUT);
  pinMode(left_ir, INPUT);
  pinMode(right_ir, INPUT);
  pinMode(left_diagonal_ir, INPUT);
  pinMode(right_diagonal_ir, INPUT);
  attachInterrupt(digitalPinToInterrupt(front_ir), front_state_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_ir), left_state_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_ir), right_state_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_diagonal_ir), left_diagonal_state_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_diagonal_ir), right_diagonal_state_isr, CHANGE);
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  delay(1500);
}

void loop() {
  float left_enc = left_wheel_encoder.read();
  float right_enc = right_wheel_encoder.read();
  float left_rot = countRotations(left_enc);
  float right_rot = countRotations(right_enc);
  wallFollow();

  /*
    Serial.print("front ir: ");
    Serial.print(front_ir_state);
    Serial.print(" left ir: ");
    Serial.print(left_ir_state);
    Serial.print(" right ir: ");
    Serial.print(right_ir_state);
    Serial.print(" left-d ir: ");
    Serial.print(left_diagonal_ir_state);
    Serial.print(" right-d ir: ");
    Serial.println(right_diagonal_ir_state);*/
}
