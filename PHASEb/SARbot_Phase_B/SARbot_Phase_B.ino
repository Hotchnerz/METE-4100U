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
int right_ir = 23;
int right_ir_state = 1;
int left_diagonal_ir = 25;
int left_diagonal_ir_state = 1;
int right_diagonal_ir = 49;
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
const double motor_base_speed = 40; // base motor speed absolute pwm value
double left_motor_speed; // final calculated speed
double right_motor_speed;

//double diff;
//double motor_pwm;
// create PID object
/*PID left_motor_PID(&avg_left_dist, &left_motor_pwm, &setpoint_dist, kp, ki, kd, DIRECT); // left motor pid instance
  PID right_motor_PID(&avg_right_dist, &right_motor_pwm, &setpoint_dist, kp, ki, kd, DIRECT); // right motor pid instance*/

/*
  // moving average of distance measurements LEFT ***NOT USED
  float left_average(float data) { // raw sensor value
  float calculated_distance = -0.0809 * (data) + 60.581; // calculate the distance of the object from the sensor
  float average_distances = 0;
  for (int i = 0; i < arraylen - 1; i++) {
    left_distances[i] = left_distances[i + 1]; // push all old values down by one and pop out oldest
  }
  left_distances[arraylen - 1] = calculated_distance; // add the newest measurement
  for (int i = 0; i < arraylen; i++) {
    average_distances += left_distances[i];  // sum the array
  }
  average_distances = average_distances / float(arraylen);
  if (average_distances <= 0) {
    average_distances == 0;
  }
  return average_distances; // return the average of the history of measurments
  }
  /
  // moving average of distance measurements RIGHT ***NOT USED
  float right_average(float data) { // raw sensor value
  float calculated_distance = -0.0809 * (data) + 60.581; // calculate the distance of the object from the sensor
  float average_distances = 0;
  for (int i = 0; i < arraylen - 1; i++) {
    right_distances[i] = right_distances[i + 1]; // push all old values down by one and pop out oldest
  }
  right_distances[arraylen - 1] = calculated_distance; // add the newest measurement
  for (int i = 0; i < arraylen; i++) {
    average_distances += right_distances[i];  // sum the array
  }
  average_distances = average_distances / float(arraylen);
  if (average_distances <= 0) {
    average_distances == 0;
  }
  return average_distances;  // return the average of the history of measurements
  }
*/

// function to count the wheel rotations
float countRotations(float encoder_position) {
  float rotations = encoder_position / 5800;
  return rotations;
}

// correct speed function
void correctSpeed() {
  if (right_diagonal_ir_state == 0) {
    right_motor_corr += 0.16;
  } else if (left_diagonal_ir_state == 0) {
    left_motor_corr += 0.16;
  } else {
    left_motor_corr = 0;
    right_motor_corr = 0;
  }
  left_motor_speed = motor_base_speed + left_motor_corr + 1;//constrain((motor_base_speed + left_motor_corr + 3), motor_base_speed, 90);
  right_motor_speed = motor_base_speed + right_motor_corr;//constrain((motor_base_speed + right_motor_corr), motor_base_speed, 90);;
}

// move forward
void moveFwd() {
  correctSpeed();
  left_motor.setSpeed(left_motor_speed);
  right_motor.setSpeed(right_motor_speed);
}

// move forward
void moveFwdABit() {
  const float setpoint_rot = 0.25; // amount of wheel rotations
  float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0; // initialize the rotation counter
  float right_rot = 0;
  // perform right turn
  while ((right_rot >= (-1 * setpoint_rot)) or (left_rot >= (-1 * setpoint_rot))) {
    float left_enc = left_wheel_encoder.read(); // read wheel encoder values
    float right_enc = right_wheel_encoder.read();
    right_rot = initial_right_rot - countRotations(right_enc);
    left_rot = initial_left_rot - countRotations(left_enc); // find the difference between the initial and current rotations
    //correctSpeed();
    right_motor.setSpeed(motor_base_speed);
    left_motor.setSpeed(motor_base_speed); // set turning speed (pwm)
  }
  left_motor.setSpeed(0); // set turning speed (pwm)
  right_motor.setSpeed(0);
}

void moveFwdMiner() {
  const float setpoint_rot = 0.60; // amount of wheel rotations
  float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0; // initialize the rotation counter
  float right_rot = 0;
  // perform right turn
  while ((right_rot >= (-1 * setpoint_rot)) or (left_rot >= (-1 * setpoint_rot))) {
    float left_enc = left_wheel_encoder.read(); // read wheel encoder values
    float right_enc = right_wheel_encoder.read();
    right_rot = initial_right_rot - countRotations(right_enc);
    left_rot = initial_left_rot - countRotations(left_enc); // find the difference between the initial and current rotations
    //correctSpeed();
    right_motor.setSpeed(motor_base_speed);
    left_motor.setSpeed(motor_base_speed); // set turning speed (pwm)
  }
  left_motor.setSpeed(0); // set turning speed (pwm)
  right_motor.setSpeed(0);
}

// move forward
void reverseABit() {
  const float setpoint_rot = 0.12; // amount of wheel rotations
  float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0; // initialize the rotation counter
  float right_rot = 0;
  // perform right turn
  while ((right_rot <= setpoint_rot) or (left_rot <= setpoint_rot)) {
    float left_enc = left_wheel_encoder.read(); // read wheel encoder values
    float right_enc = right_wheel_encoder.read();
    right_rot = initial_right_rot - countRotations(right_enc);
    left_rot = initial_left_rot - countRotations(left_enc); // find the difference between the initial and current rotations
    right_motor.setSpeed(-35);
    left_motor.setSpeed(-35); // set turning speed (pwm)
  }
  left_motor.setSpeed(0); // set turning speed (pwm)
  right_motor.setSpeed(0);
}

// 90 degree right turn function
void turnRight() {
  const float setpoint_rot = 0.300; // amount of wheel rotations
  float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0; // initialize the rotation counter
  float right_rot = 0;
  // perform right turn
  while ((right_rot <= setpoint_rot) or (left_rot >= (-1 * setpoint_rot))) {
    float left_enc = left_wheel_encoder.read(); // read wheel encoder values
    float right_enc = right_wheel_encoder.read();
    right_rot = initial_right_rot - countRotations(right_enc);
    left_rot = initial_left_rot - countRotations(left_enc); // find the difference between the initial and current rotations
    right_motor.setSpeed(-35);
    left_motor.setSpeed(35); // set turning speed (pwm)
  }
  left_motor.setSpeed(0); // set turning speed (pwm)
  right_motor.setSpeed(0);
}

// left turn function
void turnLeft() {
  const float setpoint_rot = 0.30; // amount of wheel rotations
  float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0; // initialize the rotation counter
  float right_rot = 0;
  // perform left turn
  while ((left_rot <= setpoint_rot) or (right_rot >= (-1 * setpoint_rot))) {
    float left_enc = left_wheel_encoder.read(); // read wheel encoder values
    float right_enc = right_wheel_encoder.read();
    right_rot = initial_right_rot - countRotations(right_enc);
    left_rot = initial_left_rot - countRotations(left_enc); // find the difference between the initial and current rotations
    left_motor.setSpeed(-35);
    right_motor.setSpeed(35); // set turning speed (pwm)
  }
  left_motor.setSpeed(0); // set turning speed (pwm)
  right_motor.setSpeed(0);
}

//performs u-turn
void uTurn() {
  const float setpoint_rot = 0.90; // amount of wheel rotations
  float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0; // initialize the rotation counter
  float right_rot = 0;

  while ((right_rot <= setpoint_rot) or (left_rot >= -1 * setpoint_rot)) {
    float left_enc = left_wheel_encoder.read(); // read wheel encoder values
    float right_enc = right_wheel_encoder.read();
    right_rot = initial_right_rot - countRotations(right_enc);
    left_rot = initial_left_rot - countRotations(left_enc); // find the difference between the initial and current rotations
    if (right_rot <= setpoint_rot) {
      right_motor.setSpeed(-40);
    } else
      right_motor.setSpeed(0);
    if (left_rot >= -1 * setpoint_rot) {
      left_motor.setSpeed(40); // set turning speed (pwm)
    } else
      left_motor.setSpeed(0);
  }
  left_motor.setSpeed(0); // set turning speed (pwm)
  right_motor.setSpeed(0);
}

// get wheel encoder rotation
float getRotations(float encoder_position) {
  float rotations = encoder_position / 5800;
  return rotations;
}

// interrupt service routines
void front_state_isr() {
  front_ir_state = digitalRead(front_ir);
}
void bot_state_isr() {
  bot_ir_state = digitalRead(bot_ir);
}
void left_diagonal_state_isr() {
  left_diagonal_ir_state = digitalRead(left_diagonal_ir);
}
void right_diagonal_state_isr() {
  right_diagonal_ir_state = digitalRead(right_diagonal_ir);
}
void left_state_isr() {
  left_ir_state = digitalRead(left_ir);
}
void right_state_isr() {
  right_ir_state = digitalRead(right_ir);
}

void printMaze() {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < columns; j++) {
      if ((j == x_start) && (i == y_start)) {
        Serial.print("R");
      } else if ((j == x_ramp) && (i == y_ramp)) {
        Serial.print("/");
      } else
        Serial.print(floor1[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

bool miner = false;
void wallFollow() {
  if ((front_ir_state == 1) && (bot_ir_state == 0) && (miner == false)) {
    // see miner
    if (miner == false) {
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      delay(1000);
      reverseABit();
      reverseABit();
      reverseABit();
      reverseABit();
      arm.write(0);
      delay(1000);
      moveFwdMiner(); 
      arm.write(180);
      delay(1000);
      if ((bot_ir_state == 0)) {
        miner = true;
      }else if (bot_ir_state == 1){
        reverseABit();
      }
    }
  } /*else if ((front_ir_state == 1) && (right_ir_state == 0) && (left_ir_state == 1)) {
    //left_turn_special
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
  } */else if ((front_ir_state == 1) && (right_ir_state == 0)) {
    // forward moving
    moveFwd();
  } else if (right_ir_state == 1) {
    // turn right
    moveFwdABit();
    turnRight();
    moveFwdABit();
    moveFwdABit();
    moveFwdABit();

  } else if ((front_ir_state == 0) && (right_ir_state == 0) && (left_ir_state == 1)) {
    // turn left
    moveFwdABit();
    turnLeft();
    moveFwdABit();
    moveFwdABit();
    moveFwdABit();
  } else if ((front_ir_state == 0) && (right_ir_state == 0) && (left_ir_state == 0)) {
    // uTurn
    moveFwdABit();
    moveFwdABit();
    uTurn();
  } 
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  arm.attach(13);
  arm.write(180);
  // initialize ir pins and interrupt pins
  pinMode(front_ir, INPUT);
  pinMode(bot_ir, INPUT);
  pinMode(left_diagonal_ir, INPUT);
  pinMode(right_diagonal_ir, INPUT);
  pinMode(left_ir, INPUT);
  pinMode(right_ir, INPUT);
  attachInterrupt(digitalPinToInterrupt(front_ir), front_state_isr, CHANGE); // initialize interrupt pin
  attachInterrupt(digitalPinToInterrupt(bot_ir), bot_state_isr, CHANGE); // initialize interrupt pin
  attachInterrupt(digitalPinToInterrupt(left_diagonal_ir), left_diagonal_state_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_diagonal_ir), right_diagonal_state_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_ir), left_state_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_ir), right_state_isr, CHANGE);
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  delay(10000);
  // init PID
  /*left_motor_PID.SetMode(AUTOMATIC);
    left_motor_PID.SetTunings(kp, ki, kd);
    right_motor_PID.SetMode(AUTOMATIC);
    right_motor_PID.SetTunings(kp, ki, kd);*/
  //motor_PID.SetMode(AUTOMATIC);
  //motor_PID.SetTunings(kp, ki, kd);
  //motor_PID.SetOutputLimits(-255, 255);
}

void loop() {
  // put your main code here, to run repeatedly:
  float left_enc = left_wheel_encoder.read();
  float right_enc = right_wheel_encoder.read();
  float left_rot = countRotations(left_enc); // convert the initial encoder value into rotation
  float right_rot = countRotations(right_enc); //180 mm is 0.93 rot
  wallFollow();
  //correctSpeed();
  Serial.print(left_motor_speed);
  Serial.print(" | ");
  Serial.print(right_motor_speed);
  Serial.print(" | ");
  Serial.print(front_ir_state);
  Serial.print(" | ");
  Serial.print(left_ir_state);
  Serial.print(" | ");
  Serial.print(right_ir_state);
  Serial.print(" | ");
  Serial.print(left_diagonal_ir_state);
  Serial.print(" | ");
  Serial.println(right_diagonal_ir_state);

}
