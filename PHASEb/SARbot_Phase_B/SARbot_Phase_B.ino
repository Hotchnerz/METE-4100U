/*
   SARbot Phase B - Group 6 Code
*/
#include <CytronMotorDriver.h> // motor driver library 
#include <Encoder.h> // encoder library 
#include <PID_v1.h> // PID library 

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

// infrared sensor
int front_ir = 25;
int front_ir_state = 1;
int bot_ir = 23;
int bot_ir_state = 1;

int left_IR_control = 2; //high low active/stand-by control
int left_IR_input = A0;
int right_IR_control = 3; //high low active/stand-by control
int right_IR_input = A1;
float left_IR_data = 0;
float left_distances[25]; // array to hold left distance measurements
float right_IR_data = 0;
float right_distances[25]; // array to hold right distance measurements
byte arraylen = sizeof(left_distances) / sizeof(left_distances[0]); // length of array

// Maze Mapping Variables
const int rows = 6;
const int columns = 4;
int floor1[rows][columns];
const int x_start = 2;
const int y_start = 2;
const int x_ramp = 2;
const int y_ramp = 5;
int start_point[2] = {x_start, y_start};

// PID distance control variables
double setpoint_dist = 0; // 6.5 cm
double kp = 5;
double ki = 0;
double kd = 0;
// inputs
double avg_left_dist; //average calculated ir distance
double avg_right_dist;
// output is pwm speed control
double left_motor_pwm; // control signal output
double right_motor_pwm;
const double motor_base_speed = 50; // base motor speed absolute pwm value
double left_motor_speed;
double right_motor_speed;

double diff;
double motor_pwm;
// create PID object
/*PID left_motor_PID(&avg_left_dist, &left_motor_pwm, &setpoint_dist, kp, ki, kd, DIRECT); // left motor pid instance
PID right_motor_PID(&avg_right_dist, &right_motor_pwm, &setpoint_dist, kp, ki, kd, DIRECT); // right motor pid instance*/
PID motor_PID(&diff, &motor_pwm, &setpoint_dist, kp, ki, kd, DIRECT); // Zero Distance Design


// moving average of distance measurements LEFT
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
  return average_distances = average_distances / float(arraylen); // return the average of the history of measurments
}

// moving average of distance measurements RIGHT
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
  return average_distances = average_distances / float(arraylen); // return the average of the history of measurments
}

// function to count the wheel rotations
float countRotations(float encoder_position) {
  float rotations = encoder_position / 5800;
  return rotations;
}

// move forward
void moveFwd() {
  //left_motor_PID.Compute();
  //right_motor_PID.Compute();
  diff = avg_right_dist - avg_left_dist;
  motor_PID.Compute();
  /*left_motor_speed = constrain((motor_base_speed + (-1 * motor_pwm)), motor_base_speed, 100); // constrain var, min, max
  right_motor_speed = constrain((motor_base_speed + motor_pwm), motor_base_speed, 100);*/
  left_motor_speed = constrain((motor_base_speed + (-1 * motor_pwm)), motor_base_speed, 100); // constrain var, min, max
  right_motor_speed = ((motor_base_speed + motor_pwm), motor_base_speed,100);
  left_motor.setSpeed(left_motor_speed);
  right_motor.setSpeed(right_motor_speed);
}

// 90 degree right turn function
void turnRight() {
  const float offset_rot = 0.2; // amount of wheel rotations
  const float setpoint_rot = 0.45; // amount of wheel rotations
  float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0; // initialize the rotation counter
  float right_rot = 0;
  /*
    // compensate offset forward
    while ((right_rot >= -1 * offset_rot) or (left_rot >= -1 * offset_rot)) {
      float left_enc = left_wheel_encoder.read(); // read wheel encoder values
      float right_enc = right_wheel_encoder.read();
      right_rot = initial_right_rot - countRotations(right_enc);
      left_rot = initial_left_rot - countRotations(left_enc); // find the difference between the initial and current rotations
      if (right_rot >= -1 * offset_rot) {
        right_motor.setSpeed(40);
      } else
        right_motor.setSpeed(0);

      if (left_rot >= -1 * offset_rot) {
        left_motor.setSpeed(40); // set turning speed (pwm)
      } else
        left_motor.setSpeed(0);
      Serial.print(right_rot);
      Serial.print(" | ");
      Serial.println(left_rot);
    }
    left_motor.setSpeed(0); // set turning speed (pwm)
    right_motor.setSpeed(0);

    left_rot = 0; // re-initialize the rotation counter
    right_rot = 0;*/
  // perform right turn
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

// left turn function
void turnLeft() {
  const float setpoint_rot = 0.45; // amount of wheel rotations
  float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0; // initialize the rotation counter
  float right_rot = 0;

  // perform left turn
  while ((left_rot <= setpoint_rot) or (right_rot >= -1 * setpoint_rot)) {
    float left_enc = left_wheel_encoder.read(); // read wheel encoder values
    float right_enc = right_wheel_encoder.read();
    right_rot = initial_right_rot - countRotations(right_enc);
    left_rot = initial_left_rot - countRotations(left_enc); // find the difference between the initial and current rotations
    if (right_rot >= -1 * setpoint_rot) {
      right_motor.setSpeed(40);
    } else
      right_motor.setSpeed(0);

    if (left_rot <= setpoint_rot) {
      left_motor.setSpeed(-40); // set turning speed (pwm)
    } else
      left_motor.setSpeed(0);
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

void wallFollow() {
  if ((front_ir_state == 1) && (avg_right_dist <= 12.0)) {
    moveFwd();
  } else {
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // initialize ir pins and interrupt pins
  pinMode(front_ir, INPUT);
  attachInterrupt(digitalPinToInterrupt(front_ir), front_state_isr, CHANGE); // initialize interrupt pin
  pinMode(bot_ir, INPUT);
  attachInterrupt(digitalPinToInterrupt(bot_ir), bot_state_isr, CHANGE); // initialize interrupt pin

  pinMode(left_IR_control, OUTPUT);
  digitalWrite(left_IR_control, HIGH); // set state to active
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  pinMode(left_IR_control, OUTPUT);
  pinMode(right_IR_control, OUTPUT);
  digitalWrite(right_IR_control, HIGH);
  digitalWrite(left_IR_control, HIGH);
  // init PID
  /*left_motor_PID.SetMode(AUTOMATIC);
  left_motor_PID.SetTunings(kp, ki, kd);
  right_motor_PID.SetMode(AUTOMATIC);
  right_motor_PID.SetTunings(kp, ki, kd);*/
  motor_PID.SetMode(AUTOMATIC);
  motor_PID.SetTunings(kp, ki, kd);
  motor_PID.SetOutputLimits(-255, 255);
}

void loop() {
  int left_IR_data = analogRead(left_IR_input);
  avg_left_dist = left_average(left_IR_data);
  int right_IR_data = analogRead(right_IR_input);
  avg_right_dist = right_average(right_IR_data);
  // put your main code here, to run repeatedly:
  float left_enc = left_wheel_encoder.read();
  float right_enc = right_wheel_encoder.read();
  float left_rot = countRotations(left_enc); // convert the initial encoder value into rotation
  float right_rot = countRotations(right_enc); //180 mm is 0.93 rot
  wallFollow();
  /*Serial.print(avg_left_dist);
  Serial.print(",");
  Serial.print(setpoint_dist);
  Serial.print(",");
  Serial.println(avg_right_dist);*/motor_PID.Compute();
  left_motor_speed = motor_base_speed + (-1 * motor_pwm); // constrain var, min, max
  right_motor_speed = motor_base_speed + motor_pwm;
  
  //diff = avg_right_dist - avg_left_dist;
  
  /*motor_PID.Compute();
  left_motor_speed = motor_base_speed + (-1 * motor_pwm); // constrain var, min, max
  right_motor_speed = motor_base_speed + motor_pwm;
  */
  Serial.print("Left Sped: ");
  Serial.print(left_motor_speed);
  Serial.print(" | ");
  Serial.print("Right Sped: ");
  Serial.print(right_motor_speed);
  Serial.print(" | "); 
  Serial.print(avg_left_dist);
  Serial.print(" | "); 
  Serial.print(avg_right_dist);
  Serial.print(" | ");
  Serial.print(diff);
  Serial.print(" | ");
  Serial.println(motor_pwm);
}
