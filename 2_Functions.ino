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

// function to correct crashing diagonally into the wall during moving forward
/*
void sensorFailSafe(int sensorState) {
  currentSensorState = sensorState;
  if (sensorState == HIGH) {
    currentMillis = millis();
  } if (millis() - currentMillis >= 1500) {
    int targDistL = encoderLeftPos - 300;
    int targDistR = encoderRightPos - 300;
    while (encoderLeftPos != targDistL) { // reverse both wheels by 150 pulses to move away from stuck position
      int calcPWML = computePID(encoderLeftPos, targDistL, KpL, KiL, KdL);
      int calcPWMR = computePID(encoderRightPos, targDistR, KpR, KiR, KdR);
      setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
      setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
    }
    targDistL = encoderLeftPos + 100;
    targDistR = encoderRightPos - 100;
    while (encoderLeftPos != targDistL) { // straighten robot away from wall to point more towards center of laneway
      int calcPWML = computePID(encoderLeftPos, targDistL, KpL, KiL, KdL);
      int calcPWMR = computePID(encoderRightPos, targDistR, KpR, KiR, KdR);
      setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
      setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
    }
  }
}
*/
// Obtain value for left encoder pulses
void readEncoderLeft() {
  int bL = digitalRead(L_ENCODER_B);
  if (bL > 0) {
    encoderLeftPos++;
  } else {
    encoderLeftPos--;
  }
}

// Obtain value for right encoder pulses
void readEncoderRight() {
  int bR = digitalRead(R_ENCODER_B);
  if (bR > 0) {
    encoderRightPos++;
  } else {
    encoderRightPos--;
  }
}

// reset encoder counter to 0
void resetPos() {
  encoderLeftPos = 0;
  encoderRightPos = 0;
}

double computePID(long pulses, int target, float kP, float kI, float kD) {
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;
  double outputPWM;
  /*while (pulses != target) {
    // error calculation
    float e = target - pulses;

    // derivative calculation
    float dedt = (e - eprev) / (deltaT);

    // integral calculation
    eIntegral = eIntegral + e * deltaT;

    // control signal for PID
    outputPWM  = ((kP * e) + (kD * dedt) + (kI * eIntegral));
    eprev = e; // retain error for next loop
    return outputPWM;
    }*/
  // error calculation
  float e = target - pulses;

  // derivative calculation
  float dedt = (e - eprev) / (deltaT);

  // integral calculation
  eIntegral = eIntegral + e * deltaT;

  // control signal for PID
  outputPWM  = ((kP * e) + (kD * dedt) + (kI * eIntegral));
  eprev = e; // retain error for next loop
  return outputPWM;
}

// function to drive motor;
void setMotor(int outputPWM, int enablePin , int motDir) {
  int dir = 1; // dir: 1=fwd, 0=rev
  if (outputPWM < 0) {
    dir = 0;
  }
  int calcPWM = fabs(outputPWM);
  if (calcPWM > 255) { //max PWM
    calcPWM = 255;
  }
  analogWrite(enablePin, calcPWM); //send PWM signal to motor
  if (dir == 1) {
    digitalWrite(motDir, 0);
  } else {
    digitalWrite(motDir, 1);
  }
}

// stop motors
void stopMotors() {
  setMotor(0, L_ENABLE, L_MOT_DIR);
  setMotor(0, R_ENABLE, R_MOT_DIR);
}

// control both LEDs to the same state
void leds(bool State) {
  digitalWrite(leftLED, State);
  digitalWrite(rightLED, State);
}

// LED blinking function with number of blinks, LED variable, and delay time
void ledBlink(int count, int dirLED, int delayTime) {
  while (count > 0) {
    digitalWrite(dirLED, HIGH);
    delay(delayTime);
    digitalWrite(dirLED, LOW);
    delay(delayTime);
    count -= 1;
  }
}

// function to adjust speed of left/right motors for fwd movement only
void correctSpeed() {
  if (right_diagonal_ir_state == 0) { //correction to move away from right wall
    right_motor_corr += 0.001;
  } else if (left_diagonal_ir_state == 0) { //correction to move away from left wall
    left_motor_corr += 0.005;
  }
  else {
    left_motor_corr = 0;
    right_motor_corr = 0;
  }
  /*else if (left_ir_state == 1) { //redundant correction for opening in wall when approaching left turn
    left_motor_corr = 10;
    } else if (right_ir_state == 1) { //redundant correction for opening in wall when approaching right turn
    right_motor_corr = 10;
    }
  */
  // add motor speed correction to base speed and write to current motor speed
  left_motor_speed = left_motor_base_speed + constrain(left_motor_corr, 0, 100);
  right_motor_speed = right_motor_base_speed + constrain(right_motor_corr, 0, 100);
}

// function to move forward
void moveFwd() {
  leds(HIGH);
  correctSpeed();
  Serial.print(left_motor_speed);
  Serial.print(" ");
  Serial.println(right_motor_speed);
  Serial.print(" ");
  Serial.print(left_diagonal_ir_state);
  Serial.print(" ");
  Serial.println(right_diagonal_ir_state);
  left_motor.setSpeed(left_motor_speed);
  right_motor.setSpeed(right_motor_speed);
  /*
    if (bot_ir_state == 0) {
    ledBlink(leftLED, 6, 50);
    ledBlink(rightLED, 6, 50);
    stopMotors();
    delay(1000);
    int targDistL = encoderLeftPos - 250;
    int targDistR = encoderRightPos - 250;
    while (encoderLeftPos != targDistL) {
      int calcPWML = computePID(encoderLeftPos, targDistL, KpL, KiL, KdL);
      int calcPWMR = computePID(encoderRightPos, targDistR, KpR, KiR, KdR);
      setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
      setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
    }
    targDistL = encoderLeftPos - 300; //pulses
    targDistR = encoderRightPos + 300;
    while (targDistL != encoderLeftPos && targDistR != encoderRightPos) {
      int calcPWML = computePID(encoderLeftPos, targDistL, KpL, KiL, KdL);
      int calcPWMR = computePID(encoderRightPos, targDistR, KpR, KiR, KdR);
      setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
      setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
  */
}

// function to move forward a little
void fwdStep(int targetPulses) {
  ledBlink(2, rightLED, 75);
  ledBlink(2, leftLED, 75);
  leds(HIGH);
  stopMotors();
  delay(1000);
  resetPos();
  while (targetPulses != encoderLeftPos) {
    int calcPWML = computePID(encoderLeftPos, targetPulses, KpL, KiL, KdL); //return from computePID() function stored in calcPWML as outputPWM for setMotor()
    int calcPWMR = computePID(encoderRightPos, targetPulses, KpR, KiR, KdR);
    setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
    setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
  }
  /*if (bot_ir_state == 0) { // prevent forward crash
    stopMotors();
    resetPos();
    while (encoderLeftPos != -100 && encoderRightPos != -100) {
      int calcPWML = computePID(encoderLeftPos, -200, KpL, KiL, KdL); //command to reverse a bit if we get too close to the wall in fwdStep
      int calcPWMR = computePID(encoderRightPos, -200, KpR, KiR, KdR);
      setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
      setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
    }
    break;
    }
  */
  stopMotors();
}
/*
  float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0; // initialize the rotation counter
  float right_rot = 0;
  while ((left_rot <= setpoint_rot) && (right_rot <= setpoint_rot)) {
  correctSpeed();
  float left_enc = left_wheel_encoder.read();
  float right_enc = right_wheel_encoder.read();
  left_rot = countRotations(left_enc) - initial_left_rot;
  right_rot = countRotations(right_enc) - initial_right_rot;
  left_motor.setSpeed(left_motor_speed);
  right_motor.setSpeed(right_motor_speed);
*/

//TURN RIGHT
void turnRight() {
  leds(LOW);
  ledBlink(3, leftLED, 150);
  int targDistL =  300; //pulses
  int targDistR = - 300;
  resetPos();
  while (encoderLeftPos < targDistL && encoderRightPos > targDistR) {
    int calcPWML = computePID(encoderLeftPos, targDistL, KpL, KiL, KdL);
    int calcPWMR = computePID(encoderRightPos, targDistR, KpR, KiR, KdR);
    setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
    setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
    Serial.print(encoderLeftPos);
    Serial.print(" ");
    Serial.print(encoderRightPos);
    Serial.print(" ");
    Serial.print(calcPWML); 
    Serial.println();

  /*
  leds(LOW);
  ledBlink(3, rightLED, 150); //blink right LED 3x w/ 150 delay
  int targDistL = encoderLeftPos + 700; //half rotation
  int targDistR = - encoderRightPos - 700;
  while (targDistL >= encoderLeftPos) {
    int calcPWML = computePID(encoderLeftPos, targDistL, KpL, KiL, KdL);
    int calcPWMR = computePID(encoderRightPos, targDistR, KpR, KiR, KdR);
    setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
    setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
  }*/
}
}

/*
    float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
    float initial_right_enc = right_wheel_encoder.read();
    float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
    float initial_right_rot = countRotations(initial_right_enc);
    float left_rot = 0; // initialize the rotation counter
    float right_rot = 0;
    // perform right turn
    while ((right_rot <= setpoint_rot) or (left_rot >= (-1 * setpoint_rot))) {
    digitalWrite(rightLED, HIGH);
    float left_enc = left_wheel_encoder.read(); // read wheel encoder values
    float right_enc = right_wheel_encoder.read();
    right_rot = initial_right_rot - countRotations(right_enc);
    left_rot = initial_left_rot - countRotations(left_enc); // find the difference between the initial and current rotations
*/

//TURN LEFT
void turnLeft() {
  leds(LOW);
  ledBlink(3, leftLED, 150);
  int targDistL = - 300; //pulses
  int targDistR = 300;
  resetPos();
  while (encoderLeftPos > targDistL && encoderRightPos < targDistR) {
    int calcPWML = computePID(encoderLeftPos, targDistL, KpL, KiL, KdL);
    int calcPWMR = computePID(encoderRightPos, targDistR, KpR, KiR, KdR);
    setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
    setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
    Serial.print(encoderLeftPos);
    Serial.print(" ");
    Serial.print(encoderRightPos);
    Serial.print(" ");
    Serial.print(calcPWML); 
    Serial.println();
  }
  //Serial.println("here");
}
/*
  float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0; // initialize the rotation counter
  float right_rot = 0;
  // perform left turn
  while ((left_rot <= setpoint_rot) or (right_rot >= (-1 * setpoint_rot))) {
  digitalWrite(leftLED, HIGH);
  float left_enc = left_wheel_encoder.read(); // read wheel encoder values
  float right_enc = right_wheel_encoder.read();
  right_rot = initial_right_rot - countRotations(right_enc);
  left_rot = initial_left_rot - countRotations(left_enc); // find the difference between the initial and current rotations
  right_motor.setSpeed(50);
  left_motor.setSpeed(-50); // set turning speed (pwm)
  }
*/

void uTurn() {
  leds(LOW);
  ledBlink(3, leftLED, 150);
  int targDistL = encoderLeftPos - 1400; //full rotation
  int targDistR = encoderRightPos + 1400;
  while (targDistL <= encoderLeftPos) {
    int calcPWML = computePID(encoderLeftPos, targDistL, KpL, KiL, KdL);
    int calcPWMR = computePID(encoderRightPos, targDistR, KpR, KiR, KdR);
    setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
    setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
  }
}

void wallFollow() {
  if ((front_ir_state == 1) && (right_ir_state == 0)) {
    moveFwd();
  } else if ((front_ir_state == 0) && (right_ir_state == 0)) {
    turnLeft();
    delay(100);
  }
}

/*else if ((front_ir_state == 0) && (left_ir_state == 0) && (right_ir_state == 1)) {
   turnRight();
  } else if (front_ir_state == 0 && right_ir_state == 0 && left_ir_state == 0) {
   uTurn();
  }
  }
*/
