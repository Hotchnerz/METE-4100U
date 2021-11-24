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

void lightsOff() {
  digitalWrite(leftLED, LOW);
  digitalWrite(rightLED, LOW);
}

void ledBlink(int count, int dirLED) {
  while (count > 0) {
    digitalWrite(dirLED, HIGH);
    delay(150);
    digitalWrite(dirLED, LOW);
    delay(150);
    count -= 1;
  }
}

void correctSpeed() {
  if (right_diagonal_ir_state == 0) {
    right_motor_corr += 0.5;
  } else if (left_diagonal_ir_state == 0) {
    left_motor_corr += 1.5;
  }
  else {
    left_motor_corr = 0;
    right_motor_corr = 0;
  }
  left_motor_speed = motor_base_speed + left_motor_corr + 1;//constrain((motor_base_speed + left_motor_corr + 3), motor_base_speed, 90);
  right_motor_speed = motor_base_speed + right_motor_corr;//constrain((motor_base_speed + right_motor_corr), motor_base_speed, 90);;

}

void moveFwd() {
  lightsOff();
  digitalWrite(leftLED, HIGH);
  digitalWrite(rightLED, HIGH);
  correctSpeed();
  left_motor.setSpeed(left_motor_speed);
  right_motor.setSpeed(right_motor_speed);

}

void moveFwdABit(int setpoint_rot) {
  delay(300);
  float initial_left_enc = left_wheel_encoder.read(); // read and store initial encoder value
  float initial_right_enc = right_wheel_encoder.read();
  float initial_left_rot = countRotations(initial_left_enc); // convert the initial encoder value into rotation
  float initial_right_rot = countRotations(initial_right_enc);
  float left_rot = 0; // initialize the rotation counter
  float right_rot = 0;

  while ((right_rot >= (-1 * setpoint_rot)) or (left_rot >= (-1 * setpoint_rot))) {
    digitalWrite(rightLED, HIGH);
    digitalWrite(leftLED, HIGH);
    float left_enc = left_wheel_encoder.read(); // read wheel encoder values
    float right_enc = right_wheel_encoder.read();
    right_rot = initial_right_rot - countRotations(right_enc);
    left_rot = initial_left_rot - countRotations(left_enc); // find the difference between the initial and current rotations
    right_motor.setSpeed(50);
    left_motor.setSpeed(50); // set turning speed (pwm)
    Serial.print(right_rot);
    Serial.print(" | ");
    Serial.println(left_rot);

  }
  right_motor.setSpeed(0);
  left_motor.setSpeed(0);
}

float countRotations(float encoder_position) {
  float rotations = encoder_position / 5800;
  return rotations;
}

// 90 degree right turn function
void turnRight() {
  moveFwdABit(0.45);
  right_motor.setSpeed(0);
  left_motor.setSpeed(0);
  lightsOff();
  ledBlink(2, rightLED);

  const float setpoint_rot = 0.390; // amount of wheel rotations
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
    right_motor.setSpeed(-50);
    left_motor.setSpeed(50); // set turning speed (pwm)
  }
  left_motor.setSpeed(0); // set turning speed (pwm)
  right_motor.setSpeed(0);
  delay(250);
  lightsOff();
  moveFwdABit(0.69);
}

// left turn function
void turnLeft() {
  moveFwdABit(0.45);
  right_motor.setSpeed(0);
  left_motor.setSpeed(0);
  lightsOff();
  ledBlink(3, leftLED);
  const float setpoint_rot = 0.390; // amount of wheel rotations
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
    left_motor.setSpeed(-50);
    right_motor.setSpeed(50); // set turning speed (pwm)
  }
  left_motor.setSpeed(0); // set turning speed (pwm)
  right_motor.setSpeed(0);
  delay(250);
  lightsOff();
  moveFwdABit(0.5);
}

void wallFollow() {
  /*if ((front_ir_state == 1) && (bot_ir_state == 0) && (miner == false)) {
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
    }*/
  /*else if ((front_ir_state == 1) && (right_ir_state == 0) && (left_ir_state == 1)) {
    //left_turn_special
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    } */ if ((front_ir_state == 1) && (right_ir_state == 0)) {
    // forward moving
    moveFwd();
  } else if (right_ir_state == 1) {
    // turn right
    turnRight();
  }

  else if ((front_ir_state == 0) && (right_ir_state == 0) && (left_ir_state == 1)) {
      // turn left
      turnLeft();
  }
    /*} else if ((front_ir_state == 0) && (right_ir_state == 0) && (left_ir_state == 0)) {
      // uTurn
      moveFwdABit();
      moveFwdABit();
      uTurn();
    }*/
}
