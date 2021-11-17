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

void correctSpeed() {
  if (right_diagonal_ir_state == 0) {
    right_motor_corr += 1;
  } else if (left_diagonal_ir_state == 0) {
    left_motor_corr += 2;
  } else {
    left_motor_corr = 0;
    right_motor_corr = 0;
  }
  left_motor_speed = motor_base_speed + left_motor_corr + 1;//constrain((motor_base_speed + left_motor_corr + 3), motor_base_speed, 90);
  right_motor_speed = motor_base_speed + right_motor_corr;//constrain((motor_base_speed + right_motor_corr), motor_base_speed, 90);;
}

void moveFwd() {
  correctSpeed();
  left_motor.setSpeed(left_motor_speed);
  right_motor.setSpeed(-right_motor_speed);
}
