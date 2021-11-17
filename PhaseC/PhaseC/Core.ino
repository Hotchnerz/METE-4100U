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
  //delay(10000);
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
  moveFwd();
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
