void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // initialize servo arm and reset to home
  arm.attach(13);
  arm.write(180);
  // assign ir sensor pins
  pinMode(front_ir, INPUT);
  pinMode(bot_ir, INPUT);
  pinMode(left_diagonal_ir, INPUT);
  pinMode(right_diagonal_ir, INPUT);
  pinMode(left_ir, INPUT);
  pinMode(right_ir, INPUT);
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);
  // assign encoder pins
  pinMode(L_ENCODER_A, INPUT);
  pinMode(L_ENCODER_B, INPUT);
  pinMode(R_ENCODER_A, INPUT);
  pinMode(R_ENCODER_B, INPUT);
  // create isr for each
  attachInterrupt(digitalPinToInterrupt(front_ir), front_state_isr, CHANGE); // initialize interrupt pin
  attachInterrupt(digitalPinToInterrupt(bot_ir), bot_state_isr, CHANGE); // initialize interrupt pin
  attachInterrupt(digitalPinToInterrupt(left_diagonal_ir), left_diagonal_state_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_diagonal_ir), right_diagonal_state_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_ir), left_state_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_ir), right_state_isr, CHANGE);
  // interrupt for channel A triggered by rising edge calls readEncoder function to check state of Channel B to determine direction
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), readEncoderRight, RISING);
  //
setMotor(0, L_ENABLE, L_MOT_DIR);
setMotor(0, R_ENABLE, R_MOT_DIR);
delay(2000);
}

void loop() {
currentMillis = millis();
wallFollow();
//Serial.print(encoderLeftPos);
//Serial.print(" ");
//Serial.print(300);
//Serial.print(" ");
//Serial.println(encoderRightPos);
}

/*int targDistL = - 700; //pulses
  int targDistR = 700;
int calcPWML = computePID(encoderLeftPos, targDistL, KpL, KiL, KdL);
    int calcPWMR = computePID(encoderRightPos, targDistR, KpR, KiR, KdR);
    setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
    setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
*/
//wallFollow();
/*
int targDistL = - 300; //pulses
int targDistR = 300;
int calcPWML = computePID(encoderLeftPos, targDistL, KpL, KiL, KdL);
setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
int calcPWMR = computePID(encoderRightPos, targDistR, KpR, KiR, KdR);
setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
*/
//Serial.print(-300);
//Serial.print(" ");
//Serial.print(encoderLeftPos);
//Serial.print(" ");
//Serial.print(300);
//Serial.print(" ");
//Serial.print(encoderRightPos);
//Serial.println();
  


  /*
  int lTarget = 1485;
  int rTarget = -1485;
  int calcPWML = computePID(encoderLeftPos, lTarget, KpL, KdL, KiL);
  int calcPWMR = computePID(encoderRightPos, rTarget, KpR, KdR, KiR);
  setMotor(calcPWML, L_ENABLE, L_MOT_DIR);
  setMotor(calcPWMR, R_ENABLE, R_MOT_DIR);
  */
  
  /*
  Serial.print(lTarget);
  Serial.print(" ");
  Serial.print(rTarget);
  Serial.print(" ");
  Serial.print(encoderLeftPos);
  Serial.print(" ");
  Serial.print(encoderRightPos);
  Serial.println();
  */

//void computeLeftPID(int target, float kP, float kI, float kD) {
/*
  long currT = micros();
  int target = 5000;
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  while (encoderLeftPos < target) {
    // error calculation
    float eL = target - encoderLeftPos;

    // derivative calculation
    float dedtL = (eL - eprevL) / (deltaT);

    // integral calculation
    eIntegralL = eIntegralL + eL * deltaT;

    // control signal for PID
    float out  = KpL * eL + KdL * dedtL + KiL * eIntegralL;
    int pwmL = fabs(out);
    if (pwmL > 60) {
      pwmL = 60;
    }

    int dir = 1;
    if (out < 0) {
      dir = 0;
    }

    setMotor(pwmL, L_ENABLE, L_MOT_DIR);
    eprevL = eL; // retain error for next loop
*/




/*
  int target = 2970;

  // time difference
  long currT = micros();
  long eT = currT-prevT;
  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;

  // error
  int eL = target - encoderLeftPos;

  // derivative
  float dedtL = (eL-eprevL)/(deltaT);

  // integral
  eIntegralL = eIntegralL + eL*deltaT;

  // control signal
  float u = ((KpL*eL) + (KdL*dedtL) + (KiL*eIntegralL));

  // motor power
  float pwr = fabs(u);
  if (pwr>255){
  pwr = 255;
  }

  int dir = 1;
  if (u<0) {
  dir = -1;
  }

  eprev = eL;


  // store previous error

  //setMotor(1, pwr, L_ENABLE, L_MOT_DIR);
  //setMotor(1, pwr, R_ENABLE, R_MOT_DIR);
*/
