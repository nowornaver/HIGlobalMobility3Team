#include <MsTimer2.h>

#define TRIG_FRONT 11
#define ECHO_FRONT 12

#define TRIG_REAR 31
#define ECHO_REAR 30


// 속도
#define SPEED_MOTOR_FRONT_PWM  5
#define SPEED_MOTOR_FRONT_DIR  6
#define SPEED_MOTOR_FRONT_BRK  7

// 엔코더 (인터럽트 기반)
#define ENCODER_A_PIN 2     // 인터럽트 가능한 핀
#define ENCODER_B_PIN 4     // 방향 판별용

// 조향
#define STEERING_ANALOG_PIN A15
#define STEERING_MOTOR_PWM_PIN  8
#define STEERING_MOTOR_DIR_PIN  9
#define STEERING_MOTOR_BRK_PIN  10

double speed_angle_queue[2][2] = {{0.0, 0.0}, {0.0, 0.0}};

int toggle_count = 0;
const int ENCODER_COUNTS_PER_REV = 300;//
const double WHEEL_RADIUS = 0.135;
const double MY_PI = 3.14159265358979323846;

const int numReadings = 5;
static double readings[numReadings] = { 0, 0, 0, 0, 0 };
int readIndex = 0;
double total = 0;

double deltaT = 0.1;

double desiredSpeed_kph = 0.0;
double desiredSpeed_mps = 0.0;
double target_RPM = 0.0;
double Current_RPM = 0.0;
double totalOutput = 0.0;

const double Kp_speed = 3.25, Ki_speed = 0.8, Kd_speed = 0.06;//

double pidOutput = 0.0;
int motor_pwmValue = 0;
int previous_pos = 0;
double avg_rpm = 0.0;
int current_pos = 0;
double rpm = 0.0;

double esp = 0.1;
double fe_a = 1 / (2 * MY_PI);

double integral_error_RPM = 0.0;
double error_RPM = 0.0;
double filter = 0.0;
double filter_old = 0.0;
double derivative = 0.0;

const int angles[47] = {-24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
const int potValues[47] = {1018, 997, 976, 955, 934, 913, 892, 871, 850, 829, 808, 787, 766, 745, 724, 703, 682, 661, 640, 619, 598, 577, 556, 535, 514, 493, 472, 451, 430, 409, 388, 367, 346, 325, 304, 283, 262, 241, 220, 199, 178, 157, 136, 115, 94, 72, 50};

int currentPotValue = 0;
double currentAngle = 0.0;
double targetAngle = 0.0;
double steering_pwmValue = 0.0;
double integral_steering = 0.0;
double derivative_steering = 0.0;
const double Kp_steering = 6.7;
double Ki_steering = 1.1;
double Kd_steering = 0.7;
double previous_error_steering = 0.0;
const double integralLimit = 50.0;
double error_steering = 0;
int index = 0;

double angle1 = 0.0;
double angle2 = 0.0;
double value1 = 0.0;
double value2 = 0.0;

volatile long encoderCount = 0;

void encoderISR() {
  if (digitalRead(ENCODER_B_PIN)) encoderCount++;
  else encoderCount--;
}

void initEncoders() {
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  encoderCount = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);
}

void clearEncoderCount() {
  noInterrupts();
  encoderCount = 0;
  interrupts();
}

long readEncoder() {
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return count;
}

void setMotor(double dir, int pwmVal) {
  pwmVal = constrain(abs(pwmVal), 0, 255);
  digitalWrite(SPEED_MOTOR_FRONT_DIR, dir >= 0 ? HIGH : LOW);
  digitalWrite(SPEED_MOTOR_FRONT_BRK, pwmVal == 0 ? HIGH : LOW);
  analogWrite(SPEED_MOTOR_FRONT_PWM, pwmVal);
}

double calculateSpeedRPM() {
  current_pos = readEncoder();
  rpm = ((double)(current_pos - previous_pos) / ENCODER_COUNTS_PER_REV) / deltaT * 60.0;
  previous_pos = current_pos;

  total -= readings[readIndex];
  readings[readIndex] = rpm;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;

  avg_rpm = total / numReadings;
  return avg_rpm;
}

double computePID(double Current_RPM, double target_RPM, double kp, double ki, double kd) {
  error_RPM = target_RPM - Current_RPM;
  integral_error_RPM = integral_error_RPM - (esp * deltaT * integral_error_RPM) + (deltaT * error_RPM);
  filter_old = filter;
  filter += (deltaT / fe_a) * (error_RPM - filter);
  derivative = (filter - filter_old) / deltaT;
  pidOutput = kp * error_RPM + ki * integral_error_RPM + kd * derivative;
  return pidOutput;
}

double calculateDutyCycle(double totalOutput) {
  if (totalOutput > 127) return 127;
  else if (totalOutput < -127) return -127;
  else return totalOutput;
}

double getPotValueFromAngle(double angle) {
  index = findIndexForAngle(angle);
  if (index == -1) return -1;
  angle1 = angles[index]; angle2 = angles[index + 1];
  value1 = potValues[index]; value2 = potValues[index + 1];
  return linearInterpolate(angle, angle1, angle2, value1, value2);
}

double getAngleFromPotValue(int potValue) {
  index = findIndexForPotValue(potValue);
  if (index == -1) return -1;
  value1 = potValues[index]; value2 = potValues[index + 1];
  angle1 = angles[index]; angle2 = angles[index + 1];
  return linearInterpolate(potValue, value1, value2, angle1, angle2);
}

int findIndexForAngle(double angle) {
  for (int i = 0; i < sizeof(angles) / sizeof(int) - 1; i++) {
    if (angle >= angles[i] && angle <= angles[i + 1]) return i;
  }
  return -1;
}

int findIndexForPotValue(int potValue) {
  for (int i = 0; i < sizeof(potValues) / sizeof(int) - 1; i++) {
    if (potValue <= potValues[i] && potValue >= potValues[i + 1]) return i;
  }
  return -1;
}

double linearInterpolate(double x, double x0, double x1, double y0, double y1) {
  return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

void calculateSteeringControl(double currentAngle, double targetAngle) {
  error_steering = targetAngle - currentAngle;
  integral_steering = integral_steering - (esp * deltaT * integral_steering) + (deltaT * error_steering);
  integral_steering = constrain(integral_steering, -integralLimit, integralLimit);
  derivative_steering = (error_steering - previous_error_steering) / deltaT;
  steering_pwmValue = Kp_steering * error_steering + Ki_steering * integral_steering + Kd_steering * derivative_steering;
  previous_error_steering = error_steering;
  steering_pwmValue = constrain(steering_pwmValue, -255, 255);
}

void controlSteeringMotor(double currentAngle, double targetAngle) {
  const double threshold = 0.5;
  int pwm = 0;
  double diff = targetAngle - currentAngle;

  if (abs(diff) >= threshold) {
    pwm = constrain((int)(abs(diff) * 10), 50, 255);
    digitalWrite(STEERING_MOTOR_BRK_PIN, LOW);
    digitalWrite(STEERING_MOTOR_DIR_PIN, diff > 0 ? HIGH : LOW);
    analogWrite(STEERING_MOTOR_PWM_PIN, pwm);
  } else {
    analogWrite(STEERING_MOTOR_PWM_PIN, 0);
    digitalWrite(STEERING_MOTOR_BRK_PIN, HIGH);
  }
}

void Interrupt_10ms() {
  toggle_count += 1;
}

void setup() {
  Serial.begin(115200);

  startTime = millis();

  pinMode(SPEED_MOTOR_FRONT_PWM, OUTPUT);
  pinMode(SPEED_MOTOR_FRONT_DIR, OUTPUT);
  pinMode(SPEED_MOTOR_FRONT_BRK, OUTPUT);

  pinMode(STEERING_MOTOR_PWM_PIN, OUTPUT);
  pinMode(STEERING_MOTOR_DIR_PIN, OUTPUT);
  pinMode(STEERING_MOTOR_BRK_PIN, OUTPUT);
  pinMode(STEERING_ANALOG_PIN, INPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_REAR, OUTPUT);
  pinMode(ECHO_REAR, INPUT);


  initEncoders();
  clearEncoderCount();

  MsTimer2::set(10, Interrupt_10ms);
  MsTimer2::start();
}

void loop() {
  setMotor(totalOutput, motor_pwmValue);
  controlSteeringMotor(currentAngle, targetAngle);

long duration;
  float distance;

  // 초음파 신호 받기(RX)
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(30);
  digitalWrite(TRIG_FRONT, LOW);

  digitalWrite(TRIG_REAR, HIGH);
  delayMicroseconds(30);
  digitalWrite(TRIG_REAR, LOW);
  // 신호 보내기(TX)
  duration = pulseIn(ECHO_FRONT, HIGH);
  distance = duration * 0.0343 / 2;  // cm 단위
  
   duration = pulseIn(ECHO_REAR, HIGH);
  distance = duration * 0.0343 / 2;  // cm 단위

  // 시리얼 모니터 출력
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  desiredSpeed_mps = desiredSpeed_kph / 3.6;
  target_RPM = (desiredSpeed_mps * 60.0) / (2 * MY_PI * WHEEL_RADIUS);
  Current_RPM = calculateSpeedRPM();

  currentPotValue = analogRead(STEERING_ANALOG_PIN);
  currentAngle = getAngleFromPotValue(currentPotValue);

if (Serial.available() > 0) {
  String input = Serial.readStringUntil('\n');
  input.trim();  // 공백 제거

  if (input.equalsIgnoreCase("STOPPED")) {
    speed_angle_queue[0][0] = 0.0; // 속도 = 0
    target_RPM=0;
    // 조향각은 유지
  } 
  else if (input.indexOf(',') != -1) {
    int commaIndex = input.indexOf(',');
    speed_angle_queue[1][0] = speed_angle_queue[0][0];
    speed_angle_queue[1][1] = speed_angle_queue[0][1];
    speed_angle_queue[0][0] = input.substring(0, commaIndex).toFloat();
    speed_angle_queue[0][1] = input.substring(commaIndex + 1).toFloat();
    speed_angle_queue[0][0] = constrain(speed_angle_queue[0][0], -7.0, 7.0);
    speed_angle_queue[0][1] = constrain(speed_angle_queue[0][1], -18.0, 18.0);
  }

  while (Serial.available() > 0) Serial.read(); // 버퍼 비우기
}

  desiredSpeed_kph = speed_angle_queue[0][0];
  targetAngle = speed_angle_queue[0][1];

  totalOutput = computePID(Current_RPM, target_RPM, Kp_speed, Ki_speed, Kd_speed);
  calculateSteeringControl(currentAngle, targetAngle);
  motor_pwmValue = calculateDutyCycle(totalOutput);
  Serial.print("RPM_Target:"); Serial.print(target_RPM);
  Serial.print(",RPM_Current:"); Serial.print(Current_RPM);
  Serial.print(",Motor_PWM:"); Serial.print(motor_pwmValue);
  // Serial.print(",Pot_Target:"); Serial.print(targetPotValue);
  // Serial.print(",Pot_Current:"); Serial.print(currentPotValue);
  // Serial.print(",Steer_PWM:"); Serial.println(steering_pwmValue);
  do { delay(1); } while (toggle_count <= 9);
  toggle_count = 0;
}