#include <MsTimer2.h>

// 속도 모터 핀 설정
#define SPEED_MOTOR_FRONT_PWM  5
#define SPEED_MOTOR_FRONT_DIR  6
#define SPEED_MOTOR_FRONT_BRK  7

// 엔코더 핀 설정
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 4

// 조향 모터 핀 설정
#define STEERING_ANALOG_PIN A15
#define STEERING_MOTOR_PWM_PIN  8
#define STEERING_MOTOR_DIR_PIN  9
#define STEERING_MOTOR_BRK_PIN  10

// 타이머
int toggle_count = 0;
void Interrupt_10ms() { toggle_count++; }
volatile char rxData;
volatile bool newDataFlag = false;
volatile bool txReady = true;
volatile char txData;
// 구동 PID 변수
double deltaT = 0.1;
const int ENCODER_COUNTS_PER_REV = 300;
const double WHEEL_RADIUS = 0.135;
const double MY_PI = 3.14159265358979323846;

double desiredSpeed_kph = 0.0;
double desiredSpeed_mps = 0.0;
double target_RPM = 0.0;
double Current_RPM = 0.0;
double totalOutput = 0.0;
int motor_pwmValue = 0;

const double Kp_speed = 3.25, Ki_speed = 0.8, Kd_speed = 0.06;
double integral_error_RPM = 0.0, error_RPM = 0.0;
double filter = 0.0, filter_old = 0.0, derivative = 0.0;
double esp = 0.1;
double fe_a = 1 / (2 * MY_PI);

// 엔코더
volatile long encoderCount = 0;
int previous_pos = 0, current_pos = 0;
double rpm = 0.0;
const int numReadings = 5;
double readings[numReadings] = {0}, total = 0;
int readIndex = 0;
int steeringAngle = 0;  // 조향각 상태 변수 (예: -16 ~ 21 제한)
//char steeringAngle = 0;  // GPS 에서 받은 조향각

int speed = 0;
// 조향 PID 변수 (Pot 기반)
int currentPotValue = 0, targetPotValue = 0;
double steering_pwmValue = 0.0;
const double Kp_steering = 0.75, Ki_steering = 0.2, Kd_steering = 0.1;
double integral_steering = 0.0, derivative_steering = 0.0;
double previous_error_steering = 0.0;
const double integralLimit = 50.0;

// 시리얼 입력값
double speed_angle_queue[2][2] = {{0.0, 0.0}, {0.0, 0.0}};


// UART1 RX 인터럽트
ISR(USART1_RX_vect) {
  rxData = UDR1;       // 수신 데이터 읽기
  newDataFlag = true;  // 수신 플래그 설정
      Serial.println("RX ISR");

}

// UART1 TX 데이터 레지스터 빈 상태 인터럽트
ISR(USART1_UDRE_vect) {
  if (!txReady) {
    UDR1 = txData;    // 데이터 전송
    txReady = true;   // 전송 완료
    UCSR1B &= ~(1 << UDRIE1); // 인터럽트 비활성화
          Serial.println("TX ISR");

  }
}

// 한 바이트 송신
void sendByte(char data) {
  while (!txReady);   // 이전 전송 완료 대기
  txData = data;
  txReady = false;
  UCSR1B |= (1 << UDRIE1); // TX 인터럽트 활성화
//  Serial.println(txData);
}

void UART1_init(unsigned long baud) {
  uint16_t ubrr = (F_CPU / 16 / baud) - 1;
  UBRR1H = (ubrr >> 8);
  UBRR1L = ubrr;
  
  UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1); // RX, TX, RX 인터럽트
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 데이터 8비트, 패리티 없음, 1 스톱비트
}
// --- 선형 보간: 목표 각도 → 목표 Pot 값
int getPotFromAngle(double targetAngle) {
  const double angle0 = -17.0;
  const double angle1 = 22.0;
  const int pot0 = 10;
  const int pot1 = 1018;
  return pot0 + (targetAngle - angle0) * (float)(pot1 - pot0) / (angle1 - angle0);
}

// --- 조향 PID (Pot 값 기반)
void calculateSteeringControl_Pot(int currentPot, int targetPot) {
  double error = (double)(targetPot - currentPot);
  integral_steering = constrain(integral_steering + (error * deltaT), -integralLimit, integralLimit);
  derivative_steering = (error - previous_error_steering) / deltaT;
  steering_pwmValue = Kp_steering * error + Ki_steering * integral_steering + Kd_steering * derivative_steering;
  previous_error_steering = error;
  steering_pwmValue = constrain(steering_pwmValue, -255, 255);
}

// --- 조향 모터 제어
void controlSteeringMotor(double pwm) {
  if (abs(pwm) < 5) {
    analogWrite(STEERING_MOTOR_PWM_PIN, 0);
    digitalWrite(STEERING_MOTOR_BRK_PIN, HIGH);
    return;
  }
  digitalWrite(STEERING_MOTOR_DIR_PIN, pwm > 0 ? HIGH : LOW);
  digitalWrite(STEERING_MOTOR_BRK_PIN, LOW);
  analogWrite(STEERING_MOTOR_PWM_PIN, abs((int)pwm));
}

// --- 엔코더 관련
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

long readEncoder() {
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return count;
}

void clearEncoderCount() {
  noInterrupts();
  encoderCount = 0;
  interrupts();
}

// --- 구동 모터 제어
void setMotor(double dir, int pwmVal) {
  pwmVal = constrain(abs(pwmVal), 0, 255);
  digitalWrite(SPEED_MOTOR_FRONT_DIR, dir >= 0 ? LOW : HIGH);
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

  return total / numReadings;
}

double computePID(double Current_RPM, double target_RPM, double kp, double ki, double kd) {
  error_RPM = target_RPM - Current_RPM;
  integral_error_RPM = integral_error_RPM - (esp * deltaT * integral_error_RPM) + (deltaT * error_RPM);
  filter_old = filter;
  filter += (deltaT / fe_a) * (error_RPM - filter);
  derivative = (filter - filter_old) / deltaT;

  double output = kp * error_RPM + ki * integral_error_RPM + kd * derivative;
  return constrain(output, -127, 127);  // 안정화
}

int calculateDutyCycle(double output) {
  return constrain((int)output, -127, 127);
}

// --- Setup
unsigned long startTime = 0;

void setup() {
  Serial.begin(9600); // USB 시리얼 (디버깅용)
  UART1_init(9600);   // UART1 초기화
  pinMode(SPEED_MOTOR_FRONT_PWM, OUTPUT);
  pinMode(SPEED_MOTOR_FRONT_DIR, OUTPUT);
  pinMode(SPEED_MOTOR_FRONT_BRK, OUTPUT);

  pinMode(STEERING_MOTOR_PWM_PIN, OUTPUT);
  pinMode(STEERING_MOTOR_DIR_PIN, OUTPUT);
  pinMode(STEERING_MOTOR_BRK_PIN, OUTPUT);

  pinMode(STEERING_ANALOG_PIN, INPUT);
  initEncoders();
  clearEncoderCount();

  previous_pos = readEncoder();  // ✅ 초기값 설정

  MsTimer2::set(10, Interrupt_10ms);
  MsTimer2::start();

  startTime = millis();  // ✅ 초기 2초 무시
}

// --- Main Loop

String receivedState = "";
String receivedAngleStr = "";
double receivedAngle = 0.0;
void loop() {
  currentPotValue = analogRead(STEERING_ANALOG_PIN);

int angle = (int)steeringAngle; 


if (newDataFlag) {
  newDataFlag = false;
  
    switch (rxData) {
      case 'w':  // 전진
        speed = 0.5;  
        // 조향각은 유지
        break;

      case 's':  // 후진
        speed = -0.5;
        // 조향각 유지
        break;

      case 'a':  // 좌회전
        steeringAngle -= 5;
        if (steeringAngle < -16) steeringAngle = -16;
        // 속도 유지
        break;

      case 'd':  // 우회전
        steeringAngle += 5;
        if (steeringAngle > 21) steeringAngle = 21;
        // 속도 유지
        break;

      case 'x':  // 정지 명령 (예)
        speed = 0.0;
        steeringAngle = 0;
        break;

      default:
        // 기타 입력 무시
        break;
    }
if (angle >20 || angle <-15) {
  angle = 0;
}
   speed_angle_queue[0][0] = speed;
    speed_angle_queue[0][1] = steeringAngle; //angle 로 바꾸면 GPS 에서 수신
    speed_angle_queue[1][0] = speed;
    speed_angle_queue[1][1] = steeringAngle; //angle

   sendByte(rxData); // 받은 데이터 그대로 송신 (에코)

}


  // 초기 2초간 PID 동작 무시 (안정화 시간)
  if (millis() - startTime < 2000) {
    setMotor(0, 0);
    clearEncoderCount();
    previous_pos = readEncoder();
    return;
  }

  desiredSpeed_kph = speed_angle_queue[0][0];
  double targetAngle = speed_angle_queue[0][1];

  if (desiredSpeed_kph == 0.0) {
    setMotor(0, 0);  // 정지
    targetAngle = 1.0;
  } else {
    desiredSpeed_mps = desiredSpeed_kph / 3.6;
    target_RPM = (desiredSpeed_mps * 60.0) / (2 * MY_PI * WHEEL_RADIUS);
    Current_RPM = calculateSpeedRPM();
    totalOutput = computePID(Current_RPM, target_RPM, Kp_speed, Ki_speed, Kd_speed);
    motor_pwmValue = calculateDutyCycle(totalOutput);
    setMotor(totalOutput, motor_pwmValue);
  }

  targetPotValue = getPotFromAngle(targetAngle);
  calculateSteeringControl_Pot(currentPotValue, targetPotValue);
  controlSteeringMotor(steering_pwmValue);
//
//  Serial.print("RPM_Target:"); Serial.print(target_RPM);
//  Serial.print(",RPM_Current:"); Serial.print(Current_RPM);
//  Serial.print(",Motor_PWM:"); Serial.print(motor_pwmValue);
//  Serial.print(",Pot_Target:"); Serial.print(targetPotValue);
//  Serial.print(",Pot_Current:"); Serial.print(currentPotValue);
//  Serial.print(",Steer_PWM:"); Serial.println(steering_pwmValue);

  do { delay(1); } while (toggle_count <= 9);
  toggle_count = 0;
}
