// ================== Includes ==================
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <MsTimer2.h>
#include <stdint.h>

// ================== 핀 설정 ==================
// 속도 모터 핀
#define SPEED_MOTOR_FRONT_PWM  5
#define SPEED_MOTOR_FRONT_DIR  6
#define SPEED_MOTOR_FRONT_BRK  7
// 엔코더 핀
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 4
// 조향 모터/피드백 핀
#define STEERING_ANALOG_PIN      A15
#define STEERING_MOTOR_PWM_PIN   8
#define STEERING_MOTOR_DIR_PIN   9
#define STEERING_MOTOR_BRK_PIN   10
// 초음파 핀
#define TRIG_FRONT 11
#define ECHO_FRONT 12
#define TRIG_REAR  31   // 오른쪽 센서로 사용 (핀 이름만 REAR)
#define ECHO_REAR  30

// ================== 큐 & 모드 ==================
QueueHandle_t uartQueue;
QueueHandle_t manualQueue;
QueueHandle_t gpsQueue;
QueueHandle_t ultrasonicQueue;    // (기존 CommTask에서 U모드에 쓰던 raw 바이트 큐: 유지)
QueueHandle_t cameraQueue;
QueueHandle_t controlQueue;       // ControlTask가 소비(ManualCommand)
QueueHandle_t ultrasonicCmdQueue; // ★ 초음파 알고리즘 결과(UltrasonicCommand)

// 운전 모드
enum ControlMode { MODE_MANUAL, MODE_GPS, MODE_Ultrasonic, MODE_Camera };
volatile ControlMode currentMode = MODE_MANUAL;

// ================== 커맨드 구조체 ==================
typedef struct {
  int speed1;   // -1: 뒤로, 0: 정지, 1: 앞으로 (kph로도 사용됨)
  int angle;    // 조향각 (-26 ~ +26)
} ManualCommand;

typedef struct {
  int angle;
  int speed1;
} GPSCommand;

typedef struct {
  int angle;
  int speed1;   // 정수 스케일(여기서는 0/1 사용)
} UltrasonicCommand;

typedef struct {
  int speed1;
} CameraCommand;

// ================== UART1 (하드웨어) ==================
volatile bool txReady = true;
volatile char txData;

ISR(USART1_RX_vect) {
  uint8_t data = UDR1;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(uartQueue, &data, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

ISR(USART1_UDRE_vect) {
  if (!txReady) {
    UDR1 = txData;
    txReady = true;
    UCSR1B &= ~(1 << UDRIE1);
  }
}

void sendByte(char data) {
  while (!txReady);
  txData = data;
  txReady = false;
  UCSR1B |= (1 << UDRIE1);
}

void UART1_init(unsigned long baud) {
  uint16_t ubrr = (F_CPU / 16 / baud) - 1;
  UBRR1H = (ubrr >> 8);
  UBRR1L = ubrr;
  UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1); // RX, TX, RX 인터럽트
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);               // 8N1
}

// ================== 주행/조향 제어 변수 ==================
double deltaT = 0.1;
const int ENCODER_COUNTS_PER_REV = 300;
const double WHEEL_RADIUS = 0.135;
const double MY_PI = 3.14159265358979323846;

// 속도 PID
double desiredSpeed_kph = 0.0;
double desiredSpeed_mps = 0.0;
double target_RPM = 0.0;
double Current_RPM = 0.0;
double totalOutput = 0.0;
int    motor_pwmValue = 0;

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

// 조향 PID (Pot)
int currentPotValue = 0, targetPotValue = 0;
double steering_pwmValue = 0.0;
const double Kp_steering = 0.75, Ki_steering = 0.2, Kd_steering = 0.1;
double integral_steering = 0.0, derivative_steering = 0.0;
double previous_error_steering = 0.0;
const double integralLimit = 50.0;

// 수신 커맨드 버퍼(기존 로직 유지)
volatile double speed_angle_queue[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
int targetAngle = 0;

// ================== 초음파 제어 상수 (두 번째 코드 통합) ==================
// ★ 부동소수 기반으로 계산, ControlTask 전달 시 정수화
const float MAX_ANGLE_F   = 20.0f;   // 최댓 조향 [deg]
const float D_CURB        = 100.0f;  // 우측(연석) 목표거리 [cm]
const float D_OBS         = 120.0f;  // 전방(장애물) 목표거리 [cm]
const float OBS_ENTER     = 160.0f;  // 가까우면 가중치↑
const float OBS_EXIT      = 200.0f;  // 멀어지면 가중치↓
const float KP_CURB       = 0.22f;
const float KD_CURB       = 0.00f;
const float KP_OBS        = 0.25f;
const float KD_OBS        = 0.00f;
const float ANG_SLEW_F    = 4.0f;    // 프레임당 최대 각 변화량 [deg]
const float MIN_VALID_CM  = 20.0f;
const float MAX_VALID_CM  = 600.0f;
const float SR_DEADBAND_MIN = 100.0f;
const float SR_DEADBAND_MAX = 150.0f;
const float FRONT_STOP_DIST = 100.0f;
const float VEL_CRUISE      = 1.0f;  // 속도 명령 기준치(상대 단위)

// ★ 초음파 제어 내부 상태
static float wObs = 0.0f;                 // 장애물 가중치 0..1
static float prevAng = 0.0f;
static float prevErrCurb = 0.0f, prevErrObs = 0.0f;
static unsigned long prevT_us = 0;
unsigned long bootMs = 0;

// ================== 유틸 함수 ==================
static inline float clampf(float x, float a, float b){
  return x < a ? a : (x > b ? b : x);
}
static inline float slew(float prev, float tgt, float step){
  float d = tgt - prev;
  if (d >  step) d =  step;
  if (d < -step) d = -step;
  return prev + d;
}

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(30);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // timeout 30ms
  if (duration == 0) return -1;
  float distance = duration * 0.0343f / 2.0f;
  return roundf(distance / 10.0f) * 10.0f;        // 10단위 반올림 (원본 유지)
}

void updateObsWeight(float fc){
  float tgt;
  if (fc < 0) {
    tgt = wObs; // 무효면 유지
  } else if (fc < OBS_ENTER) {
    float x = (OBS_ENTER - fc) / OBS_ENTER; // 가까울수록 0→1
    tgt = clampf(x, 0.0f, 1.0f);
  } else if (fc > OBS_EXIT) {
    tgt = 0.0f;
  } else {
    tgt = wObs;
  }
  wObs = slew(wObs, tgt, 0.10f);
}

// ================== 하드웨어/제어 함수 ==================
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
  noInterrupts(); long count = encoderCount; interrupts(); return count;
}
void clearEncoderCount() { noInterrupts(); encoderCount = 0; interrupts(); }

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
  return constrain(output, -127, 127);
}
int calculateDutyCycle(double output) { return constrain((int)output, -127, 127); }

// Pot 기반 각도 매핑
int getPotFromAngle(int targetAngle) {
  const double angle0 = -26.0, angle1 = 26.0;
  const int pot0 = 10, pot1 = 1018;
  return pot0 + (int)(((float)(targetAngle - angle0)) * (float)(pot1 - pot0) / (angle1 - angle0));
}
void calculateSteeringControl_Pot(int currentPot, int targetPot) {
  double error = (double)(targetPot - currentPot);
  integral_steering = constrain(integral_steering + (error * deltaT), -integralLimit, integralLimit);
  derivative_steering = (error - previous_error_steering) / deltaT;
  steering_pwmValue = Kp_steering * error + Ki_steering * integral_steering + Kd_steering * derivative_steering;
  previous_error_steering = error;
  steering_pwmValue = constrain(steering_pwmValue, -255, 255);
}
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

// ================== 고수준 핸들러 ==================
void handleManualControl(char cmd) {
  static int speed1 = 0;
  static int steeringAngle = 0;
  ManualCommand manualCmd;

  switch (cmd) {
    case 'w': speed1 = 1; break;
    case 's': speed1 = -1; break;
    case 'a': steeringAngle = max(-26, steeringAngle - 5); break;
    case 'd': steeringAngle = min(26,  steeringAngle + 5); break;
    case 'x': speed1 = 0; steeringAngle = 0; break;
  }
  manualCmd.speed1 = speed1;
  manualCmd.angle  = steeringAngle;

  if (xQueueSend(controlQueue, &manualCmd, 10 / portTICK_PERIOD_MS) != pdPASS) {
    Serial.println("Warning: controlQueue full, manual command lost!");
  }
}

// ================== 태스크 선언 ==================
void CommTask(void *pvParameters);
void ManualTask(void *pvParameters);
void GPSTask(void *pvParameters);
void CAMERATask(void *pvParameters);
void ControlTask(void *pvParameters);
void UltrasonicTask(void *pvParameters);  // ★ 새로 추가: 두 번째 코드 알고리즘 담당

// ================== CommTask ==================
void CommTask(void *pvParameters) {
  Serial.println("CommTask running");
  uint8_t rxData;
  bool gpsModeActive = false;
  static char    gpsBuffer[8];
  static uint8_t gpsIndex = 0;

  for (;;) {
    if (xQueueReceive(uartQueue, &rxData, portMAX_DELAY) == pdTRUE) {
      switch (rxData) {
        case 'K': currentMode = MODE_MANUAL;     Serial.println("Mode: MANUAL");     gpsModeActive=false; continue;
        case 'G': currentMode = MODE_GPS;        Serial.println("Mode: GPS");        gpsModeActive=true;  continue;
        case 'U': currentMode = MODE_Ultrasonic; Serial.println("Mode: ULTRASONIC"); gpsModeActive=false; continue;
        case 'C': currentMode = MODE_Camera;     Serial.println("Mode: CAMERA");     gpsModeActive=false; continue;
      }

      switch (currentMode) {
        case MODE_MANUAL:
          xQueueSend(manualQueue, &rxData, 20);
          break;
        case MODE_GPS:
          if (gpsModeActive) {
            if (rxData == '\r' || rxData == '\n') {
              gpsBuffer[gpsIndex] = '\0';
              GPSCommand gpsCmd; gpsCmd.angle = atoi(gpsBuffer); gpsCmd.speed1 = 0;
              Serial.print("[GPS MODE] Parsed angle: "); Serial.println(gpsCmd.angle);
              xQueueSend(gpsQueue, &gpsCmd, 20);
              gpsIndex = 0;
            } else {
              if (gpsIndex < sizeof(gpsBuffer) - 1) gpsBuffer[gpsIndex++] = rxData;
            }
          }
          break;
        case MODE_Ultrasonic:
          // 필요 시 U모드에서 추가 바이트 처리하면 ultrasonicQueue 사용
          xQueueSend(ultrasonicQueue, &rxData, 20);
          break;
        case MODE_Camera:
          if (rxData >= '0' && rxData <= '9') {
            int val = rxData - '0';
            xQueueSend(cameraQueue, &val, 20);
          }
          break;
      }
    }
  }
}

// ================== ManualTask ==================
void ManualTask(void *pvParameters) {
  char rxData;
  for (;;) {
    if (xQueueReceive(manualQueue, &rxData, portMAX_DELAY) == pdTRUE) {
      handleManualControl(rxData);
    }
  }
}

// ================== GPSTask ==================
void GPSTask(void *pvParameters) {
  GPSCommand gpsCmd;
  ManualCommand manualCmd;
  for (;;) {
    if (xQueueReceive(gpsQueue, &gpsCmd, portMAX_DELAY) == pdTRUE) {
      Serial.print("[GPS MODE] Received angle: "); Serial.println(gpsCmd.angle);
      manualCmd.speed1 = 0;               // 필요 시 조정
      manualCmd.angle  = gpsCmd.angle;
      if (xQueueSend(controlQueue, &manualCmd, 10 / portTICK_PERIOD_MS) != pdPASS) {
        Serial.println("Warning: controlQueue full, GPS command lost!");
      }
    }
  }
}

// ================== CAMERATask ==================
void CAMERATask(void *pvParameters) {
  Serial.println("Camera Task Running");
  int cameraSpeed;
  ManualCommand manualCmd;
  for (;;) {
    if (xQueueReceive(cameraQueue, &cameraSpeed, portMAX_DELAY) == pdTRUE) {
      Serial.println(cameraSpeed);
      manualCmd.speed1 = cameraSpeed;
      manualCmd.angle  = 0;
      if (xQueueSend(controlQueue, &manualCmd, 10 / portTICK_PERIOD_MS) != pdPASS) {
        Serial.println("Warning: controlQueue full, Camera command lost!");
      }
    }
  }
}

// ================== ControlTask (모터/조향) ==================
void ControlTask(void *pvParameters) {
  Serial.println("CONTROLTASK running");
  ManualCommand cmd = {0, 0};

  for (;;) {
    // 최신 명령 있으면 갱신
    if (xQueueReceive(controlQueue, &cmd, 10 / portTICK_PERIOD_MS) == pdTRUE) {
      if (cmd.angle > 25 || cmd.angle < -25) cmd.angle = 0; // 보호
      speed_angle_queue[0][0] = cmd.speed1; // kph 취급
      speed_angle_queue[0][1] = cmd.angle;
    }
    // 내부 상태 복제 (이전 프레임)
    speed_angle_queue[1][0] = cmd.speed1;
    speed_angle_queue[1][1] = cmd.angle;

    // 속도 제어
    desiredSpeed_kph = speed_angle_queue[0][0];
    targetAngle      = (int)speed_angle_queue[0][1];
    Serial.println(cmd.speed1);

    currentPotValue = analogRead(STEERING_ANALOG_PIN);

    if (desiredSpeed_kph == 0.0) {
      setMotor(0, 0);
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

    vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz
  }
}

// ================== UltrasonicTask (★ 두 번째 코드 통합부) ==================
void UltrasonicTask(void *pvParameters) {
  // CSV 헤더(원하시면 주석 처리)
  Serial.println("Time(ms),Front(cm),Right(cm),wObs,angCurb(deg),angObs(deg),cmd(deg),vel");

  prevT_us = micros();
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(100); // 10Hz

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    // 모드가 ULTRASONIC일 때만 계산/전송
    if (currentMode != MODE_Ultrasonic) continue;

    // 1) 센서 읽기
    float front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    vTaskDelay(pdMS_TO_TICKS(20)); // 간섭 방지
    float right = readUltrasonic(TRIG_REAR,  ECHO_REAR);

    // 2) 유효범위 필터
    if (front < MIN_VALID_CM || front > MAX_VALID_CM) front = -1;
    if (right < MIN_VALID_CM || right > MAX_VALID_CM) right = -1;
    if (front < 0 || right < 0) continue; // 이번 프레임 스킵

    // 3) dt
    unsigned long now_us = micros();
    float dt = (now_us - prevT_us) * 1e-6f;
    if (dt < 1e-3f) dt = 1e-3f;
    prevT_us = now_us;

    // 4) 장애물 가중치
    updateObsWeight(front);

    // 5) 각 성분
    float angCurb = 0.0f;
    if (right >= SR_DEADBAND_MIN && right <= SR_DEADBAND_MAX) {
      angCurb = 0.0f; prevErrCurb = 0.0f;
    } else {
      float eCurb = right - D_CURB;
      float dCurb = (eCurb - prevErrCurb) / dt;
      angCurb = (KP_CURB * eCurb + KD_CURB * dCurb);
      prevErrCurb = eCurb;
    }

    float eObs = front - D_OBS;
    float dObs = (eObs - prevErrObs) / dt;
    float angObs = (KP_OBS * eObs + KD_OBS * dObs);
    prevErrObs = eObs;

    // 6) 블렌딩 → 제한 → 슬루 → 반올림
    float angTgt   = (1.0f - wObs) * angCurb + wObs * angObs;
    angTgt         = clampf(angTgt, -MAX_ANGLE_F, +MAX_ANGLE_F);
    float steerCmd = slew(prevAng, angTgt, ANG_SLEW_F);
    steerCmd       = roundf(steerCmd);
    prevAng        = steerCmd;

    // 7) 속도
    float velCmd = VEL_CRUISE;
    if (front <= FRONT_STOP_DIST) velCmd = 0.0f;

    // 8) 결과: UltrasonicCommand에 저장 ★
    UltrasonicCommand ucmd;
    ucmd.angle  = (int)steerCmd;                 // 예: -20 ~ +20
    ucmd.speed1 = (velCmd > 0.0f) ? 1 : 0;       // VEL_CRUISE==1.0 가정

    // 9) 큐 전송(최신값 유지)
    xQueueOverwrite(ultrasonicCmdQueue, &ucmd);

    // 10) ControlTask로도 전달(ManualCommand 형태) → 기존 제어 경로 재사용
    ManualCommand m;
    m.angle  = ucmd.angle;
    m.speed1 = ucmd.speed1; // ControlTask에서 kph로 사용 → 1 kph/0 kph
    xQueueOverwrite(controlQueue, &m);

    // 11) CSV 로그(옵션)
    unsigned long t = millis() - bootMs;
    Serial.print(t);                Serial.print(",");
    Serial.print(front);            Serial.print(",");
    Serial.print(right);            Serial.print(",");
    Serial.print(wObs, 2);          Serial.print(",");
    Serial.print(angCurb, 2);       Serial.print(",");
    Serial.print(angObs, 2);        Serial.print(",");
    Serial.print(steerCmd, 2);      Serial.print(",");
    Serial.println(velCmd, 2);
  }
}

// ================== setup / loop ==================
void setup() {
  Serial.begin(9600);   // USB 디버깅
  UART1_init(9600);     // UART1
  bootMs = millis();

  // 핀모드
  pinMode(SPEED_MOTOR_FRONT_PWM, OUTPUT);
  pinMode(SPEED_MOTOR_FRONT_DIR, OUTPUT);
  pinMode(SPEED_MOTOR_FRONT_BRK, OUTPUT);

  pinMode(STEERING_MOTOR_PWM_PIN, OUTPUT);
  pinMode(STEERING_MOTOR_DIR_PIN, OUTPUT);
  pinMode(STEERING_MOTOR_BRK_PIN, OUTPUT);
  pinMode(STEERING_ANALOG_PIN, INPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_REAR,  OUTPUT);
  pinMode(ECHO_REAR,  INPUT);

  initEncoders();
  clearEncoderCount();
  previous_pos = readEncoder();

  // 큐 생성
  uartQueue        = xQueueCreate(32, sizeof(uint8_t));
  manualQueue      = xQueueCreate(10, sizeof(char));
  controlQueue     = xQueueCreate(10, sizeof(ManualCommand));
  gpsQueue         = xQueueCreate(10, sizeof(GPSCommand));
  ultrasonicQueue  = xQueueCreate(16, sizeof(uint8_t));        // 유지(옵션)
  cameraQueue      = xQueueCreate(5,  sizeof(int));
  ultrasonicCmdQueue = xQueueCreate(1, sizeof(UltrasonicCommand)); // ★ 최신값 유지

  if (!uartQueue || !manualQueue || !gpsQueue || !ultrasonicQueue ||
      !cameraQueue || !controlQueue || !ultrasonicCmdQueue) {
    Serial.println("Queue creation failed!");
    while (1);
  }

  // 태스크 생성
  xTaskCreate(CommTask,       "Comm",       256, NULL, 2, NULL);
  xTaskCreate(ManualTask,     "Manual",     256, NULL, 2, NULL);
  xTaskCreate(GPSTask,        "GPS",        256, NULL, 2, NULL);
  xTaskCreate(CAMERATask,     "Camera",     256, NULL, 2, NULL);
  xTaskCreate(UltrasonicTask, "Ultrasonic", 384, NULL, 2, NULL); // ★ 새 태스크
  xTaskCreate(ControlTask,    "Control",    384, NULL, 3, NULL); // 제어 우선순위↑

  vTaskStartScheduler(); // FreeRTOS 스케줄러 시작
}

void loop() {
  // ★ 사용 안 함 (FreeRTOS 태스크로만 동작)
}
