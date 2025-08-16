#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include <MsTimer2.h>
#include <queue.h>   // QueueHandle_t, xQueueCreate 등

// 속도 모터 핀 설정
#define SPEED_MOTOR_FRONT_PWM  5
#define SPEED_MOTOR_FRONT_DIR  6
#define SPEED_MOTOR_FRONT_BRK  7

#define LED_PIN_10MS 13   // 내장 LED
#define OSC_PIN_100MS 8   // 오실로스코프 측정용 핀

// 엔코더 핀 설정
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 4

// 조향 모터 핀 설정
#define STEERING_ANALOG_PIN A15
#define STEERING_MOTOR_PWM_PIN  8
#define STEERING_MOTOR_DIR_PIN  9
#define STEERING_MOTOR_BRK_PIN  10


#define TRIG_FRONT 11
#define ECHO_FRONT 12
#include <stdint.h>
#define LED_PIN 13  // 내장 LED


#define TRIG_REAR 31
#define ECHO_REAR 30

QueueHandle_t uartQueue;
QueueHandle_t manualQueue;
QueueHandle_t gpsQueue;
QueueHandle_t ultrasonicQueue;
QueueHandle_t cameraQueue;
QueueHandle_t controlQueue;
QueueHandle_t txQueue; // TX용 큐

enum ControlMode {
  MODE_MANUAL,
  MODE_GPS,
  MODE_Ultrasonic,
  MODE_Camera
};

typedef struct {
    int speed1;       // -1: 뒤로, 0: 정지, 1: 앞으로
    int angle;    // 조향각 (-26 ~ 26)
} ManualCommand;

typedef struct {
    int8_t angle;   // -26 ~ 26
    uint8_t speed1;  // 0 또는 1
    bool parityOk;  // 패리티 체크 결과
} GPSCommand;

typedef struct {
  int speed1;
} CameraCommand;

typedef struct {
  int speed1;
  int angle;
} UltrasonicCommand;
ControlMode currentMode = MODE_MANUAL;







volatile uint32_t tick1ms = 0;

// ======= 주기용 세마포어 =======
SemaphoreHandle_t sem10ms;
SemaphoreHandle_t sem100ms;
// ---------------- Control params (현장 튜닝) ----------------
const float MAX_ANGLE   = 20.0f;   // 최대 조향각 [deg]
const float D_CURB      = 10.0f;  // 오른쪽(연석) 목표거리 [cm]
const float D_OBS       = 120.0f;  // 전방(장애물) 목표거리 [cm]

const float OBS_ENTER   = 160.0f;  // 전방 가까우면 장애물 가중치↑
const float OBS_EXIT    = 200.0f;  // 전방 멀어지면 장애물 가중치↓

const float KP_CURB     = 0.22f;   // 우측 거리 P 게인
const float KD_CURB     = 0.00f;   // 필요시 0.05~0.10
const float KP_OBS      = 0.25f;   // 전방 거리 P 게인
const float KD_OBS      = 0.00f;

const float ANG_SLEW    = 4.0f;    // 프레임당 최대 각 변화량 [deg]

// 센서 유효 범위(모델 스펙)
const float MIN_VALID_CM = 20.0f;
const float MAX_VALID_CM = 600.0f;

// ---- SR 데드밴드 (연석 거리 유지 무시 구간) ----
const float SR_DEADBAND_MIN = 100.0f;  // cm
const float SR_DEADBAND_MAX = 150.0f;  // cm

// ---- 속도 로직 ----
const float FRONT_STOP_DIST = 100.0f;  // 전방 <= 이 값이면 정지
const float VEL_CRUISE      = 1.0f;    // 기본 주행 속도(임의 단위), 모터 연동시 매핑

unsigned long startTime;

// --- 내부 상태/출력 ---
volatile float steeringTargetDeg = 0.0f;  // 최종 조향 명령 (deg)
volatile float speedTarget       = 0.0f;  // 최종 속도 명령 (VEL_CRUISE or 0)
static float wObs = 0.0f;                 // 장애물 제어 가중치 0..1
static float prevAng = 0.0f;
static float prevErrCurb = 0.0f, prevErrObs = 0.0f;
static unsigned long prevT_us = 0;


// 타이머
int targetAngle = 0 ;
int toggle_count = 0;
//void Interrupt_10ms() { toggle_count++; }
//volatile char rxData;
volatile bool newDataFlag = false;
//초음파 거리값
volatile int latestUltrasonicDistance = 0;

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
//char steeringAngle = 0;  // GPS 에서 받은 조향각

// 조향 PID 변수 (Pot 기반)
int currentPotValue = 0, targetPotValue = 0;
double steering_pwmValue = 0.0;
const double Kp_steering = 0.75, Ki_steering = 0.2, Kd_steering = 0.1;
double integral_steering = 0.0, derivative_steering = 0.0;
double previous_error_steering = 0.0;
const double integralLimit = 50.0;

// 시리얼 입력값
volatile double speed_angle_queue[2][2] = {{0.0, 0.0}, {0.0, 0.0}};


// UART1 RX 인터럽트
ISR(USART1_RX_vect) {
    uint8_t data = UDR1;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(uartQueue, &data, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(); // Mega에서는 인자 없음
    }
}

// UART1 TX 데이터 레지스터 빈 상태 인터럽트
ISR(USART1_UDRE_vect) {
    char data;
    if (xQueueReceiveFromISR(txQueue, &data, NULL) == pdTRUE) {
        UDR1 = data; // 큐에서 꺼낸 데이터 전송
    } else {
        // 큐 비어있으면 인터럽트 종료
        UCSR1B &= ~(1 << UDRIE1);
    }
}
ISR(TIMER1_COMPA_vect) {
  tick1ms++;

    BaseType_t xHigherPriorityTaskWoken10 = pdFALSE;
    BaseType_t xHigherPriorityTaskWoken100 = pdFALSE;

    if (tick1ms % 10 == 0) {
        xSemaphoreGiveFromISR(sem10ms, &xHigherPriorityTaskWoken10);
    }

    if (tick1ms % 100 == 0) {
        xSemaphoreGiveFromISR(sem100ms, &xHigherPriorityTaskWoken100);
    }

    // 한 번만 호출
    if (xHigherPriorityTaskWoken10 || xHigherPriorityTaskWoken100) {
        portYIELD_FROM_ISR();
    }
}
void setupTimer1_1ms() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  OCR1A = 249; // 16MHz / 64분주 / 250 = 1ms
  TCCR1B |= (1 << WGM12); // CTC 모드
  TCCR1B |= (1 << CS11) | (1 << CS10); // 64분주
  TIMSK1 |= (1 << OCIE1A); // 비교일치 인터럽트 허용
  interrupts();
}
void handleManualControl(char cmd) {

      static int speed1 = 0;
    static int steeringAngle = 0;
    ManualCommand manualCmd;
  switch (cmd) {
    case 'w': 
      speed1 = 1; 
      break;
    case 's': 
      speed1 = -1; 
      break;
    case 'd': 
    steeringAngle = max(-26, steeringAngle - 5);
        break;
    case 'a': 
    steeringAngle = min(26, steeringAngle + 5);
        break;
    case 'x': 
        speed1 = 0; 
        steeringAngle = 0; 
        break;
  }

      manualCmd.speed1 = speed1;
    manualCmd.angle = steeringAngle;
    if (xQueueSend(controlQueue, &manualCmd, 10 / portTICK_PERIOD_MS) != pdPASS) {
        // 큐가 가득 차서 넣기 실패 시 로그 출력 등 처리 가능
        Serial.println("Warning: manualQueue full, command lost!");
    }
}
// 한 바이트 송신
void sendByte(char data) {
    if (xQueueSend(txQueue, &data, 0) != pdPASS) {
        Serial.println("TX Queue Full!"); // 큐가 꽉 찼으면 처리
    }
    // TX 인터럽트 활성화
    UCSR1B |= (1 << UDRIE1);
}

void UART1_init(unsigned long baud) {
  uint16_t ubrr = (F_CPU / 16 / baud) - 1;
  UBRR1H = (ubrr >> 8);
  UBRR1L = ubrr;
  
  UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1); // RX, TX, RX 인터럽트
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 데이터 8비트, 패리티 없음, 1 스톱비트
}
// --- 선형 보간: 목표 각도 → 목표 Pot 값
int getPotFromAngle(int targetAngle) {
  const double angle0 = -26.0;
  const double angle1 = 26.0;
  const int pot0 = 10;
  const int pot1 = 1018;
return pot0 + (int)(((float)(targetAngle - angle0)) * (float)(pot1 - pot0) / (angle1 - angle0));
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

void handleGPSData(char data) {
  
  // 여기서 GPS 각도 처리 로직 구현
  // 예: 데이터 프로토콜을 따로 설계
}

void handleUltrasonicData (char data) {
  for (;;) {

    
  }
}

void handleCameraData(char data) {

  for (;;) {
    
  }
}
GPSCommand GPS_unpack(uint8_t data) {
    GPSCommand cmd;
    
    uint8_t dataNoParity = data & 0x7F;   // 상위 7비트
    uint8_t parityBit = (data >> 7) & 0x01;

    // 홀수 패리티 계산
    uint8_t calcParity = 0;
    for (int i = 0; i < 7; i++)
        calcParity ^= (dataNoParity >> i) & 0x01;

    cmd.parityOk = (parityBit == calcParity);
    cmd.speed = (dataNoParity >> 6) & 0x01;               // 7번째 비트
    cmd.angle = (int8_t)((dataNoParity & 0x3F) - 26);    // 1~6비트 -> -26~26

    return cmd;
}
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(30);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // timeout 30ms
  if (duration == 0) return -1; // timeout
  float distance = duration * 0.0343 / 2;
  return round(distance / 10.0) * 10; // 10단위 반올림
}
static inline float clampf(float x, float a, float b){
  return x < a ? a : (x > b ? b : x);
}
static inline float slew(float prev, float tgt, float step){
  float d = tgt - prev;
  if (d >  step) d =  step;
  if (d < -step) d = -step;
  return prev + d;
}

void updateObsWeight(float fc){
  float tgt;
  if (fc < 0) {
    tgt = wObs; // 무효면 유지
  } else if (fc < OBS_ENTER) {
    float x = (OBS_ENTER - fc) / OBS_ENTER;   // 가까울수록 0→1
    tgt = clampf(x, 0.0f, 1.0f);
  } else if (fc > OBS_EXIT) {
    tgt = 0.0f;
  } else {
    tgt = wObs; // 중간 구간 유지
  }
  wObs = slew(wObs, tgt, 0.10f);
}

void ManualTask(void *pvParameters) {
  char rxData;
  for (;;) {
    if (xQueueReceive(manualQueue, &rxData, portMAX_DELAY) == pdTRUE) {
      handleManualControl(rxData);
    }
  }
}
void SensorTask(void *pvParameters) { //초음파
  
    UltrasonicCommand ultraCommand = {0,0};
    ManualCommand manualCmd = {0,0};     // controlQueue에 넣는 데이터
    static TickType_t nextAdjustTick = 0;
    for (;;) {
      Serial.println(latestUltrasonicDistance);
      Serial.println(manualCmd.angle);
      if (currentMode == MODE_Ultrasonic){
        latestUltrasonicDistance = readUltrasonic(TRIG_FRONT, ECHO_FRONT); // 센서 읽기
        if (latestUltrasonicDistance < MIN_VALID_CM || latestUltrasonicDistance > MAX_VALID_CM) {
            latestUltrasonicDistance = -1;
        }
        TickType_t now = xTaskGetTickCount();
        // 초음파 데이터가 무효면 바로 다음 루프로 (딜레이 포함)
        if (latestUltrasonicDistance < 0) {
            vTaskDelayUntil(&xLastWakeTime, 50 / portTICK_PERIOD_MS);
            continue;
        }

if (now >= nextAdjustTick){if (latestUltrasonicDistance < 130) {
            manualCmd.speed1 = 0;
            manualCmd.angle = manualCmd.angle-5;
            // xQueueSend(controlQueue, &manualCmd, 10 / portTICK_PERIOD_MS);
        }
        else if (latestUltrasonicDistance > 150) {
          manualCmd.angle = manualCmd.angle+5;
        }

        else {
            manualCmd.speed1 = 0;
            manualCmd.angle = 0;

        }

      if (manualCmd.angle >  (int)MAX_ANGLE) manualCmd.angle =  (int)MAX_ANGLE;
      if (manualCmd.angle < -(int)MAX_ANGLE) manualCmd.angle = -(int)MAX_ANGLE;
      nextAdjustTick = now + pdMS_TO_TICKS(1000);
      }
        // 장애물 가까우면 속도 0 명령 전송
        

                    xQueueSend(controlQueue, &manualCmd, 10 / portTICK_PERIOD_MS);


    }

            vTaskDelayUntil(&xLastWakeTime, 50 / portTICK_PERIOD_MS);

    }
}

// ======= LED Task (10ms마다 토글) =======
void LedTask10ms(void *pvParameters) {
  pinMode(LED_PIN_10MS, OUTPUT);
  for (;;) {
    if (xSemaphoreTake(sem10ms, portMAX_DELAY) == pdTRUE) {
      digitalWrite(LED_PIN_10MS, !digitalRead(LED_PIN_10MS));
    }
  }
}

void OscTask100ms(void *pvParameters) {
  pinMode(OSC_PIN_100MS, OUTPUT);
  for (;;) {
    if (xSemaphoreTake(sem100ms, portMAX_DELAY) == pdTRUE) {
      digitalWrite(OSC_PIN_100MS, !digitalRead(OSC_PIN_100MS));
//      Serial.println("100ms tick"); // Task가 동작하는지 확인
    }
  }
}

void CAMERATask(void *pvParameters) { 
    int cameraSpeed;
    // Serial.println(cameraSpeed);
      ManualCommand manualCmd;     // controlQueue에 넣는 데이터
  Serial.println("Camera Task Running");

  for (;;) {
    // 카메라 상위 제어기에서 speed 값 받기
    if (xQueueReceive(cameraQueue, &cameraSpeed, portMAX_DELAY) == pdTRUE) {
            Serial.println(cameraSpeed);  // 받은 값 그대로 출력
            manualCmd.speed1 = cameraSpeed; 
            manualCmd.angle = 0;  // 카메라는 각도 명령 안 줌
      // controlQueue로 전달
      if (xQueueSend(controlQueue, &manualCmd, 10 / portTICK_PERIOD_MS) != pdPASS) {
        Serial.println("Warning: controlQueue full, Camera command lost!");
      }
    }
  }
}
void ControlTask(void *pvParameters) {
      ManualCommand cmd;

    Serial.println("CONTROLTASK running");

  for (;;) {

    // PID 연산, 모터 제어
           currentPotValue = analogRead(STEERING_ANALOG_PIN);
if (xSemaphoreTake(sem10ms, portMAX_DELAY) == pdTRUE) {
 if (xQueueReceive(controlQueue, &cmd,  10 / portTICK_PERIOD_MS) == pdTRUE) {
    if (cmd.angle >25 || cmd.angle <-25) {
  cmd.angle = 0;
}
    speed_angle_queue[0][0] = cmd.speed1;
    speed_angle_queue[0][1] = cmd.angle;

 }
     speed_angle_queue[1][0] = cmd.speed1;
    speed_angle_queue[1][1] = cmd.angle;
  desiredSpeed_kph = speed_angle_queue[0][0];
  targetAngle = speed_angle_queue[0][1];
//   Serial.println(cmd.speed1);
//Serial.println(cmd.speed1);

  if (desiredSpeed_kph == 0.0) {
    setMotor(0, 0);  // 정지
    cmd.angle = 1.0;
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
//Serial.print("Cmd Angle: ");
//Serial.println(targetAngle);
//Serial.print("TargetPotValue: ");
//Serial.println(targetPotValue);
//Serial.print("currentPotValue");
//Serial.println(currentPotValue);
//Serial.print("Steering PWM: ");
//Serial.println(steering_pwmValue);

    


}
  }
}
void GPSTask(void *pvParameters) {
    int gpsAngle = 0;
    char rxChar;
    GPSCommand gpsCmd;
    ManualCommand manualCmd;     // controlQueue에 넣는 데이터

    for (;;) {
        // 큐에서 GPS 데이터(조향각 문자)를 1바이트씩 받는다고 가정
        if (xQueueReceive(gpsQueue, &gpsCmd, portMAX_DELAY) == pdTRUE) {
            // 예: 조향각은 -26 ~ 26 사이 정수값으로 시리얼에서 ASCII 숫자 형태로 들어온다고 가정
            // 실제 구현에 맞게 파싱 수정 필요
            Serial.print("[GPS MODE] Received angle: ");
                           Serial.println(gpsCmd.angle);

            
//            Serial.println(gpsCmd.angle);
            // 음수 표현 등 필요 시 별도 프로토콜 구현
            
            // GPS 조향각 명령 만들기 (속도는 0으로 설정)
         manualCmd.speed1 = 0;  // 속도 0 (필요에 따라 조절)
         manualCmd.angle = gpsCmd.angle;
         Serial.println(manualCmd.angle);
        

            // controlQueue에 넣기
            if (xQueueSend(controlQueue, &manualCmd, 10 / portTICK_PERIOD_MS) != pdPASS) {
                Serial.println("Warning: controlQueue full, GPS command lost!");
            }
        }
    }
}


void CommTask(void *pvParameters) {
    Serial.println("CommTask running");
    int8_t rxData;
    bool gpsModeActive = false;
    bool UltrasonicActive = false;
    bool CameraModeActive = false;
    static char gpsBuffer[8];  // angle 입력 버퍼
    static uint8_t gpsIndex = 0;
  for (;;) {
    // UART 수신 처리
    if (xQueueReceive(uartQueue, &rxData,  portMAX_DELAY) == pdTRUE) {
      switch (rxData) {
        case 'K': // Manual mode
          currentMode = MODE_MANUAL;
          Serial.println("Mode: MANUAL");
          continue;
        case 'G': // GPS mode
          currentMode = MODE_GPS;
          gpsModeActive = true;
          UltrasonicActive=false;
          CameraModeActive=false;
          Serial.println("Mode: GPS");
          continue;
        case 'U': // Ultrasonic mode
          currentMode = MODE_Ultrasonic;
          gpsModeActive =false;
          UltrasonicActive=true;
          CameraModeActive=false;
          Serial.println("Mode: ULTRASONIC");
          continue;
        case 'C': // Camera mode
          currentMode = MODE_Camera;
          gpsModeActive =false;
          UltrasonicActive=false;
          CameraModeActive=true;
          Serial.println("Mode: CAMERA");
          continue;
      }

      // 모드별 처리
      switch (currentMode) {
        case MODE_MANUAL:
          xQueueSend(manualQueue, &rxData, 20);
          break;
        case MODE_GPS:
        if (gpsModeActive) {
        GPSCommand gpsCmd=GPS_unpack(rxData);
        if (!gpsCmd.parityOk) {
            Serial.println("[GPS MODE] Parity error!");
            break;
        }
          Serial.print("[GPS MODE] angle: ");
          Serial.print(gpsCmd.angle);
          Serial.print(", speed: ");
          Serial.println(gpsCmd.speed);
            xQueueSend(gpsQueue, &gpsCmd, 20);
}
          break;
        case MODE_Ultrasonic:
        if (UltrasonicActive) {
          UltrasonicCommand ultraCommand ;
          if (latestUltrasonicDistance <500) {
            ultraCommand.speed1 =0;
            xQueueSend(ultrasonicQueue, &ultraCommand, 20);

          }
          
        }
          break;
        case MODE_Camera:
            if (rxData >= '0' && rxData <= '9') {
            int val = rxData - '0';  // '1' -> 1
             xQueueSend(cameraQueue, &val, 20);
  }          break;
      }
      
  }

}
}
// --- Setup

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


  
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

    pinMode(TRIG_REAR, OUTPUT);
  pinMode(ECHO_REAR, INPUT);

  
  // 세마포어 생성
  sem10ms = xSemaphoreCreateBinary();
  sem100ms = xSemaphoreCreateBinary();

  
  // 1ms 타이머 시작
  setupTimer1_1ms();

  
  initEncoders();
  clearEncoderCount();

  previous_pos = readEncoder();  // ✅ 초기값 설정
 uartQueue = xQueueCreate(32, sizeof(int8_t));
manualQueue = xQueueCreate(10, sizeof(int8_t));  // rxData(char) 넣으니 이렇게
controlQueue = xQueueCreate(10, sizeof(ManualCommand));
gpsQueue = xQueueCreate(10, sizeof(GPSCommand));
ultrasonicQueue = xQueueCreate(10, sizeof(int8_t));
cameraQueue = xQueueCreate(10, sizeof(int));
txQueue = xQueueCreate(32, sizeof(char)); // 최대 32바이트 전송 버퍼

  if (!uartQueue || !manualQueue || !gpsQueue || !ultrasonicQueue || !cameraQueue) {
    Serial.println("Queue creation failed!");
    while (1);
  }
  
  // Task 생성
  xTaskCreate(LedTask10ms, "LED10", 128, NULL, 3, NULL);
  xTaskCreate(OscTask100ms, "OSC100", 128, NULL, 2, NULL);
  xTaskCreate(CommTask, "COMM", 128, NULL, 1, NULL);
  xTaskCreate(SensorTask, "Sensor", 128, NULL, 1, NULL);
  xTaskCreate(ControlTask, "Control", 256, NULL, 2, NULL);
    xTaskCreate(ManualTask, "ManualTask", 128, NULL, 1, NULL);
  xTaskCreate(GPSTask, "GPS", 128, NULL, 1, NULL);  // GPS Task 추가
  xTaskCreate(CAMERATask, "Camera", 128, NULL, 1, NULL);  // GPS Task 추가
  vTaskStartScheduler();

}

// --- Main Loop


void loop() {

}
