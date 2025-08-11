#include <Arduino_FreeRTOS.h>

#include <MsTimer2.h>
#include <queue.h>   // QueueHandle_t, xQueueCreate 등

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


#define TRIG_FRONT 11
#define ECHO_FRONT 12


#define TRIG_REAR 31
#define ECHO_REAR 30

QueueHandle_t uartQueue;
QueueHandle_t manualQueue;
QueueHandle_t gpsQueue;
QueueHandle_t ultrasonicQueue;
QueueHandle_t cameraQueue;
QueueHandle_t controlQueue;
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

ControlMode currentMode = MODE_MANUAL;

// 타이머
int toggle_count = 0;
//void Interrupt_10ms() { toggle_count++; }
//volatile char rxData;
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
  if (!txReady) {
    UDR1 = txData;    // 데이터 전송
    txReady = true;   // 전송 완료
    UCSR1B &= ~(1 << UDRIE1); // 인터럽트 비활성화

  }
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
    case 'a': 
    steeringAngle = max(-26, steeringAngle - 5);
        break;
    case 'd': 
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

void ManualTask(void *pvParameters) {
  char rxData;
  for (;;) {
    if (xQueueReceive(manualQueue, &rxData, portMAX_DELAY) == pdTRUE) {
      handleManualControl(rxData);
    }
  }
}
void SensorTask(void *pvParameters) {
  for (;;) {
    // 센서 읽기 코드
    vTaskDelay(100 / portTICK_PERIOD_MS); // 20Hz 주기
      float front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  delay(20); // 간섭 방지
  float rear  = readUltrasonic(TRIG_REAR,  ECHO_REAR);

  }
}

void ControlTask(void *pvParameters) {
      ManualCommand cmd;

    Serial.println("CONTROLTASK running");

  for (;;) {
       currentPotValue = analogRead(STEERING_ANALOG_PIN);
    // PID 연산, 모터 제어
    vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz 주기
 if (xQueueReceive(controlQueue, &cmd, portMAX_DELAY) == pdTRUE) {
    if (cmd.angle >25 || cmd.angle <-25) {
  cmd.angle = 0;
}
    speed_angle_queue[0][0] = cmd.speed1;
    speed_angle_queue[0][1] = cmd.angle;
    speed_angle_queue[1][0] = cmd.speed1;
    speed_angle_queue[1][1] = cmd.angle;
    
  desiredSpeed_kph = speed_angle_queue[0][0];
  int targetAngle = speed_angle_queue[0][1];

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
  
 }


    

     

  }
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


void CommTask(void *pvParameters) {
    Serial.println("CommTask running");
    uint8_t rxData;

  for (;;) {
    // UART 수신 처리
    if (xQueueReceive(uartQueue, &rxData, portMAX_DELAY) == pdTRUE) {
      switch (rxData) {
        case 'K': // Manual mode
          currentMode = MODE_MANUAL;
          Serial.println("Mode: MANUAL");
          continue;
        case 'G': // GPS mode
          currentMode = MODE_GPS;
          Serial.println("Mode: GPS");
          continue;
        case 'U': // Ultrasonic mode
          currentMode = MODE_Ultrasonic;
          Serial.println("Mode: ULTRASONIC");
          continue;
        case 'C': // Camera mode
          currentMode = MODE_Camera;
          Serial.println("Mode: CAMERA");
          continue;
      }

      // 모드별 처리
      switch (currentMode) {
        case MODE_MANUAL:
          xQueueSend(manualQueue, &rxData, 20);
          break;
        case MODE_GPS:
          xQueueSend(gpsQueue, &rxData, 20);
          break;
        case MODE_Ultrasonic:
          xQueueSend(ultrasonicQueue, &rxData, 20);
          break;
        case MODE_Camera:
          xQueueSend(cameraQueue, &rxData, 20);
          break;
      }
      

  }
//  else {
//          steeringAngle = 0;
//      speed1 = 0;
//  }
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

  
  initEncoders();
  clearEncoderCount();
//QueueHandle_t uartQueue;
//QueueHandle_t manualQueue;
//QueueHandle_t gpsQueue;
//QueueHandle_t ultrasonicQueue;
//QueueHandle_t cameraQueue;
  previous_pos = readEncoder();  // ✅ 초기값 설정
 uartQueue = xQueueCreate(32, sizeof(uint8_t));
manualQueue = xQueueCreate(10, sizeof(char));  // rxData(char) 넣으니 이렇게
controlQueue = xQueueCreate(10, sizeof(ManualCommand));
gpsQueue = xQueueCreate(16, sizeof(uint8_t));
ultrasonicQueue = xQueueCreate(16, sizeof(uint8_t));
cameraQueue = xQueueCreate(16, sizeof(uint8_t));

  if (!uartQueue || !manualQueue || !gpsQueue || !ultrasonicQueue || !cameraQueue) {
    Serial.println("Queue creation failed!");
    while (1);
  }
  xTaskCreate(SensorTask, "Sensor", 128, NULL, 1, NULL);
  xTaskCreate(ControlTask, "Control", 256, NULL, 1, NULL);
  xTaskCreate(CommTask, "Comm", 128, NULL, 1, NULL);
    xTaskCreate(ManualTask, "ManualTask", 128, NULL, 1, NULL);


  vTaskStartScheduler();

}

// --- Main Loop

void loop() {


}
