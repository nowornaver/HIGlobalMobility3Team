#define TRIG_FRONT 11
#define ECHO_FRONT 12
#include <avr/interrupt.h>

#define TRIG_REAR 31
#define ECHO_REAR 30
volatile bool flag1ms = false;
volatile bool flag10ms = false;
volatile bool flag100ms = false;

volatile uint16_t cnt10ms = 0;
volatile uint16_t cnt100ms = 0;
unsigned long startTime;

void setup() {
  Serial.begin(9600);
  startTime = millis();

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  noInterrupts();  // 인터럽트 비활성화
  // Timer1 초기화
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 249;  // 1ms마다 인터럽트 (16MHz / 64 prescaler)
  TCCR1B |= (1 << WGM12);         // CTC 모드
  TCCR1B |= (1 << CS11) | (1 << CS10); // 분주율 64
  TIMSK1 |= (1 << OCIE1A);        // 비교 일치 인터럽트 허용

  interrupts();   // 인터럽트 활성화
  pinMode(TRIG_REAR, OUTPUT);
  pinMode(ECHO_REAR, INPUT);

  // CSV 헤더 출력
  Serial.println("Time(ms),Front(cm),Rear(cm)");
}
ISR(TIMER1_COMPA_vect) {
  flag1ms = true;
  cnt10ms++;
  cnt100ms++;

  if (cnt10ms >= 10) {   // 10ms
    flag10ms = true;
    cnt10ms = 0;
  }
  if (cnt100ms >= 100) { // 100ms
    flag100ms = true;
    cnt100ms = 0;
  }
}
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(30);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // timeout 30ms
  if (duration == 0) return -1; // timeout
  float distance = duration * 0.0343 / 2;
  return round(distance / 10.0) * 10; // 10단위 반올림
}

void loop() {
  if (flag100ms) {
  float front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  delay(20); // 간섭 방지

  float rear  = readUltrasonic(TRIG_REAR,  ECHO_REAR);

  unsigned long currentTime = millis() - startTime;
  Serial.print(currentTime);
  Serial.print(",");
  Serial.print(front);
  Serial.print(",");
  Serial.println(rear);
    flag100ms = false;


  }



}
