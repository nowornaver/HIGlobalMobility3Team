#include <avr/interrupt.h>

#define TEST_PIN 8    // 오실로스코프 측정용 핀 번호

volatile bool flag1ms = false;
volatile bool flag10ms = false;
volatile bool flag100ms = false;

volatile uint16_t cnt10ms = 0;
volatile uint16_t cnt100ms = 0;

volatile bool testPinState = false;

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);   // 내장 LED
  pinMode(TEST_PIN, OUTPUT);      // 테스트 핀
  digitalWrite(TEST_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);

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

void loop() {
  if (flag1ms) {
    flag1ms = false;
    // 필요 시 1ms 작업 추가
  }

  if (flag10ms) {
    flag10ms = false;

    // 10ms마다 TEST_PIN 토글 (20ms 주기 신호)


    // Serial.println("10ms Task 실행"); // 측정 시 간섭 가능 -> 필요시만 활성화
  }

  if (flag100ms) {
    flag100ms = false;
    // 100ms마다 LED 토글
        testPinState = !testPinState;
    digitalWrite(TEST_PIN, testPinState);
//    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // PC에서 데이터 들어오면 에코
  if (Serial.available() > 0) {
    String received = Serial.readStringUntil('\n');
    Serial.print("Arduino received: ");
    Serial.println(received);
  }
}
