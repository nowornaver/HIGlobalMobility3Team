volatile char rxData;
volatile bool newDataFlag = false;
volatile bool txReady = true;
volatile char txData;

// UART1 RX 인터럽트
ISR(USART1_RX_vect) {
  rxData = UDR1;       // 수신 데이터 읽기
  newDataFlag = true;  // 수신 플래그 설정
}

// UART1 TX 데이터 레지스터 빈 상태 인터럽트
ISR(USART1_UDRE_vect) {
  if (!txReady) {
    UDR1 = txData;    // 데이터 전송
    txReady = true;   // 전송 완료
    UCSR1B &= ~(1 << UDRIE1); // 인터럽트 비활성화
  }
}

// 한 바이트 송신
void sendByte(char data) {
  while (!txReady);   // 이전 전송 완료 대기
  txData = data;
  txReady = false;
  UCSR1B |= (1 << UDRIE1); // TX 인터럽트 활성화
}

// UART1 초기화 함수 (보레이트 115200)
void UART1_init(unsigned long baud) {
  uint16_t ubrr = (F_CPU / 16 / baud) - 1;
  UBRR1H = (ubrr >> 8);
  UBRR1L = ubrr;
  
  UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1); // RX, TX, RX 인터럽트
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 데이터 8비트, 패리티 없음, 1 스톱비트
}

void setup() {
  Serial.begin(115200); // USB 시리얼 (디버깅용)
  UART1_init(115200);   // UART1 초기화
  Serial.println("UART1 Echo Test Start");
}

void loop() {
  if (newDataFlag) {
    newDataFlag = false;

    Serial.print("Received from UART1: ");
    Serial.println(rxData);

    sendByte(rxData); // 받은 데이터 그대로 송신 (에코)
  }
}
