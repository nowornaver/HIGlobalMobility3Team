#define RX_BUF_SIZE 64
String inputString = ""; // 수신 버퍼
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  inputString.reserve(RX_BUF_SIZE); // 메모리 미리 확보
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (stringComplete) {
    Serial.print("Arduino received: ");
    Serial.println(inputString);

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // LED 토글
    inputString = ""; // 버퍼 초기화
    stringComplete = false;
  }
}

// Serial 이벤트 처리
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
