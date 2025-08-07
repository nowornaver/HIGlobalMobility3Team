#define TRIG_FRONT 11
#define ECHO_FRONT 12

#define TRIG_REAR 31
#define ECHO_REAR 30

unsigned long startTime;

void setup() {
  Serial.begin(115200);
  startTime = millis();

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_REAR, OUTPUT);
  pinMode(ECHO_REAR, INPUT);

  // CSV 헤더 출력
  Serial.println("Time(ms),Front(cm),Rear(cm)");
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
  float front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  float rear  = readUltrasonic(TRIG_REAR,  ECHO_REAR);

  unsigned long currentTime = millis() - startTime;

  // CSV 형태 출력: 시간,전방,후방
  Serial.print(currentTime);
  Serial.print(",");
  Serial.print(front);
  Serial.print(",");
  Serial.println(rear);



  delay(100);  // 0.1초 간격
}
