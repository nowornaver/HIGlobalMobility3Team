#define TRIG_FRONT 11
#define ECHO_FRONT 12

#define TRIG_REAR 31   // 오른쪽 센서로 사용 (핀 이름만 REAR)
#define ECHO_REAR 30

// ---------------- Control params (현장 튜닝) ----------------
const float MAX_ANGLE   = 20.0f;   // 최댓 조향각 [deg]
const float D_CURB      = 100.0f;  // 오른쪽(연석) 목표거리 [cm]
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

void setup() {
  Serial.begin(115200);
  startTime = millis();

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_REAR, OUTPUT);
  pinMode(ECHO_REAR, INPUT);

  // CSV 헤더에 vel 추가
  Serial.println("Time(ms),Front(cm),Right(cm),wObs,angCurb(deg),angObs(deg),cmd(deg),vel");
  prevT_us = micros();
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
  return round(distance / 10.0) * 10; // 10단위 반올림 (원본 유지)
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

// 전방 거리 기반 wObs 업데이트(히스테리시스 + 부드럽게)
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

void loop() {
  // 1) 센서 읽기
  float front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);  // FC
  float right = readUltrasonic(TRIG_REAR,  ECHO_REAR);   // SR

  // 2) 유효범위 필터: -1 또는 20~600 밖이면 무효로 간주 -> 이번 줄 스킵
  if (front < MIN_VALID_CM || front > MAX_VALID_CM) front = -1;
  if (right < MIN_VALID_CM || right > MAX_VALID_CM) right = -1;
  if (front < 0 || right < 0) { delay(100); return; }

  // 3) dt 계산
  unsigned long now_us = micros();
  float dt = (now_us - prevT_us) * 1e-6f;
  if (dt < 1e-3f) dt = 1e-3f;
  prevT_us = now_us;

  // 4) 장애물 가중치
  updateObsWeight(front);

  // 5) 각 제어각 계산
  // 5-1) 연석 유지각 (SR 데드밴드 적용)
  float angCurb = 0.0f;
  if (right >= SR_DEADBAND_MIN && right <= SR_DEADBAND_MAX) {
    angCurb = 0.0f;
    prevErrCurb = 0.0f; // D항 잔여 제거
  } else {
    float eCurb = right - D_CURB;             // +: 멂 → 우(+각), -: 가까움 → 좌(-각)
    float dCurb = (eCurb - prevErrCurb) / dt;
    angCurb =  (KP_CURB * eCurb + KD_CURB * dCurb); // 부호 의도대로
    prevErrCurb = eCurb;
  }

  // 5-2) 장애물 유지각
  float eObs = front - D_OBS;                 // +: 멂 → 우(+), -: 가까움 → 좌(-)
  float dObs = (eObs - prevErrObs) / dt;
  float angObs =  (KP_OBS * eObs + KD_OBS * dObs);
  prevErrObs = eObs;

  // 6) 블렌딩 → 제한 → 슬루
  float angTgt = (1.0f - wObs) * angCurb + wObs * angObs;
  angTgt = clampf(angTgt, -MAX_ANGLE, +MAX_ANGLE);

  float steerCmd = slew(prevAng, angTgt, ANG_SLEW);
  steerCmd = round(steerCmd);    // ★ 소수점 제거 (정수로 반올림)
  prevAng = steerCmd;
  steeringTargetDeg = steerCmd;  // 최종 조향 명령 (정수)

  // === 7) 속도 결정 ===
  float velCmd = VEL_CRUISE;
  if (front <= FRONT_STOP_DIST) {
    velCmd = 0.0f;  // 전방 100cm 이하 → 정지
  }
  speedTarget = velCmd;

  // 8) CSV 출력
  unsigned long t = millis() - startTime;
  Serial.print(t);                Serial.print(",");
  Serial.print(front);            Serial.print(",");   // Front(cm)
  Serial.print(right);            Serial.print(",");   // Right(cm)
  Serial.print(wObs, 2);          Serial.print(",");
  Serial.print(angCurb, 2);       Serial.print(",");
  Serial.print(angObs, 2);        Serial.print(",");
  Serial.print(steerCmd, 2);      Serial.print(",");
  Serial.println(velCmd, 2);      // 마지막 컬럼: 속도 명령

  delay(100);  // 10 Hz
}
