// ===============================
// RO60 external encoder feedback
// Arduino Mega 2560
// Encoder A -> pin 2
// Encoder B -> pin 3
// ===============================

const int ENC_A = 2;
const int ENC_B = 3;

// 编码器参数
// 256 PPR，四倍频后：256 * 4 = 1024 counts/rev
const long ENCODER_PPR = 256;
const long COUNTS_PER_REV = ENCODER_PPR * 4;

volatile long encoderCount = 0;

int lastA = 0;
int lastB = 0;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 20;  // 50 Hz

void setup() {
  Serial.begin(115200);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  lastA = digitalRead(ENC_A);
  lastB = digitalRead(ENC_B);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

  Serial.println("RO60 Encoder Feedback Start");
  Serial.println("Output format: angle_deg");
  Serial.println("Input z to zero angle");
}

void loop() {
  // 串口输入 z，可以把当前位置设为 0 度
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'z' || c == 'Z') {
      noInterrupts();
      encoderCount = 0;
      interrupts();

      Serial.println("ZERO");
    }
  }

  unsigned long now = millis();

  if (now - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = now;

    long countCopy;

    noInterrupts();
    countCopy = encoderCount;
    interrupts();

    float angleDeg = (float)countCopy * 360.0 / (float)COUNTS_PER_REV;

    // 为了 MuJoCo / Python 稳定读取，只输出角度数字
    Serial.println(angleDeg, 2);
  }
}

void encoderISR() {
  int A = digitalRead(ENC_A);
  int B = digitalRead(ENC_B);

  int lastState = (lastA << 1) | lastB;
  int currentState = (A << 1) | B;

  // Quadrature decoding
  if (
    (lastState == 0 && currentState == 1) ||
    (lastState == 1 && currentState == 3) ||
    (lastState == 3 && currentState == 2) ||
    (lastState == 2 && currentState == 0)
  ) {
    encoderCount++;
  } else if (
    (lastState == 0 && currentState == 2) ||
    (lastState == 2 && currentState == 3) ||
    (lastState == 3 && currentState == 1) ||
    (lastState == 1 && currentState == 0)
  ) {
    encoderCount--;
  }

  lastA = A;
  lastB = B;
}