// ==========================================
// KHAI BÁO ĐỘNG CƠ SỐ 1
// ==========================================
#define EN_PIN_1 11      // Chân R_EN & L_EN 
#define RPWM_PIN_1 12    // Chân cấp PWM 
#define LPWM_PIN_1 10    // Chân cấp PWM 

#define ENCODER_A_1 3    // Kênh A (Chân ngắt INT1)
#define ENCODER_B_1 8    // Kênh B

// ==========================================
// KHAI BÁO ĐỘNG CƠ SỐ 2
// ==========================================
#define EN_PIN_2 5       // Chân R_EN & L_EN 
#define RPWM_PIN_2 4     // Chân cấp PWM 
#define LPWM_PIN_2 6     // Chân cấp PWM 

#define ENCODER_A_2 2    // Kênh A (Chân ngắt INT0)
#define ENCODER_B_2 7    // Kênh B

// ==========================================
// THÔNG SỐ CÀI ĐẶT
// ==========================================
const long PULSES_PER_REV = 990; 
const long TARGET_PULSES = 5 * PULSES_PER_REV; 

// Cài đặt PWM riêng cho từng động cơ & từng chiều (0 - 255)
const int PWM_M1_FWD = 150;
const int PWM_M1_BWD = 160;

const int PWM_M2_FWD = 155;
const int PWM_M2_BWD = 150;

volatile long pulseCount1 = 0; 
volatile long pulseCount2 = 0; 

// ==========================================
// HÀM ĐỌC XUNG AN TOÀN (TRÁNH LỖI NGẮT)
// ==========================================
long getPulseCount1() {
  noInterrupts();
  long c = pulseCount1;
  interrupts();
  return c;
}

long getPulseCount2() {
  noInterrupts();
  long c = pulseCount2;
  interrupts();
  return c;
}

void setup() {
  Serial.begin(115200);
  
  // 1. Cài đặt chân Driver BTS7960
  pinMode(EN_PIN_1, OUTPUT);
  pinMode(RPWM_PIN_1, OUTPUT);
  pinMode(LPWM_PIN_1, OUTPUT);
  
  pinMode(EN_PIN_2, OUTPUT);
  pinMode(RPWM_PIN_2, OUTPUT);
  pinMode(LPWM_PIN_2, OUTPUT);
  
  digitalWrite(EN_PIN_1, HIGH); 
  digitalWrite(EN_PIN_2, HIGH); 
  
  // 2. Cài đặt chân Encoder
  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);
  
  pinMode(ENCODER_A_2, INPUT_PULLUP);
  pinMode(ENCODER_B_2, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), countPulse1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_2), countPulse2, RISING);
  
  // ==========================================
  // BƯỚC 1: CẢ 2 ĐỘNG CƠ QUAY THUẬN CÙNG LÚC
  // ==========================================
  Serial.println("Kich hoat: Ca 2 dong co QUAY THUAN 5 vong...");
  
  noInterrupts();
  pulseCount1 = 0; 
  pulseCount2 = 0; 
  interrupts();
  
  bool m1_done = false;
  bool m2_done = false;
  
  motor1Forward(PWM_M1_FWD); 
  motor2Forward(PWM_M2_FWD); 
  
  while (!m1_done || !m2_done) {
    if (!m1_done && abs(getPulseCount1()) >= TARGET_PULSES) {
      motor1Stop();
      m1_done = true;
      Serial.println("-> Dong co 1 da xong 5 vong thuan.");
    }
    
    if (!m2_done && abs(getPulseCount2()) >= TARGET_PULSES) {
      motor2Stop();
      m2_done = true;
      Serial.println("-> Dong co 2 da xong 5 vong thuan.");
    }
  }

  // ==========================================
  // BƯỚC 2: QUAY NGUỢC NGAY LẬP TỨC (ĐÃ XÓA DELAY)
  // ==========================================
  Serial.println("Kich hoat ngay: Ca 2 dong co QUAY NGUOC 5 vong...");
  
  noInterrupts();
  pulseCount1 = 0; 
  pulseCount2 = 0; 
  interrupts();
  
  m1_done = false;
  m2_done = false;
  
  motor1Backward(PWM_M1_BWD);
  motor2Backward(PWM_M2_BWD);
  
  while (!m1_done || !m2_done) {
    if (!m1_done && abs(getPulseCount1()) >= TARGET_PULSES) {
      motor1Stop();
      m1_done = true;
      Serial.println("-> Dong co 1 da xong 5 vong nguoc.");
    }
    
    if (!m2_done && abs(getPulseCount2()) >= TARGET_PULSES) {
      motor2Stop();
      m2_done = true;
      Serial.println("-> Dong co 2 da xong 5 vong nguoc.");
    }
  }
  
  Serial.println("HOAN THANH TOAN BO CHU TRINH.");
}

void loop() {
  // Trống
}

// ==========================================
// HÀM NGẮT ENCODER
// ==========================================
void countPulse1() {
  if (digitalRead(ENCODER_B_1) == LOW) pulseCount1++; 
  else pulseCount1--; 
}

void countPulse2() {
  if (digitalRead(ENCODER_B_2) == LOW) pulseCount2++; 
  else pulseCount2--; 
}

// ==========================================
// HÀM ĐIỀU KHIỂN ĐỘNG CƠ 1
// ==========================================
void motor1Forward(int speed) {
  analogWrite(LPWM_PIN_1, 0);
  analogWrite(RPWM_PIN_1, speed);
}

void motor1Backward(int speed) {
  analogWrite(RPWM_PIN_1, 0);
  analogWrite(LPWM_PIN_1, speed);
}

void motor1Stop() {
  analogWrite(RPWM_PIN_1, 0);
  analogWrite(LPWM_PIN_1, 0);
}

// ==========================================
// HÀM ĐIỀU KHIỂN ĐỘNG CƠ 2
// ==========================================
void motor2Forward(int speed) {
  analogWrite(LPWM_PIN_2, 0);
  analogWrite(RPWM_PIN_2, speed);
}

void motor2Backward(int speed) {
  analogWrite(RPWM_PIN_2, 0);
  analogWrite(LPWM_PIN_2, speed);
}

void motor2Stop() {
  analogWrite(RPWM_PIN_2, 0);
  analogWrite(LPWM_PIN_2, 0);
}