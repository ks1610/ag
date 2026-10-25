// --- Khai báo chân kết nối BTS7960 #2 ---
#define EN_PIN 11       // Chân R_EN & L_EN 
#define RPWM_PIN 12      // Chân cấp PWM 
#define LPWM_PIN 10      // Chân cấp PWM 

// --- Khai báo chân kết nối Encoder JGB37 #2 ---
#define ENCODER_A 3      // Kênh A (Chân ngắt)
#define ENCODER_B 8      // Kênh B

// // ==========================================
// // KHAI BÁO ĐỘNG CƠ SỐ 2
// // ==========================================
// // Chân kết nối BTS7960 #2
// #define EN_PIN_2 5       // Chân R_EN & L_EN 
// #define RPWM_PIN_2 4     // Chân cấp PWM 
// #define LPWM_PIN_2 6     // Chân cấp PWM 

// // Chân kết nối Encoder JGB37 #2
// #define ENCODER_A_2 2    // Kênh A (Chân ngắt)
// #define ENCODER_B_2 7    // Kênh B

// --- Thông số số xung ---
// Động cơ JGB37 110 vòng/phút thường có tỷ số truyền 1:90 và encoder 11 xung.
// Số xung trên 1 vòng trục ra = 11 * 90 = 990 xung (đếm 1 sườn).
const long PULSES_PER_REV = 990; 
const long TARGET_PULSES = 5 * PULSES_PER_REV; // Tổng số xung cho 5 vòng

volatile long pulseCount = 0; // Biến lưu trữ số xung đếm được

void setup() {
  Serial.begin(115200);
  
  // 1. Cài đặt chân Driver
  pinMode(EN_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  
  digitalWrite(EN_PIN, HIGH); // Bật IC driver BTS7960
  
  // 2. Cài đặt chân Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // Khởi tạo ngắt kích hoạt khi Kênh A chuyển từ LOW lên HIGH (RISING)
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), countPulse, RISING);
  
  // ==========================================
  // CHU TRÌNH ĐIỀU KHIỂN ĐỘNG CƠ
  // ==========================================
  
  // Bước 1: Quay thuận 5 vòng
  Serial.println("Bat dau quay thuan 5 vong...");
  pulseCount = 0; 
  motorForward(150); // Cấp tốc độ PWM (0-255)
  
  // Chờ cho đến khi đếm đủ số xung của 5 vòng
  while(abs(pulseCount) < TARGET_PULSES) {
    // Động cơ đang quay...
  }
  
  motorStop();
  Serial.println("Da quay xong 5 vong thuan. Dung 2 giay.");
  delay(2000); // Dừng lại 2 giây
  
  // Bước 2: Quay ngược 5 vòng
  Serial.println("Bat dau quay nguoc 5 vong...");
  pulseCount = 0; // Reset lại bộ đếm để đếm lại 5 vòng
  motorBackward(150);
  
  while(abs(pulseCount) < TARGET_PULSES) {
    // Động cơ đang quay ngược...
  }
  
  motorStop();
  Serial.println("Da quay xong 5 vong nguoc. Ket thuc.");
}

void loop() {
  // Để trống vì chỉ cần quay 1 chu trình trong setup()
}

// --- Hàm đếm xung cho ngắt ---
void countPulse() {
  // Xác định chiều quay bằng cách kiểm tra trạng thái Kênh B khi Kênh A mức RISING
  if (digitalRead(ENCODER_B) == LOW) {
    pulseCount++; // Quay chiều 1
  } else {
    pulseCount--; // Quay chiều 2
  }
}

// --- Các hàm điều khiển động cơ ---
void motorForward(int speed) {
  analogWrite(LPWM_PIN, 0);
  analogWrite(RPWM_PIN, speed);
}

void motorBackward(int speed) {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, speed);
}

void motorStop() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}