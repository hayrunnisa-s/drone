#include <Wire.h>

// --- AYARLAR ---
#define UCUS_GUCU 160      // (0-255 arası) Drone'u kaldıracak minimum güç. Bunu deneyerek bulmalısın!
                           // 160 ile başla, kalkmazsa 170, 180 diye artır. Çok artırma tavana yapışır!
#define UCUS_SURESI 3000   // Kaç milisaniye havada kalsın? (3000ms = 3 saniye)

// --- PIN TANIMLAMALARI (ESP32) ---
// Motorları L9110S'e bağladığın pinler [cite: 60]
const int motorSolOn_Pin = 13; 
const int motorSagOn_Pin = 12;
const int motorSolArka_Pin = 14; 
const int motorSagArka_Pin = 27;

// PWM Ayarları (ESP32 için gerekli)
const int frekans = 5000;
const int cozunurluk = 8;
const int kanal_1 = 0;
const int kanal_2 = 1;
const int kanal_3 = 2;
const int kanal_4 = 3;

// --- DEĞİŞKENLER ---
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float elapsedTime, timeUtils, timePrev;
int i;
float rad_to_deg = 180 / 3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

// PID KATSAYILARI (Denge Ayarları)
// Eğer drone çok titriyorsa P'yi azalt. Tepki vermiyorsa artır.
double kp = 3.55;
double ki = 0.003;
double kd = 2.05;

unsigned long baslangicZamani;
bool ucusBasladi = false;

void setup() {
  Wire.begin(); // SDA: 21, SCL: 22 (ESP32 Standart) [cite: 63]
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);

  // Motor Pin Kurulumu (ESP32 ledcSetup)
  ledcSetup(kanal_1, frekans, cozunurluk);
  ledcSetup(kanal_2, frekans, cozunurluk);
  ledcSetup(kanal_3, frekans, cozunurluk);
  ledcSetup(kanal_4, frekans, cozunurluk);

  ledcAttachPin(motorSolOn_Pin, kanal_1);
  ledcAttachPin(motorSagOn_Pin, kanal_2);
  ledcAttachPin(motorSolArka_Pin, kanal_3);
  ledcAttachPin(motorSagArka_Pin, kanal_4);

  // Motorları durdur
  motorDurdur();

  Serial.println("Kalibrasyon yapiliyor... Lutfen dokunmayin (5 sn)");
  delay(5000); 
  baslangicZamani = millis();
  Serial.println("--- UCUS BASLIYOR ---");
}

void loop() {
  // --- ZAMAN YÖNETİMİ ---
  timePrev = timeUtils;
  timeUtils = millis();
  elapsedTime = (timeUtils - timePrev) / 1000;

  // --- SENSÖR OKUMA (MPU6050) [cite: 70] ---
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);
  Acc_rawX = Wire.read() << 8 | Wire.read();
  Acc_rawY = Wire.read() << 8 | Wire.read();
  Acc_rawZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);
  Gyr_rawX = Wire.read() << 8 | Wire.read();
  Gyr_rawY = Wire.read() << 8 | Wire.read();

  // Açı Hesaplama
  Acceleration_angle[0] = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;
  Acceleration_angle[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;

  Gyro_angle[0] = Gyr_rawX / 131.0;
  Gyro_angle[1] = Gyr_rawY / 131.0;

  // Tamamlayıcı Filtre (Sensör verilerini birleştirme)
  Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.02 * Acceleration_angle[0];
  Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1] * elapsedTime) + 0.02 * Acceleration_angle[1];

  // --- BASIT PID HESAPLAMA (Sadece X ekseni/Roll dengesi için örnek) ---
  // Drone'un yatay kalmasını hedefler (Hedef Açı = 0)
  error = Total_angle[0] - 0; 
  pid_p = kp * error;
  if (-3 < error && error < 3) { pid_i = pid_i + (ki * error); }
  pid_d = kd * ((error - previous_error) / elapsedTime);
  PID = pid_p + pid_i + pid_d;
  
  // PID Sınırlandırma
  if (PID < -50) PID = -50;
  if (PID > 50) PID = 50;

  previous_error = error;

  // --- UÇUŞ SENARYOSU (10 CM UÇUŞ) [cite: 80] ---
  unsigned long gecenSure = millis() - baslangicZamani;

  if (gecenSure < 2000) {
    // İlk 2 saniye bekle
    motorDurdur();
  } 
  else if (gecenSure < (2000 + UCUS_SURESI)) {
    // UÇUŞ MODU: Belirlenen sürede motorlara güç ver
    // PID değerini motorlara dağıt (Dengeleme)
    // Sol motorlar +PID, Sağ motorlar -PID (veya tam tersi, denemen lazım)
    
    int motorSolHiz = UCUS_GUCU + PID;
    int motorSagHiz = UCUS_GUCU - PID;

    // Hızları 0-255 arasında tut
    motorSolHiz = constrain(motorSolHiz, 0, 255);
    motorSagHiz = constrain(motorSagHiz, 0, 255);

    motorYaz(motorSolHiz, motorSagHiz, motorSolHiz, motorSagHiz); // Basitçe ön/arka ayrımı yapmadan sağ/sol dengesi
  } 
  else {
    // Süre bitti, İNİŞ YAP
    motorDurdur();
    // Güvenlik: Eğer drone çok yan yatarsa (30 derece) motorları kapat
    if(abs(Total_angle[0]) > 30 || abs(Total_angle[1]) > 30) {
       motorDurdur();
    }
  }
}

void motorYaz(int solOn, int sagOn, int solArka, int sagArka) {
  ledcWrite(kanal_1, solOn);
  ledcWrite(kanal_2, sagOn);
  ledcWrite(kanal_3, solArka);
  ledcWrite(kanal_4, sagArka);
}

void motorDurdur() {
  ledcWrite(kanal_1, 0);
  ledcWrite(kanal_2, 0);
  ledcWrite(kanal_3, 0);
  ledcWrite(kanal_4, 0);
}