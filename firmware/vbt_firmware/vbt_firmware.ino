#include <bluefruit.h>
#include <Wire.h>
#include "ICM42688.h"

ICM42688 IMU(Wire, 0x68);
BLEService        vbtService = BLEService(0x180C);
BLECharacteristic vbtCharacteristic = BLECharacteristic(0x2A6E);

float velocity = 0.0;
unsigned long lastUpdate = 0;
float grav_offset = 1.0; // 初期値

const unsigned long BLE_INTERVAL_MS = 100; 
unsigned long lastBleTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(D1, OUTPUT); digitalWrite(D1, HIGH); // VDDIO
  pinMode(D2, OUTPUT); digitalWrite(D2, LOW);  // AD0
  pinMode(D3, OUTPUT); digitalWrite(D3, HIGH); // CS
  delay(500);

  if (IMU.begin() < 0) {
    Serial.println("❌ Sensor Error");
  } else {
    IMU.setAccelFS(ICM42688::gpm16);
    // 初回の簡易的なゼロ点合わせ
    float sum = 0;
    for(int i=0; i<50; i++) { sum += IMU.accZ(); delay(5); }
    grav_offset = sum / 50.0;
  }

  Bluefruit.begin();
  Bluefruit.setName("VBT_Device");
  vbtService.begin();
  vbtCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  vbtCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  vbtCharacteristic.setFixedLen(4);
  vbtCharacteristic.begin();
  Bluefruit.Advertising.addService(vbtService);
  Bluefruit.Advertising.start(0);

  lastUpdate = micros();
}

void loop() {
  unsigned long now_micros = micros();
  unsigned long now_millis = millis();
  float dt = (now_micros - lastUpdate) / 1000000.0;
  lastUpdate = now_micros;
  if (dt > 0.1 || dt <= 0) dt = 0;

  if (IMU.getAGT() > 0) {
    float acc_z = IMU.accZ();
    
    // --- アダプティブ重力補正 ---
    // ほとんど動いていないときは、今の値を重力オフセットとしてジワジワ取り込む（傾き対策）
    static float lpf_acc = 1.0;
    lpf_acc = lpf_acc * 0.98 + acc_z * 0.02; 
    
    float linear_accel = (acc_z - grav_offset) * 9.80665;

    // ノイズカット
    if (abs(linear_accel) < 0.3) {
        linear_accel = 0;
        // 静止中にゼロ点を微調整
        grav_offset = grav_offset * 0.9 + acc_z * 0.1;
    }
    
    velocity += linear_accel * dt;

    // 強力なドリフト防止
    if (linear_accel == 0) {
      velocity *= 0.85; // 急ブレーキ
      if (abs(velocity) < 0.02) velocity = 0;
    }

    // リミッター
    if (velocity > 4.0) velocity = 4.0;
    if (velocity < -4.0) velocity = -4.0;

    // Bluetooth送信
    if (now_millis - lastBleTime >= BLE_INTERVAL_MS) {
      lastBleTime = now_millis;
      if (Bluefruit.connected()) {
        vbtCharacteristic.notify(&velocity, 4);
      }
      // シリアルにも簡潔に出す
      Serial.print("G:"); Serial.print(grav_offset, 3);
      Serial.print(" | V:"); Serial.println(velocity, 2);
    }
  }
  delay(5);
}