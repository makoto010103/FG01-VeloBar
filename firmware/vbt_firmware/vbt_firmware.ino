#include <bluefruit.h>
#include <Wire.h>
#include "ICM42688.h"

const int PIN_VDDIO = D1;
const int PIN_AD0 = D2;
const int PIN_CS = D3;

ICM42688 IMU(Wire, 0x68);
BLEService        vbtService = BLEService(0x180C);
BLECharacteristic vbtCharacteristic = BLECharacteristic(0x2A6E);

float velocity = 0.0;
unsigned long lastUpdate = 0;
float grav_offset = 0.0;

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_VDDIO, OUTPUT); digitalWrite(PIN_VDDIO, HIGH);
  pinMode(PIN_AD0, OUTPUT);    digitalWrite(PIN_AD0, LOW);
  pinMode(PIN_CS, OUTPUT);     digitalWrite(PIN_CS, HIGH);
  delay(500);

  Serial.println("--- VBT Device Starting ---");

  if (IMU.begin() < 0) {
    Serial.println("❌ Sensor Error: Check Wiring!");
  } else {
    Serial.println("✅ Sensor Found!");
    IMU.setAccelFS(ICM42688::gpm16);
    
    Serial.println("CALIBRATING... STAY STILL");
    float sum_z = 0;
    for(int i=0; i<100; i++) {
        if (IMU.getAGT() > 0) sum_z += IMU.accZ();
        delay(10);
    }
    grav_offset = sum_z / 100.0;
    Serial.print("Zero Point fixed at: "); Serial.println(grav_offset, 3);
  }

  Bluefruit.begin();
  Bluefruit.setName("VBT_Device");
  vbtService.begin();
  vbtCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  vbtCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  vbtCharacteristic.setFixedLen(4);
  vbtCharacteristic.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(vbtService);
  Bluefruit.Advertising.start(0);

  Serial.println("🚀 System Ready! Start Lifting!");
  lastUpdate = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0;
  lastUpdate = now;

  // dtが異常に長い場合（接続処理などで止まっていた場合）は計算をスキップ
  if (dt > 0.1) {
    dt = 0;
  }

  if (IMU.getAGT() > 0) {
    float acc_z = IMU.accZ();
    float linear_accel = (acc_z - grav_offset) * 9.80665; 

    // ノイズ除去（0.15m/s^2未満は無視）
    if (abs(linear_accel) < 0.15) linear_accel = 0;
    
    velocity += linear_accel * dt;

    // --- 安全装置：速度リミッター（人間が出せる速度ではない場合カット） ---
    if (velocity > 5.0) velocity = 5.0;
    if (velocity < -5.0) velocity = -5.0;

    // ドリフト防止：加速度が0なら、速度を徐々に0に戻す
    if (linear_accel == 0) velocity *= 0.95;
    
    // 完全に停止させる閾値
    if (abs(velocity) < 0.02) velocity = 0;

    static int counter = 0;
    counter++;
    
    // --- 1. Bluetooth送信（100msに1回 = 10Hz） ---
    // ここが速すぎるとWeb側がパンクしてグラフが描画されなくなる
    if (counter % 20 == 0) {
      if (Bluefruit.connected()) {
        vbtCharacteristic.notify(&velocity, 4);
      }
    }

    // --- 2. シリアル表示（デバッグ用） ---
    if (counter % 50 == 0) {
      Serial.print("RawZ:"); Serial.print(acc_z, 2);
      Serial.print(" | Vel:"); Serial.println(velocity, 3);
      counter = 0;
    }
  }
  delay(5); 
}


