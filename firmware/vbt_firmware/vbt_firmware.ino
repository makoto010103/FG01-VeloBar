#include <bluefruit.h>
#include <Wire.h>
#include "ICM42688.h"

// センサーの制御用ピン
const int PIN_VDDIO = D1;
const int PIN_AD0 = D2;
const int PIN_CS = D3;

// I2Cのアドレス設定
ICM42688 IMU(Wire, 0x68);

// BLEの定義 (Bluefruit用)
BLEService        vbtService = BLEService(0x180C);
BLECharacteristic vbtCharacteristic = BLECharacteristic(0x2A6E);

// 計算用変数
float velocity = 0.0;
unsigned long lastUpdate = 0;
float grav_offset = 0.0; // キャリブレーションで決定

void setup() {
  Serial.begin(115200);

  // ピン初期化
  pinMode(PIN_VDDIO, OUTPUT); digitalWrite(PIN_VDDIO, HIGH);
  pinMode(PIN_AD0, OUTPUT);    digitalWrite(PIN_AD0, LOW);
  pinMode(PIN_CS, OUTPUT);     digitalWrite(PIN_CS, HIGH);
  delay(200);

  // センサー初期化
  if (IMU.begin() < 0) {
    Serial.println("IMU initialization failed");
    while (1) delay(1000);
  }
  IMU.setAccelFS(ICM42688::gpm16);
  IMU.setGyroFS(ICM42688::dps2000);

  // --- キャリブレーション (静止状態で3秒待機) ---
  Serial.println("CALIBRATING... KEEP THE DEVICE STILL!");
  float sum_z = 0;
  int samples = 100;
  for(int i=0; i<samples; i++) {
    if (IMU.getAGT() > 0) {
      sum_z += IMU.accZ();
    }
    delay(10);
  }
  grav_offset = sum_z / (float)samples;
  Serial.print("Calibration Done. Offset: "); Serial.println(grav_offset, 3);
  // ------------------------------------------

  // Bluefruit BLE初期化
  Bluefruit.begin();
  Bluefruit.setName("VBT_Device");
  vbtService.begin();
  vbtCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  vbtCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  vbtCharacteristic.setFixedLen(4);
  vbtCharacteristic.begin();
  startAdv();

  Serial.println("VBT Device Ready. Tracking movement...");
  lastUpdate = micros();
}

void startAdv(void) {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(vbtService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0;
  lastUpdate = now;

  if (IMU.getAGT() > 0) {
    float az = IMU.accZ();
    
    // 計測された重力オフセットを引いて、純粋な動き(m/s^2)に変換
    float linear_accel = (az - grav_offset) * 9.80665; 

    // ノイズ除去（ここを調整して感度を変えます）
    if (abs(linear_accel) < 0.25) linear_accel = 0;

    // 速度積分
    velocity += linear_accel * dt;

    // ドリフト防止
    if (linear_accel == 0) velocity *= 0.85; // 静止時は急速に0へ戻す
    if (abs(velocity) < 0.05) velocity = 0;

    // BLEで送信
    if (Bluefruit.connected()) {
      vbtCharacteristic.notify(&velocity, 4);
    }

    // デバッグ出力
    Serial.print("Acc:"); Serial.print(linear_accel, 2);
    Serial.print(" Vel:"); Serial.println(velocity, 2);
  }

  delay(10);
}

