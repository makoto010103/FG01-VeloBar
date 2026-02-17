#include <ArduinoBLE.h>
#include <Wire.h>
#include "ICM42688.h"

// センサーの制御用ピン
const int PIN_VDDIO = D1;
const int PIN_AD0 = D2;
const int PIN_CS = D3;

// I2Cのアドレス設定
ICM42688 IMU(Wire, 0x68);

// BLEの定義
BLEService vbtService("180C"); // 独自のサービスID
BLEFloatCharacteristic velocityCharacteristic("2A6E", BLERead | BLENotify);

// 計算用変数
float velocity = 0.0;
unsigned long lastUpdate = 0;
float grav_offset = 0.98; // 初期値（後でキャリブレーション可能）

void setup() {
  Serial.begin(115200);
  // while (!Serial); // スタンドアロン動作を考慮してコメントアウト

  // ピン初期化
  pinMode(PIN_VDDIO, OUTPUT); digitalWrite(PIN_VDDIO, HIGH);
  pinMode(PIN_AD0, OUTPUT);    digitalWrite(PIN_AD0, LOW);
  pinMode(PIN_CS, OUTPUT);     digitalWrite(PIN_CS, HIGH);
  delay(200);

  // センサー初期化
  if (IMU.begin() < 0) {
    Serial.println("IMU initialization failed");
    while (1);
  }
  IMU.setAccelFS(ICM42688::gpm16);
  IMU.setGyroFS(ICM42688::dps2000);

  // BLE初期化
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed");
    while (1);
  }

  BLE.setLocalName("VBT_Device");
  BLE.setAdvertisedService(vbtService);
  vbtService.addCharacteristic(velocityCharacteristic);
  BLE.addService(vbtService);
  velocityCharacteristic.writeValue(0.0);
  BLE.advertise();

  Serial.println("VBT Device Ready. Waiting for BLE connection...");
  lastUpdate = micros();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      unsigned long now = micros();
      float dt = (now - lastUpdate) / 1000000.0;
      lastUpdate = now;

      if (IMU.getAGT() > 0) {
        // 加速度Z軸（垂直方向）を使用
        // 本来はクォータニオン等で傾き補正が必要ですが、まずは垂直固定前提で実装
        float az = IMU.accZ();
        
        // 重力を引いて純粋な「動いた加速度」を出す
        // センサーの向きによって 1.0 または -1.0 を調整
        float linear_accel = (az + 1.0) * 9.80665; // gからm/s^2へ変換 (Z=-1.0g時が静止と仮定)

        // ノイズ除去（デッドバンド）
        if (abs(linear_accel) < 0.2) linear_accel = 0;

        // 加速度を積分して速度を出す
        velocity += linear_accel * dt;

        // ドリフト対策：加速度が小さくなったら速度を徐々に0付近へ戻す
        if (linear_accel == 0) velocity *= 0.95; 
        if (abs(velocity) < 0.05) velocity = 0;

        // BLEで送信
        velocityCharacteristic.writeValue(velocity);

        // デバッグ表示
        Serial.print("Accel:"); Serial.print(linear_accel);
        Serial.print(" Vel:"); Serial.println(velocity);
      }
      delay(10); // 約100Hzでサンプリング
    }
    Serial.println("Disconnected from central");
    velocity = 0;
  }
}
