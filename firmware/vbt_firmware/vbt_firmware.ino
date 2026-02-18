#include <bluefruit.h>
#include <Wire.h>
#include "ICM42688.h"

ICM42688 IMU(Wire, 0x68);
BLEService        vbtService = BLEService(0x180C);
BLECharacteristic vbtCharacteristic = BLECharacteristic(0x2A6E);

float velocity = 0.0;
unsigned long lastUpdate = 0;
float grav_mag = 1.0; 

const unsigned long BLE_INTERVAL_MS = 100;    
unsigned long lastBleTime = 0;

// 接続・切断のイベントハンドラ
void connect_callback(uint16_t conn_handle) {
  Serial.println("🔗 Connected!");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.println("❌ Disconnected. Advertising restarted.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(D1, OUTPUT); digitalWrite(D1, HIGH);
  pinMode(D2, OUTPUT); digitalWrite(D2, LOW);
  pinMode(D3, OUTPUT); digitalWrite(D3, HIGH);
  delay(500);

  Serial.println("--- VBT Device Final Stable Ver ---");

  if (IMU.begin() < 0) {
    Serial.println("❌ Sensor Error");
  } else {
    IMU.setAccelFS(ICM42688::gpm16);
    float sum = 0;
    for(int i=0; i<40; i++) {
        if(IMU.getAGT() > 0) {
            float ax=IMU.accX(), ay=IMU.accY(), az=IMU.accZ();
            sum += sqrt(ax*ax + ay*ay + az*az);
        }
        delay(10);
    }
    grav_mag = sum / 40.0;
  }

  // Bluetooth設定の強化
  Bluefruit.begin();
  Bluefruit.setTxPower(4); 
  Bluefruit.setName("VBT_Device");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  vbtService.begin();
  vbtCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  vbtCharacteristic.setFixedLen(4);
  vbtCharacteristic.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE); 
  Bluefruit.Advertising.addService(vbtService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true); 
  Bluefruit.Advertising.setInterval(32, 244);    
  Bluefruit.Advertising.setFastAdvertisingInterval(30); 
  Bluefruit.Advertising.start(0);                

  Serial.println("🚀 Ready to Connect!");
  lastUpdate = micros();
}

void loop() {
  unsigned long now_micros = micros();
  unsigned long now_millis = millis();
  float dt = (now_micros - lastUpdate) / 1000000.0;
  lastUpdate = now_micros;
  if (dt > 0.1 || dt <= 0) dt = 0;

  if (IMU.getAGT() > 0) {
    float ax = IMU.accX();
    float ay = IMU.accY();
    float az = IMU.accZ();
    
    // 3軸の合成加速度（傾き耐性）
    float current_mag = sqrt(ax*ax + ay*ay + az*az);
    float linear_accel = (current_mag - grav_mag) * 9.80665;

    // 静止判定
    static float last_mag = 1.0;
    float diff = abs(current_mag - last_mag);
    last_mag = current_mag;

    static int stillCount = 0;
    if (diff < 0.02) stillCount++;
    else stillCount = 0;

    // 自動ゼロリセット（0.1秒静止でリセット）
    if (stillCount > 20) {
        grav_mag = grav_mag * 0.9 + current_mag * 0.1;
        velocity = 0;
    }

    // 感度調整（スクワット対応: 0.25 -> 0.15）
    if (abs(linear_accel) < 0.15) linear_accel = 0;

    // 積分（リークを弱めてゆっくりした動きを保持: 0.99 -> 0.998）
    velocity = (velocity + linear_accel * dt) * 0.998;

    // リミッター
    if (velocity > 4.0) velocity = 4.0;
    if (velocity < -4.0) velocity = -4.0;

    // 送信
    if (now_millis - lastBleTime >= BLE_INTERVAL_MS) {
      lastBleTime = now_millis;
      if (Bluefruit.connected()) {
        vbtCharacteristic.notify(&velocity, 4);
      }
      Serial.print("Acc:"); Serial.print(linear_accel, 2);
      Serial.print(" | Vel:"); Serial.println(velocity, 2);
    }
  }
  delay(5); 
}