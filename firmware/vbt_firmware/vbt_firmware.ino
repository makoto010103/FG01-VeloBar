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
  
  // 接続後に通信パラメータを最適化 (11.25ms 〜 30ms)
  // これにより、スマホ側がデータを要求する頻度が上がり、バッファ詰まりを防ぎます
  Bluefruit.Periph.setConnInterval(9, 24);
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

  Serial.println("--- VBT Device Ultra-Stable Ver ---");

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

  // Bluetooth設定の最適化
  Bluefruit.begin();
  Bluefruit.setTxPower(4); 
  Bluefruit.setName("VBT_Device");
  
  // MTU（一度に送れるデータ量）を最大にリクエスト
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  vbtService.begin();
  vbtCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  vbtCharacteristic.setFixedLen(4);
  vbtCharacteristic.begin();

  // アドバタイズ設定
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE); 
  Bluefruit.Advertising.addService(vbtService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true); 
  Bluefruit.Advertising.setInterval(32, 244);    
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
    
    float current_mag = sqrt(ax*ax + ay*ay + az*az);
    float linear_accel = (current_mag - grav_mag) * 9.80665;

    static float last_mag = 1.0;
    float diff = abs(current_mag - last_mag);
    last_mag = current_mag;

    static int stillCount = 0;
    if (diff < 0.02) stillCount++;
    else stillCount = 0;

    if (stillCount > 20) {
        grav_mag = grav_mag * 0.9 + current_mag * 0.1;
        velocity = 0;
    }

    if (abs(linear_accel) < 0.15) linear_accel = 0;
    velocity = (velocity + linear_accel * dt) * 0.998;

    if (velocity > 4.0) velocity = 4.0;
    if (velocity < -4.0) velocity = -4.0;

    // 通信処理
    if (now_millis - lastBleTime >= BLE_INTERVAL_MS) {
      lastBleTime = now_millis;
      if (Bluefruit.connected()) {
        // notifyが成功したかチェック（失敗＝バッファ詰まりの可能性）
        if (!vbtCharacteristic.notify(&velocity, 4)) {
           // 失敗時はコンソールで警告
           // Serial.println("BLE Notify Full!");
        }
      }
      
      // シリアル出力の頻度を下げて負荷を軽減 (1秒に1回程度に制限)
      static unsigned long lastSerialTime = 0;
      if (now_millis - lastSerialTime >= 1000) {
          lastSerialTime = now_millis;
          Serial.print("V:"); Serial.print(velocity, 2);
          Serial.print(" G:"); Serial.println(grav_mag, 3);
      }
    }
  }
  delay(5); 
}