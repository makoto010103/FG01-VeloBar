#include <bluefruit.h>
#include <Wire.h>
#include "ICM42688.h"

// センサーと通信のオブジェクト
ICM42688 IMU(Wire, 0x68);
BLEService        vbtService = BLEService(0x180C);
BLECharacteristic vbtCharacteristic = BLECharacteristic(0x2A6E);

float velocity = 0.0;
unsigned long lastUpdate = 0;
float grav_mag = 1.0; 

const unsigned long BLE_INTERVAL_MS = 100;    
unsigned long lastBleTime = 0;

// Onboard LED for status (Red on XIAO nRF52840 is usually D6/LED_RED)
#define LED_HEARTBEAT LED_BLUE

// 接続・切断のイベントハンドラ
void connect_callback(uint16_t conn_handle) {
  Serial.println("🔗 Connected!");
  // 接続パラメータの最適化
  Bluefruit.Periph.setConnInterval(9, 24);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.println("❌ Disconnected. Advertising restarted.");
}

void setup() {
  // ステータスLED設定
  pinMode(LED_HEARTBEAT, OUTPUT);
  digitalWrite(LED_HEARTBEAT, HIGH); // 消灯(Negative Logic)

  Serial.begin(115200);
  // 起動時の初期待ちを少し長くして安定させる
  for(int i=0; i<10; i++) { delay(200); digitalWrite(LED_HEARTBEAT, i%2); }
  digitalWrite(LED_HEARTBEAT, HIGH);

  pinMode(D1, OUTPUT); digitalWrite(D1, HIGH);
  pinMode(D2, OUTPUT); digitalWrite(D2, LOW);
  pinMode(D3, OUTPUT); digitalWrite(D3, HIGH);
  delay(500);

  Serial.println("--- VBT Device Immortal Ver ---");

  // --- Watchdog Timer (WDT) 設定 ---
  // 5秒間プログラムが止まったら自動的にリセットをかけます
  NRF_WDT->CONFIG         = 0x01;     // Stop WDT when sleeping
  NRF_WDT->CRV            = 5 * 32768; // 5 seconds (Clock is 32.768kHz)
  NRF_WDT->RREN           = 0x01;     // Enable reload register 0
  NRF_WDT->TASKS_START    = 1;        // Start WDT

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
        NRF_WDT->RR[0] = WDT_RR_RR_Reload; // WDTをリフレッシュ
        delay(10);
    }
    grav_mag = sum / 40.0;
    Serial.print("Initial Gravity: "); Serial.println(grav_mag, 3);
  }

  // Bluetooth設定 (configはbeginの前に呼ぶ必要がある)
  Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH); // 極端なMAXより安定を取ってHIGHに設定
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
  Bluefruit.Advertising.start(0);                

  Serial.println("🚀 Ready to Connect!");
  lastUpdate = micros();
}

void loop() {
  // --- 1. WDTのリフレッシュ (生きてるアピール) ---
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;

  // --- 2. ステータスLEDの点滅 (Hearbeat) ---
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 500) {
      lastBlink = millis();
      digitalWrite(LED_HEARTBEAT, !digitalRead(LED_HEARTBEAT));
  }

  unsigned long now_micros = micros();
  unsigned long now_millis = millis();
  float dt = (now_micros - lastUpdate) / 1000000.0;
  lastUpdate = now_micros;
  if (dt > 0.1 || dt <= 0) dt = 0;

  if (IMU.getAGT() > 0) {
    float ax=IMU.accX(), ay=IMU.accY(), az=IMU.accZ();
    float current_mag = sqrt(ax*ax + ay*ay + az*az);
    float linear_accel = (current_mag - grav_mag) * 9.80665;

    static float last_mag = 1.0;
    float diff = abs(current_mag - last_mag);
    last_mag = current_mag;

    static int stillCount = 0;
    if (diff < 0.015) stillCount++; // より厳密な静止判定
    else stillCount = 0;

    // 静止判定のしきい値を延長 (20 -> 150: 約0.75〜1秒の完全静止が必要)
    if (stillCount > 150) {
        // 重力値の更新をより穏やかに (0.1 -> 0.02)
        grav_mag = grav_mag * 0.98 + current_mag * 0.02;
        velocity = 0;
    }

    // ノイズしきい値を引き下げ (0.15 -> 0.05)
    if (abs(linear_accel) < 0.05) linear_accel = 0;
    velocity = (velocity + linear_accel * dt) * 0.998;

    if (velocity > 4.0) velocity = 4.0;
    if (velocity < -4.0) velocity = -4.0;

    if (now_millis - lastBleTime >= BLE_INTERVAL_MS) {
      lastBleTime = now_millis;
      if (Bluefruit.connected()) {
        vbtCharacteristic.notify(&velocity, 4);
      }
      
      // シリアル出力 (1秒に1回)
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