#include <bluefruit.h>
#include <Wire.h>
#include "ICM42688.h"
#include "MadgwickAHRS.h" // 6軸センサーフュージョン

// センサーと通信のオブジェクト
// センサーと通信のオブジェクト
ICM42688 IMU(Wire, 0x68);

// Custom 128-bit UUIDs for VeloBar
// Service: 19B10010-E8F2-537E-4F6C-D104768A1214
const uint8_t UUID128_SVC[] = {
    0x14, 0x12, 0x8A, 0x76, 0x04, 0xD1, 0x6C, 0x4F,
    0x7E, 0x53, 0xF2, 0xE8, 0x10, 0x00, 0xB1, 0x19
};
// Characteristic: 19B10011-E8F2-537E-4F6C-D104768A1214
const uint8_t UUID128_CHR[] = {
    0x14, 0x12, 0x8A, 0x76, 0x04, 0xD1, 0x6C, 0x4F,
    0x7E, 0x53, 0xF2, 0xE8, 0x11, 0x00, 0xB1, 0x19
};

BLEService        vbtService = BLEService(UUID128_SVC);
BLECharacteristic vbtCharacteristic = BLECharacteristic(UUID128_CHR);
Madgwick          filter;

float velocity = 0.0;
unsigned long lastUpdate = 0;

// 50Hz (20ms) Update Rate for smoother graphs and better peak capture
const unsigned long BLE_INTERVAL_MS = 20;
unsigned long lastBleTime = 0;

// 静止判定カウンター（グローバルスコープで正しく管理）
int zupt_static_frames = 0;

// Madgwickフィルター実効サンプリングレート計測用
unsigned long loopCount = 0;
unsigned long loopRateTimer = 0;
float effectiveSampleRate = 100.0f; // 初期値

// Onboard LED for status (Using Red as per user request)
#define LED_HEARTBEAT LED_RED

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

  Serial.println("--- VeloBar 6DOF Fusion Ver ---");

  // --- Watchdog Timer (WDT) 設定 ---
  // 5秒間プログラムが止まったら自動的にリセットをかけます
  NRF_WDT->CONFIG         = 0x01;     // Stop WDT when sleeping
  NRF_WDT->CRV            = 5 * 32768; // 5 seconds
  NRF_WDT->RREN           = 0x01;     // Enable reload register 0
  NRF_WDT->TASKS_START    = 1;        // Start WDT

  if (IMU.begin() < 0) {
    Serial.println("❌ Sensor Error");
  } else {
    IMU.setAccelFS(ICM42688::gpm16);
    IMU.setGyroFS(ICM42688::dps2000); // ジャイロスコープのフルスケール設定
    
    // 静止状態で少し待機し、IMUを安定させる
    delay(100); 
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
    
    // フィルタ初期化
    // 実効サンプリングレートはloop()内で動的に計測・更新する
    filter.begin(100.0f);
  }

  // Bluetooth設定
  Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);
  Bluefruit.begin();
  Bluefruit.setTxPower(4); 
  Bluefruit.setName("VeloBar"); // Naming Updated!
  
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
  // --- 1. WDTのリフレッシュ ---
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;

  // --- 2. ステータスLEDの点滅 ---
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 500) {
      lastBlink = millis();
      digitalWrite(LED_HEARTBEAT, !digitalRead(LED_HEARTBEAT));
  }

  unsigned long now_micros = micros();
  unsigned long now_millis = millis();
  
  // dt計算の安定化
  unsigned long elapsed_us = now_micros - lastUpdate;
  lastUpdate = now_micros;
  float dt = elapsed_us / 1000000.0f;
  // 異常値ガード: 0.1ms〜100msの範囲外はデフォルト(5ms)を使う
  if (dt > 0.1f || dt <= 0.0001f) dt = 0.005f;
  
  // 実効サンプリングレートを1秒ごとに計測してMadgwickに反映
  loopCount++;
  if (now_millis - loopRateTimer >= 1000) {
    effectiveSampleRate = (float)loopCount;
    loopCount = 0;
    loopRateTimer = now_millis;
    // Madgwickの内部ゲインをサンプリングレートに合わせて調整
    filter.begin(effectiveSampleRate);
  }

  if (IMU.getAGT() > 0) {
    float ax_g = IMU.accX();
    float ay_g = IMU.accY();
    float az_g = IMU.accZ();
    float gx_dps = IMU.gyrX();
    float gy_dps = IMU.gyrY();
    float gz_dps = IMU.gyrZ();

    // 1. Madgwickフィルター更新 (姿勢推定)
    // dt引数付きのupdateIMUで、実際の経過時間を正確に渡す
    filter.updateIMU(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt);

    // 2. 地球座標系の鉛直加速度を計算
    // クォータニオン(q0, q1, q2, q3)を使ってセンサー座標系の加速度を地球座標系(North, East, Down or similar)に回転
    // Madgwickのqは Earth -> Sensor なので、回転行列を作るか、重力ベクトルを引く
    
    // 重力方向の推定 (センサー座標系での重力方向)
    float q0 = filter.q0, q1 = filter.q1, q2 = filter.q2, q3 = filter.q3;
    float gravity_x = 2.0f * (q1 * q3 - q0 * q2);
    float gravity_y = 2.0f * (q0 * q1 + q2 * q3);
    float gravity_z = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // リニア加速度 (センサー座標系) = 測定加速度 - 重力成分
    float lin_acc_x = ax_g - gravity_x;
    float lin_acc_y = ay_g - gravity_y;
    float lin_acc_z = az_g - gravity_z;

    // 鉛直方向の加速度 (Earth-Z) だけ取り出したい
    // Earth-Z accel = dot product of Linear Accel and Gravity Vector (normalized) in sensor frame
    // 重力ベクトル(gravity_x,y,z)は長さ1なので、これとの内積を取れば鉛直成分
    // ただしMadgwickの座標系定義に注意が必要。通常、重力は下向き(Z or -Z)。
    // ここでは「重力方向」への射影成分を計算します。
    // gravity_x, y, z は「下向き」の単位ベクトル(センサー座標系)
    float vertical_accel_g = (lin_acc_x * gravity_x) + (lin_acc_y * gravity_y) + (lin_acc_z * gravity_z);
    
    // G -> m/s^2
    float vertical_accel_mps2 = vertical_accel_g * 9.80665;

    // --- 3. 静止判定 (Zero Velocity Update) ---
    // ジャイロの動きと加速度の変動の両方を見る
    float gyro_mag = sqrt(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);
    
    // 加速度の大きさ（重力込み）で動作中かどうかも判定
    float acc_mag = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    bool acc_near_1g = (acc_mag > 0.95f && acc_mag < 1.05f); // 重力のみ≒静止
    
    // 静止判定ロジック（高重量対応: 閾値を緩和）
    bool is_static = false;
    // ジャイロ10dps未満 かつ 加速度の乱れが0.25G以内なら静止予備軍
    if (gyro_mag < 10.0f && abs(acc_mag - 1.0f) < 0.25f) {
        zupt_static_frames++;
        if (zupt_static_frames > 25) { 
            is_static = true;
        }
        // 長時間静止している場合は姿勢を強力に補正
        if (zupt_static_frames > 100) {
            // 内部的に重力方向を再学習させる処理に相当
        }
    } else {
        zupt_static_frames = 0;
    }

    // --- 3.5 回転検知 (Rotation Clamp) ---
    // バーベルの回転(Rolling)による誤検知を防ぐ
    // 300dps以上の高速回転は通常の挙上動作ではないとみなす
    // (200dpsではベンチプレスの通常動作でも発動する可能性があったため引き上げ)
    if (gyro_mag > 300.0f) {
        vertical_accel_mps2 = 0;
        // 回転中は速度を穏やかに減衰（0.9→0.95に緩和）
        velocity *= 0.95f;
    }

    // --- 4. 速度積分 ---
    if (is_static) {
        // Soft ZUPT: 速やかに0に減衰させる
        // 加速度そのものも0とみなす
        vertical_accel_mps2 = 0;
        velocity *= 0.8; // 強い減衰
        if (abs(velocity) < 0.01) velocity = 0;
    } else {
        // ノイズしきい値を少し上げ、微振動をカット
        if (abs(vertical_accel_mps2) < 0.04f) vertical_accel_mps2 = 0;

        velocity += vertical_accel_mps2 * dt;
        
        // 速度の自然減衰（リーキー積分）: ドリフトを逃がす
        // 挙上中の0.5〜1.0秒間では影響は軽微だが、10秒以上の放置ドリフトを劇的に抑える
        velocity *= 0.998f; 
    }

    // --- 安全リミット (解除) ---
    // Webアプリ側のグラフ表示のみ±5m/sとする
    // if (velocity > 5.0) velocity = 5.0;
    // if (velocity < -5.0) velocity = -5.0;

    // --- 5. BLE送信 & ログ ---
    if (now_millis - lastBleTime >= BLE_INTERVAL_MS) {
      lastBleTime = now_millis;
      if (Bluefruit.connected()) {
        vbtCharacteristic.notify(&velocity, 4);
      }
      
      // デバッグログ: 1秒ごとに主要パラメータを出力
      static unsigned long lastSerialTime = 0;
      if (now_millis - lastSerialTime >= 1000) {
          lastSerialTime = now_millis;
          Serial.print("V:"); Serial.print(velocity, 3);
          Serial.print(" AccZ:"); Serial.print(vertical_accel_mps2, 2);
          Serial.print(" Gyro:"); Serial.print(gyro_mag, 1);
          Serial.print(" Static:"); Serial.print(is_static ? "Y" : "N");
          Serial.print(" ZFrames:"); Serial.print(zupt_static_frames);
          Serial.print(" dt:"); Serial.print(dt * 1000, 1); // ms表示
          Serial.print(" SR:"); Serial.println(effectiveSampleRate, 0); // Hz
      }
    }
  }
  delay(5); 
}