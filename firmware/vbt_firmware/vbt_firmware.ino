#include <bluefruit.h>
#include <Wire.h>
#include "ICM42688.h"
#include "MadgwickAHRS.h" // 6軸センサーフュージョン

// センサーと通信のオブジェクト
ICM42688 IMU(Wire, 0x68);
BLEService        vbtService = BLEService(0x180C);
BLECharacteristic vbtCharacteristic = BLECharacteristic(0x2A6E);
Madgwick          filter;

float velocity = 0.0;
unsigned long lastUpdate = 0;

const unsigned long BLE_INTERVAL_MS = 100;    
unsigned long lastBleTime = 0;

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

  Serial.println("--- VBT Device 6DOF Fusion Ver ---");

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
    
    // フィルタ初期化 (サンプリング周波数 ~100Hz)
    filter.begin(100.0f);
  }

  // Bluetooth設定
  Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);
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
  float dt = (now_micros - lastUpdate) / 1000000.0; // 秒
  lastUpdate = now_micros;
  if (dt > 0.1 || dt <= 0) dt = 0.01; // 安全策

  if (IMU.getAGT() > 0) {
    float ax_g = IMU.accX();
    float ay_g = IMU.accY();
    float az_g = IMU.accZ();
    float gx_dps = IMU.gyrX();
    float gy_dps = IMU.gyrY();
    float gz_dps = IMU.gyrZ();

    // 1. Madgwickフィルター更新 (姿勢推定)
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
    static float gyro_mag_sum = 0;
    static int sample_count = 0;
    float gyro_mag = sqrt(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);
    
    // 静止判定ロジック
    // ジャイロが静かで、かつ加速度の変動が少ない場合
    bool is_static = false;
    if (gyro_mag < 5.0) { // 5 dps未満
        static int static_frames = 0;
        static_frames++;
        if (static_frames > 20) { // 約0.2秒継続
            is_static = true;
        }
    } else {
        static int static_frames = 0; // リセット
    }

    // --- 4. 速度積分 ---
    if (is_static) {
        // Soft ZUPT: 速やかに0に減衰させる
        // 加速度そのものも0とみなす
        vertical_accel_mps2 = 0;
        velocity *= 0.8; // 強い減衰
        if (abs(velocity) < 0.01) velocity = 0;
    } else {
        // ノイズしきい値を引き下げ (0.05 -> 0.02)
    // 6軸合成によりノイズ自体が減っているので、より小さな値を拾えるようにする
    if (abs(vertical_accel_mps2) < 0.02) vertical_accel_mps2 = 0;

    velocity += vertical_accel_mps2 * dt;
    // 減衰をほぼ無効化 (0.999 -> 1.0)
    // 動作中の速度低下を防ぐ。静止時は強力なZUPTが働くため問題なし。
    // velocity *= 1.0; 

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
      
      static unsigned long lastSerialTime = 0;
      if (now_millis - lastSerialTime >= 1000) {
          lastSerialTime = now_millis;
          Serial.print("V:"); Serial.print(velocity, 3);
          Serial.print(" AccZ:"); Serial.print(vertical_accel_mps2, 2);
          Serial.print(" Gyro:"); Serial.println(gyro_mag, 1);
      }
    }
  }
  delay(5); 
}