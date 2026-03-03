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
BLEBas            blebas; // Standard Battery Service
Madgwick          filter;

float velocity = 0.0;
unsigned long lastUpdate = 0;

// 33Hz (30ms) Update Rate: スマホのWeb Bluetoothが過負荷でパケットロス（数十秒のフリーズ）を起こすのを防ぐための最適な間隔
const unsigned long BLE_INTERVAL_MS = 30;
unsigned long lastBleTime = 0;

// 【新規追加】 ZUPT連動型オフセット（ゼロ点）自動補正用変数
float accel_offset_z = 0.0f;     // 学習・ロックされた鉛直加速度のノイズ（オフセット）
float offset_accumulator = 0.0f; // 静止中の移動平均計算用アキュムレータ
int offset_sample_count = 0;     // 静止中のサンプル数

// 静止判定カウンター（グローバルスコープで正しく管理）
int zupt_static_frames = 0;

// Madgwickフィルター実効サンプリングレート計測用
unsigned long loopCount = 0;
unsigned long loopRateTimer = 0;
float effectiveSampleRate = 100.0f; // 初期値

// Onboard LED for status (Using Red as per user request)
#define LED_HEARTBEAT LED_RED

// Battery Monitoring
// PIN_VBAT is already defined by the Seeed nRF52840 board package as (32)
unsigned long lastBatteryCheck = 0;
int currentBatteryLevel = 100;

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

  // Battery Voltage Divider Enable Pin
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, HIGH); // Disable voltage divider by default to save power

  Serial.begin(115200);
  // 起動時の初期待ちを少し長くして安定させる
  for(int i=0; i<10; i++) { delay(200); digitalWrite(LED_HEARTBEAT, i%2); }
  digitalWrite(LED_HEARTBEAT, HIGH);

  // Battery ADC Resolution
  analogReference(AR_INTERNAL_2_4); // Set reference to 2.4V for nRF52840 (Seeed recommended)
  analogReadResolution(12);     // 12-bit ADC

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

  // Battery Service Begin
  blebas.begin();
  blebas.write(100); // Initial dummy read

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

  // --- 2.5 バッテリー残量の計測と送信 (5秒ごと) ---
  if (millis() - lastBatteryCheck > 5000) {
      lastBatteryCheck = millis();
      
      // XIAO nRF52840 Sense uses P0.31 with a voltage divider (1M / 510K)
      // To read it, we must pull VBAT_ENABLE (pin 14) LOW
      digitalWrite(VBAT_ENABLE, LOW);
      delay(10); // Wait for voltage to settle (high impedance divider needs more time)
      
      // Calculate voltage (ADC is 12-bit, reference is 2.4V)
      // Take multiple samples to reduce noise
      int adc_sum = 0;
      for (int i = 0; i < 8; i++) {
          adc_sum += analogRead(PIN_VBAT);
      }
      float adc_avg = adc_sum / 8.0f;
      
      float vbat_mv = adc_avg * (2400.0f / 4096.0f) * 2.9607f;
      
      // Turn off voltage divider to save power
      digitalWrite(VBAT_ENABLE, HIGH);
      
      // LiPo conversion mapping: 4.2V(100%), 3.7V(50%), 3.3V(0%)
      float battery_pct = 0;
      if (vbat_mv >= 4150) battery_pct = 100;
      else if (vbat_mv > 3300) {
          battery_pct = 100.0f * ((vbat_mv - 3300.0f) / (4150.0f - 3300.0f));
      } else {
          battery_pct = 0;
      }
      
      if (battery_pct > 100) battery_pct = 100;
      if (battery_pct < 0) battery_pct = 0;
      
      currentBatteryLevel = (int)battery_pct;
      if (Bluefruit.connected()) {
          blebas.write(currentBatteryLevel);
      }
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
    // ただしMadgwickの座標系定義に注意が必要。
    float vertical_accel_g = (lin_acc_x * gravity_x) + (lin_acc_y * gravity_y) + (lin_acc_z * gravity_z);

    // G -> m/s^2
    float vertical_accel_mps2 = vertical_accel_g * 9.80665;

    // --- 3. 静止判定 (Zero Velocity Update) ---
    // ジャイロの動きと加速度の変動の両方を見る
    float gyro_mag = sqrt(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);
    
    // 加速度の大きさ（重力込み）で動作中かどうかも判定
    float acc_mag = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    bool acc_near_1g = (acc_mag > 0.97f && acc_mag < 1.03f); // 重力のみ≒静止
    
    // 静止判定ロジック
    bool is_static = false;
    // v3.7: 「速度連動ZUPT」方式
    // 問題1: gyro<3.0dpsは厳しすぎて、レップ間の休撓中（呼吸・体の揺れで>3dps）にZUPTが発動せず、速度が0.5〜0.9m/sに張り付くが発生。
    // 問題2: gyro<15.0dpsに戻すと、スクワット中の滑らかな加速（0.01G）でZUPTが発動して挙上中の速度を殺す。
    // 解決策: gyro<15.0dpsに戻し、だが「velocityが既に小さい時だけこのクランプを発動」する。
    // 具体的に: velocity < 0.15m/sの時のみZUPTが速度を殺す。
    // 挙上中(velocity=1.0m/s等)はたとえZUPT条件を満たしても殺さない。
    if (acc_near_1g && gyro_mag < 15.0f && abs(acc_mag - 1.0f) < 0.20f) {
        zupt_static_frames++;
        
        // --- ① ZUPT連動型のオフセット（ゼロ点）自動補正 ---
        // 完全に静止している状態（ZUPT=True）の間、加速度の初期ノイズ（オフセット）を学習し続ける
        offset_accumulator += vertical_accel_mps2;
        offset_sample_count++;
        // 最新の平均オフセット値を常に更新（移動平均）
        accel_offset_z = offset_accumulator / (float)offset_sample_count;

        if (zupt_static_frames > 15) {
            // 速度が既に小さい時（静止しているかどうかの確認）のみ殺す
            // 挙上中に偶然条件が満たされても絶対に殺さない
            if (abs(velocity) < 0.15f) {
                is_static = true;
            }
        }
        
        // 長時間静止判定が続いた場合はMadgwickを重力方向だけで座標リセット（ドリフト対策）
        if (zupt_static_frames > 100) {
           filter.updateIMU(0, 0, 0, ax_g, ay_g, az_g, dt);
        }
    } else {
        // --- ① 動いている間（ZUPT=False）の処理 ---
        // 動体検知で即座にオフセットの学習をストップし、最後に学習した「ゼロ点」をロック（固定）する
        zupt_static_frames = 0;
        offset_accumulator = 0.0f;
        offset_sample_count = 0;
    }

    // --- 3.5 回転検知 (Rotation Clamp) ---
    // バーベルの回転(Rolling)による誤検知を防ぎ゙
    // 800dps以上の超高速回転は通常の挙上動作ではないとみなす
    // (スマホケース等に入れると遊びで300dpsを超えることがあるため大幅緩和)
    if (gyro_mag > 800.0f) {
        vertical_accel_mps2 = 0;
        // 回転中は速度を強烈に減衰させて異常値が残るのを防ぐ
        velocity *= 0.80f;
    }

    // --- 4. 速度積分 (ZUPTオフセット適用と動的リーキー積分) ---
    if (is_static) {
        // ZUPT発動中: 速やかに0に減衰させる
        vertical_accel_mps2 = 0;
        velocity *= 0.80f; // 強い減衰
        if (abs(velocity) < 0.01f) velocity = 0.0f;
    } else {
        // --- ① オフセット補正の適用 ---
        // 計測された鉛直加速度から「ロックされたオフセット値（ゼロ点）」を引き算し、純粋な運動加速度のみを抽出
        // この処理により、傾きやセンサー固有の定常ノイズによる架空の加速度を根絶する
        float corrected_accel = vertical_accel_mps2 - accel_offset_z;
        
        // 極小ノイズのデッドゾーン（±0.02m/s^2未満の震えは無視）
        // v4.1: 0.06は遅いスクワットの実加速度(0.03-0.05G≈0.3-0.5m/s^2)を殺していたため大幅に緩和
        if (abs(corrected_accel) < 0.02f) {
            corrected_accel = 0.0f;
        }

        // 速度の積み上げ（積分）
        velocity += corrected_accel * dt;
        
        // --- ② 動的リーキー積分（定数減衰の廃止） ---
        // バーベルの挙動（上向きか下向きか）に合わせて、最適でフェアーなブレーキ係数を掛ける
        
        if (velocity < 0.0f) {
            // 【下降・反射領域】
            // VBTでは計測範囲外。次のレップに向けた計算上のドリフト（マイナスの借金）を完全に消し去るため超強烈なブレーキ
            velocity *= 0.90f; 
        } else {
            // 【挙上（コンセントリック）領域】 (velocity >= 0.0f)
            
            // 挙上が終わり、重力に負けて速度が0に戻ろうとしている（落下加速が始まっている）時のソフトランディング処理
            if (velocity < 0.20f && corrected_accel < -0.5f) {
                // 勢いが死んで落ち始めているフェーズ。ZUPTが発動するまでの浮き上がりを抑える。
                // v4.1: 0.95は200Hzで回すと10ms毎に5%減衰=170msで消滅。0.985に緩和し、ゆっくり自然減衰させる
                velocity *= 0.985f;
            } else {
                // 【真の挙上フェーズ（粘りゾーン含む）】
                // 係数を `0.9995` に設定し、2秒の粘るスローレップでも実力（積分速度）の80%以上を維持する
                // 「薄皮一枚のセーフティネット」として働き、完全な1.0による無限発散（ドリフト）のみを器用に回避
                velocity *= 0.9995f; 
            }
        }
    }

    // --- 安全リミット (v4.1: 復活) ---
    // スクワット最速でも~1.3m/s。3.5m/s超は確実にドリフト異常値
    if (velocity > 3.5f) velocity = 3.5f;
    if (velocity < -3.5f) velocity = -3.5f;

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