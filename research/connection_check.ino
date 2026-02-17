#include <Wire.h>
#include "ICM42688.h"

// センサーの制御用ピンの定義
const int PIN_VDDIO = D1;
const int PIN_AD0 = D2;
const int PIN_CS = D3;

// ICM42688オブジェクトの作成（I2Cアドレス0x68）
ICM42688 IMU(Wire, 0x68);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("--- VBT Device Connection Check ---");

  // 【重要】センサーの電源と設定ピンを初期化します
  pinMode(PIN_VDDIO, OUTPUT);
  digitalWrite(PIN_VDDIO, HIGH); // VDDIO供給開始
  
  pinMode(PIN_AD0, OUTPUT);
  digitalWrite(PIN_AD0, LOW);    // I2Cアドレスを0x68に設定
  
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);   // I2Cモードを有効化

  delay(200); // 電源が安定するまで待機（長めに）

  // I2Cバスを明示的に初期化
  Wire.begin();
  delay(100);

  // デバッグ: I2Cスキャンを実行
  Serial.println("I2Cデバイスをスキャン中...");
  byte count = 0;
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at 0x");
      if (i < 16) Serial.print("0");
      Serial.println(i, HEX);
      count++;
    }
  }
  if (count == 0) {
    Serial.println("I2Cデバイスが見つかりません！");
  }
  Serial.println("スキャン完了\n");

  // センサーの接続確認
  Serial.println("ICM-42688の初期化を開始...");
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("ERROR: ICM-42688が見つかりません。");
    Serial.print("Status: "); Serial.println(status);
    Serial.println("配線とピンの設定を確認してください。");
    while (1) delay(10);
  }

  Serial.println("SUCCESS: ICM-42688を認識しました！");
  
  // センサーの設定（計測できる状態にする）
  status = IMU.setAccelFS(ICM42688::gpm16);  // ±16g
  Serial.print("setAccelFS status: "); Serial.println(status);
  
  status = IMU.setGyroFS(ICM42688::dps2000); // ±2000deg/s
  Serial.print("setGyroFS status: "); Serial.println(status);
  
  delay(100);
  Serial.println("\nデータ取得を開始します...\n");
}

void loop() {
  // センサーから最新データを読み取る
  int status = IMU.getAGT();
  
  if (status < 0) {
    Serial.print("ERROR: データ読み取り失敗 (Status: ");
    Serial.print(status);
    Serial.println(")");
    delay(500);
    return;
  }

  // 加速度データを表示（これが動けば成功！）
  Serial.print("加速度 X:"); Serial.print(IMU.accX());
  Serial.print(" Y:"); Serial.print(IMU.accY());
  Serial.print(" Z:"); Serial.print(IMU.accZ());
  Serial.println(" g");

  delay(100);
}
