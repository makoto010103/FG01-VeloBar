#include <Wire.h>

// センサーの制御用ピンの定義
const int PIN_VDDIO = D1;
const int PIN_AD0 = D2;
const int PIN_CS = D3;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("=== XIAO nRF52840 ピンテスト ===\n");

  // 制御ピンの初期化
  pinMode(PIN_VDDIO, OUTPUT);
  digitalWrite(PIN_VDDIO, HIGH);
  
  pinMode(PIN_AD0, OUTPUT);
  digitalWrite(PIN_AD0, LOW);
  
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  Serial.println("制御ピンを設定しました:");
  Serial.println("  D1 (VDDIO) = HIGH");
  Serial.println("  D2 (AD0)   = LOW");
  Serial.println("  D3 (CS)    = HIGH");
  Serial.println();

  delay(200);

  // I2Cピンの情報を表示
  Serial.println("I2Cピンの設定:");
  Serial.print("  SDA = D4 (物理ピン: P0.");
  Serial.print(g_ADigitalPinMap[D4]);
  Serial.println(")");
  Serial.print("  SCL = D5 (物理ピン: P0.");
  Serial.print(g_ADigitalPinMap[D5]);
  Serial.println(")");
  Serial.println();

  // I2C初期化（デフォルトピン）
  Serial.println("I2Cバスを初期化中（デフォルトピン使用）...");
  Wire.begin();
  delay(100);

  // I2Cスキャン
  Serial.println("\n=== I2Cスキャン開始 ===");
  byte count = 0;
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("✓ I2Cデバイス発見: 0x");
      if (i < 16) Serial.print("0");
      Serial.println(i, HEX);
      count++;
    }
  }
  
  Serial.println("=== スキャン完了 ===");
  Serial.print("検出されたデバイス数: ");
  Serial.println(count);
  
  if (count == 0) {
    Serial.println("\n【問題】I2Cデバイスが見つかりません！");
    Serial.println("\n考えられる原因:");
    Serial.println("1. SDA/SCLの配線が間違っている");
    Serial.println("2. センサーに電源が供給されていない");
    Serial.println("3. 接触不良がある");
    Serial.println("\n配線を再確認してください:");
    Serial.println("  ICM Pin 3 (SCL) → XIAO D5");
    Serial.println("  ICM Pin 4 (SDA) → XIAO D4");
  }
}

void loop() {
  // 何もしない
}
