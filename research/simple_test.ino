#include <Wire.h>

// センサーの制御用ピンの定義
// VDDIO は 3V3 に直接接続したので、ピン制御不要
const int PIN_AD0 = D2;
const int PIN_CS = D3;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("=== VBT Device Simple Test ===\n");

  // 制御ピンの初期化（VDDIOは不要）
  pinMode(PIN_AD0, OUTPUT);
  digitalWrite(PIN_AD0, LOW);    // I2Cアドレス 0x68
  
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);    // I2Cモード有効

  Serial.println("制御ピンを設定しました:");
  Serial.println("  VDDIO → 3V3 (直接接続)");
  Serial.println("  D2 (AD0) = LOW (アドレス 0x68)");
  Serial.println("  D3 (CS)  = HIGH (I2Cモード)");
  Serial.println();

  delay(200);

  // I2C初期化
  Wire.begin();
  delay(100);

  // I2Cスキャン
  Serial.println("=== I2Cスキャン開始 ===");
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
    Serial.println("\n❌ I2Cデバイスが見つかりません");
    Serial.println("\n次を確認してください:");
    Serial.println("1. ICM Pin 3 (SCL) → XIAO D5");
    Serial.println("2. ICM Pin 4 (SDA) → XIAO D4");
    Serial.println("3. ICM Pin 1 (VDD) → XIAO 3V3");
    Serial.println("4. ICM Pin 2 (VDDIO) → XIAO 3V3");
    Serial.println("5. ICM Pin 10 (GND) → XIAO GND");
  } else if (count == 1) {
    Serial.println("\n✓ 正常です！次のステップに進めます。");
  } else {
    Serial.println("\n⚠ 複数のデバイスが検出されました（異常な可能性）");
  }
}

void loop() {
  // 何もしない
}
