#include <Wire.h>

// ピン定義 (XIAO nRF52840)
const int PIN_IMU_VDDIO = D1;  // VDDIO用 (HIGH出力)
const int PIN_IMU_AD0   = D2;  // AD0用 (LOW出力 -> 0x68)
const int PIN_IMU_CS    = D3;  // CS用 (HIGH出力 -> I2Cモード)

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\nI2C Scanner (GPIO Config)");

  // 1. ピン設定（配線を簡単にするため、GPIOで電圧を供給）
  pinMode(PIN_IMU_VDDIO, OUTPUT);
  pinMode(PIN_IMU_AD0, OUTPUT);
  pinMode(PIN_IMU_CS, OUTPUT);

  // 2. 電圧設定
  digitalWrite(PIN_IMU_VDDIO, HIGH); // VDDIOに3.3V供給
  digitalWrite(PIN_IMU_AD0, LOW);    // AD0をGND(0V)に設定 -> アドレス0x68
  digitalWrite(PIN_IMU_CS, HIGH);    // CSを3.3Vに設定 -> I2Cモード有効
  
  delay(100); // 電圧安定待ち

  // 3. I2Cバス開始
  // I2Cピンの内蔵プルアップ抵抗も有効化
  pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
  pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
  Wire.begin();
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);
}
