void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // 内蔵LED（緑）
  pinMode(LED_RED, OUTPUT);      // 赤LED
  pinMode(LED_BLUE, OUTPUT);     // 青LED
}

void loop() {
  // 緑LED点滅
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  
  // 赤LED点滅
  digitalWrite(LED_RED, HIGH);
  delay(500);
  digitalWrite(LED_RED, LOW);
  delay(500);
  
  // 青LED点滅
  digitalWrite(LED_BLUE, HIGH);
  delay(500);
  digitalWrite(LED_BLUE, LOW);
  delay(500);
}
