// HC-05 / HC-06 Bluetooth Test - Teensy 4.1 on Serial2 (pins 7=RX2, 8=TX2)

unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);  // Default HC-05 baud rate
  while (!Serial) ;     // Wait for Serial Monitor

  Serial.println("=== HC-05 Bluetooth Test ===");
  Serial.println("Type into Serial Monitor to send to Bluetooth.");
  Serial.println("A message will also be sent every 5 seconds.\n");
}

void loop() {
  // Periodically send test message to Bluetooth
  if (millis() - lastSend >= 5000) {
    lastSend = millis();
    Serial2.println("Hello from Teensy 4.1!");
    Serial.println("[USB] Sent: Hello from Teensy 4.1!");
  }

  // Forward anything from Bluetooth to Serial Monitor
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
  }

  // Forward anything from USB Serial to Bluetooth
  if (Serial.available()) {
    char c = Serial.read();
    Serial2.write(c);
  }
}
