void setup() {
  // put your setup code here, to run once:
  bufferInit();
  
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    // Wifi pinned to core 0
  xTaskCreatePinnedToCore(WifiCOMMHandle, "WifiCOMMHandle", 8192, NULL, 1, &WiFiCOMM, 0);
}
