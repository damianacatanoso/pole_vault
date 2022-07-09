// Server is connecting to the client only if triggering event happened
// Receives the packet
// Converts packet assigning it to variables


void WifiCOMMHandle(void *pvParameters)
{
  wl_status_t state;

  // might need to execute this code every time triggering event
  WiFi.softAP(ssid, pass, 1, 1, 4);

  IPAddress IP = WiFi.softAPIP();
  
  server.begin();                         // starts the server

  Serial.print("AP IP address: ");Serial.println(IP);
  Serial.print("Status: "); Serial.println(WiFi.status());
  Serial.print("IP: ");     Serial.println(WiFi.localIP());
  Serial.print("Subnet: "); Serial.println(WiFi.subnetMask());
  Serial.print("Gateway: "); Serial.println(WiFi.gatewayIP());
  Serial.print("SSID: "); Serial.println(WiFi.SSID());
  Serial.print("Signal: "); Serial.println(WiFi.RSSI());
  Serial.println("*** Server STARTED ***");


  while (true) {
    if (triggering_event) {
      state = WiFi.status(); 
      // Create client object, returns 0 if client doesn't have data available
      WiFiClient client = server.available();    
      
      if (client) {
        if (!client.connected()) {
        }
        if (client.connected()) {
  
          // Read client's telemetry
          uint8_t RX_COUNT = 0;
          while(client.connected()) {
            if(client.available()) {
              uint8_t rx_byte = client.read();
              RX_BUFFER[RX_COUNT++] = rx_byte;
              if(RX_COUNT == RX_BUFFER_SIZE) {
                break;
              }
            }
          }
  
          if(RX_COUNT == RX_BUFFER_SIZE) {
            unsigned short rx_crc_value = 0;
            rx_crc_value = calculate_crc(rx_crc_value, RX_BUFFER, RX_COUNT-2);
  
            if((RX_BUFFER[0] == B10111111) && (RX_BUFFER[1] == B11111101) && (RX_BUFFER[2] == B11110000) && (RX_BUFFER[3] == B00001111) && (RX_BUFFER[RX_COUNT-2] == lowbyte(rx_crc_value)) && (RX_BUFFER[RX_COUNT-1] == highbyte(rx_crc_value))) {
              bufferConversion(RX_BUFFER);
              triggering_event = false;
            }
          }
          
        }
        client.stop();                // terminates the connection with the client
      }
    }
    vTaskDelay(50/portTICK_PERIOD_MS);
    
  }
  
}
