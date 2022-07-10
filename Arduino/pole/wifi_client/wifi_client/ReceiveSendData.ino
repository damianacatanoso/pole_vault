
void WifiCOMMHandle() {

  if (data_ready) {
    
    state = WiFi.status();
    if (state != WL_CONNECTED) {
      if (state == WL_NO_SHIELD) {
        Serial.println("*** Connecting WiFi... ***");
//        WiFi.mode(WIFI_STA); 
        WiFi.begin(ssid, pass);
        
      }
      else if (state == WL_CONNECT_FAILED) {
        Serial.println("*** Connection Failed! ***");
        WiFi.disconnect(true);
      }
      else if (state == WL_DISCONNECTED) {
        if (!WiFi_FIRST_CONNECT) {
          WiFi_FIRST_CONNECT = true;          
//          Serial.println("****** WiFi Disconnected! ******");
          
          WiFi.disconnect(true);
        }
      }
//      vTaskDelay (200/portTICK_PERIOD_MS);
    }
    else {
      if (WiFi_FIRST_CONNECT) {  // Report only once
        WiFi_FIRST_CONNECT = false;
//        Serial.print("Connected to ");
//        Serial.println(ssid);
//        Serial.print("IP address: ");
//        Serial.println(WiFi.localIP());
      }
        
      if(client.connect(server, port)) {
//          Serial.println("client.connected() is: true");
        update_tx_buffer();
        sendTelemetry();
      }
      else {
//          Serial.println("client.connected() is false");
        FAILED_CONNECT_TO_SERVER++;
        if(FAILED_CONNECT_TO_SERVER >= 3) {
          FAILED_CONNECT_TO_SERVER = 0;
        }
      }
      
//      vTaskDelay(50/portTICK_PERIOD_MS);
      client.stop();
    } 
  }

}
