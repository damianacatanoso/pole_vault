
void WifiCOMMHandle() {

  while (true) {
    
    state = WiFi.status();
    if (state != WL_CONNECTED) {
      if (state == WL_NO_SHIELD) {
//        Serial.println("*** Connecting WiFi... ***");
        WiFi.mode(WIFI_STA); 
        WiFi.begin(ssid, pass);
        
      }
      else if (state == WL_CONNECT_FAILED) {
//        Serial.println("*** Connection Failed! ***");
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
#ifndef HB_BINARY
      count_hb++;
      if (count_hb==6){ 
        count_hb = 0;
#endif
        
        if(client.connect(server, port)) {
//          Serial.println("client.connected() is: true");
#ifndef HB_BINARY
          sendTelemetry();
#endif

          uint8_t RX_COUNT = 0;
  //        Serial.println("*** Server Connected! ***");
          
          while(client.connected() || client.available()) {
            if(client.available()) {
              uint8_t rx_byte = client.read();
              RX_BUFFER_TASK[RX_COUNT++] = rx_byte;
              if(RX_COUNT == RX_BUFFER_SIZE) {
                break;
              }
            }
          }
         
          // Make sure the packet has been received correctly and it is for this board
          if(RX_COUNT == RX_BUFFER_SIZE) {
            unsigned short rx_crc_value = 0, tx_crc_value = 0;
            rx_crc_value = calculate_crc(rx_crc_value, RX_BUFFER_TASK, RX_COUNT-2);
            // if the headers and crc are the same
            if((RX_BUFFER_TASK[0] == B10111111) && (RX_BUFFER_TASK[1] == B11111101) && (RX_BUFFER_TASK[2] == B11110000) && (RX_BUFFER_TASK[3] == B00001111) && (RX_BUFFER_TASK[RX_COUNT-2] == lowbyte(rx_crc_value)) && (RX_BUFFER_TASK[RX_COUNT-1] == highbyte(rx_crc_value))) {
              if(RX_BUFFER_TASK[4] == BOARD_NUM_TASK || RX_BUFFER_TASK[4] == B11111110) {
//                byte_command = RX_BUFFER_TASK[5];
//                if (current_mode == 'O') {
//                  if (byte_command == B10011001 || byte_command == B00100000 || byte_command == B00100001) { 
//                    commandComplete = 0; 
//                    if (byte_command == B10011001) { final_state = (char)RX_BUFFER_TASK[8]; }
//                  }
//                }
                rx_queue_send(); //send latest RX_BUFFER_TASK values to the rx_queue
                
//              Serial.print("    Client received command: ");
//              Serial.print(RX_BUFFER[5],HEX);  
                  
              }
            }
            
#ifdef HB_BINARY
          tx_queue_receive();
          sendTelemetry();
#endif
          }
      
        }
        else {
//          Serial.println("client.connected() is false");
          FAILED_CONNECT_TO_SERVER++;
          if(FAILED_CONNECT_TO_SERVER >= 3) {
            FAILED_CONNECT_TO_SERVER = 0;
          }
        }

        vTaskDelay(50/portTICK_PERIOD_MS);
        client.stop();
        
#ifndef HB_BINARY
      }
#endif
//      
    }
//    vTaskDelay(100/portTICK_PERIOD_MS);  
  }


}


//void rx_queue_send() { //Executed by Wifi
//// uxQueueSpacesAvailable(rx_queue) returns number of free spaces in the queue
//  //loop until queue is completely empty. When it is, read.
//  while (uxQueueSpacesAvailable(rx_queue)!=RX_BUFFER_TASK_SIZE) {
//    // wait
//  }
//  if (uxQueueSpacesAvailable(rx_queue)==RX_BUFFER_TASK_SIZE) { //if queue is completely empty
////    Serial.println("sending rx_queue");
//    for( int i = 0; i< RX_BUFFER_TASK_SIZE; i++ ){ //fill it
//      xQueueSend(rx_queue, &RX_BUFFER_TASK[i], 1); //specify number of ticks to wait 
//    }
//  }
//}
//
//void rx_queue_receive() { //Executed by loop
//  if (uxQueueSpacesAvailable(rx_queue)==0) { //if queue is full
////    Serial.println("receiving rx_queue");
//    for( int i = 0; i< RX_BUFFER_SIZE; i++ ){
//      xQueueReceive(rx_queue, &RX_BUFFER[i], 1); //receive elements
//    }
//    byte_command = RX_BUFFER[5];
//    if (current_mode == 'O') {
//      if (byte_command == B10011001 || byte_command == B00100000 || byte_command == B00100001) {  //boltall, unboltall
//        commandComplete = 0; 
//        if (byte_command == B10011001) { final_state = (char)RX_BUFFER[8]; }
//      }
//    }
//    new_rx_buffer_received = true;
//  }
//}
//
//void tx_queue_send() { //Executed by loop
//  if (new_rx_buffer_received) {
//    if (current_mode == 'O') {
//      if (byte_command == B10011001 || byte_command == B00100000 || byte_command == B00100001) {  //boltall, unboltall
//        commandComplete = 0; 
//        if (byte_command == B10011001) { final_state = (char)RX_BUFFER[8]; }
//      }
//    }
//    if (uxQueueSpacesAvailable(tx_queue)==TX_BUFFER_SIZE) { //if queue is completely empty
////      Serial.println("sending tx_queue");
//      for( int i = 0; i< TX_BUFFER_SIZE; i++ ){ //fill it
//        xQueueSend(tx_queue, &TX_BUFFER[i], 1); //specify number of ticks to wait 
//      }
//      new_rx_buffer_received = false;
//    }
//  }
//}
//
//void tx_queue_receive() { //Executed by Wifi
//  //loop until queue is completely full. When it is, read.
//  while (uxQueueSpacesAvailable(tx_queue)!=0) {
//    // wait
//  }
//  if (uxQueueSpacesAvailable(tx_queue)==0) { //if queue is full
////    Serial.println("receiving tx_queue");
//    for( int i = 0; i< TX_BUFFER_TASK_SIZE; i++ ){
//      xQueueReceive(tx_queue, &TX_BUFFER_TASK[i], 1); //receive elements
//    }
//  }
//}
