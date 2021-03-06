#define LEDPIN 13
  // Pin 13: Arduino has an LED connected on pin 13
  // Pin 11: Teensy 2.0 has the LED on pin 11
  // Pin  6: Teensy++ 2.0 has the LED on pin 6
  // Pin 13: Teensy 3.0 has the LED on pin 13
//pin 0 5 and 2 gave boot up issues sometimes use other and define as INPUT_PULLUP
#define SENSORPIN 25

// variables will change:
int sensorState = 0, lastState=0;         // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  //pinMode(LEDPIN, OUTPUT);      
  // initialize the sensor pin as an input:
  pinMode(SENSORPIN, INPUT_PULLUP);     
//  digitalWrite(SENSORPIN, HIGH); // turn on the pullup
  
  Serial.begin(115200);
  Serial.println("START");
}

void loop(){
  Serial.println(digitalRead(SENSORPIN));
  // read the state of the pushbutton value:
//  sensorState = digitalRead(SENSORPIN);

  // check if the sensor beam is broken
  // if it is, the sensorState is LOW:
//  if (sensorState == LOW) {      
//    Serial.println(10);
//  } 
//  else {
//    // turn LED off:
//    //digitalWrite(LEDPIN, LOW); 
//    Serial.println(0);
//  }
  
//  if (sensorState && !lastState) {
//    //Serial.println("Unbroken");
//  } 
//  if (!sensorState && lastState) {
//    //Serial.println("Broken");
//  }
//  lastState = sensorState;
}
