//This code 

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

 /*#if defined(ARDUINO_ARCH_SAMD)
    // for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
     #define Serial SerialUSB 
 #endif*/

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);
}

void loop() { 
  //samples cannot exceed about 720 so 360 samples for x and z axis
  int numSamples = 360;
  int halfNumSamples = 180; 
  short data [numSamples*2];
  
  //unsigned long start;
  //unsigned long finish;
  unsigned long impact;
  
  
  lis.begin(0x18);
  //set G Range 2, 4, 8 or 16 G!
  lis.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!
  
  //Threshold is the gforces you expect in the y axis of the pole at takoff between 60-100 m/s^2, accelo is not super accurate so it is tough to estimate 
  byte threshold=55 ; //in m/s^2
  int poleHitBoxAt=NULL;
  int i=0;
  int j;
  //boolean first = true;
  int k;
  int pause = 0;
  float ratio = 209; //to convert to Gs
                                          //float yAccel;
                                                      //unsigned long mils0; 
                                                      //unsigned long mils1;
  //Next three lines prevent false reading from accelero at startup
  //sensors_event_t event;
  //lis.getEvent(&event);
  lis.read();
  delay(2000);
  //STARTS COLLECTING DATA
     //start= micros();     
     while(true){
        i=0;
        for(i; i<numSamples; i++){            
          lis.read();//lis.getEvent(&event);
          
          //SAVES (X,Z) DATA POINTS
          data [i] = lis.x;//data [i] = event.acceleration.x;                                                      
          data [i+(numSamples)] = lis.z;//data [i+(numSamples)] = event.acceleration.z;                                                      
          //checks if threshold reached in Y axis (hit back of box)
          
          if(lis.y/ratio>= threshold){ //I tested it with my hand to make sure accel is in +y direction at impact
            poleHitBoxAt = i;
            impact= micros(); 
            break; //break for loop
          }
          //delayMicroseconds(400);
          //  0 = 0.557s (646Hz)
          //200 = 0.626s (575Hz)
          //400 = 0.691s (520Hz)
        }
        /*if(first)
          {finish = micros();
           first = false;
        }*/
        //leaves while true loop if box hit
        if(poleHitBoxAt!=NULL){
            break;
        }
      }
      while(j<=halfNumSamples){ 
        //corrects pointer in data
        i++;
        if(i>=numSamples){
          i-=numSamples;
        }
        //sensors_event_t event;
        //lis.getEvent(&event);
        lis.read();
        data [i] = lis.x;
        //data [i+numSamples] = event.acceleration.y;
        data [i+numSamples] = lis.z;
        //delay(pause);
        j++;
      }
      
                                                                      //mils1=micros()-mils0;
      //VAULTER JUST TOOK OFF SO WE HAVE TIME BEFORE SENDING DATA
      delay(10000);
//      #if defined(ARDUINO_ARCH_SAMD)
//        #define Serial SerialUSB
//      #endif
      
      //PRINT OUT DATA
      while(true){
//        #ifndef ESP8266
//          while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
//        #endif
//        Serial.begin(9600);
        int b=0;
        k=poleHitBoxAt+halfNumSamples+1;
        
        while(b<numSamples){
          if(k>=numSamples) k-=numSamples;
          Serial.print(float(data [k])/ratio);
          Serial.print(" \t "); 
          Serial.print(float(data [k+numSamples])/ratio);
          Serial.println();
          b++;
          k++;
        }
        
                                                                //Serial.println( yAccel);
        //float dif = data [poleHitBoxAt] - data [poleHitBoxAt-1]; 
        //Serial.println( mils1/1000);
        //Serial.println((float(finish) - float(start))/float(1000));
        Serial.println("1.55ms between data");
        Serial.print(impact/float(1000000)); 
        delay(10000);
      }
}
