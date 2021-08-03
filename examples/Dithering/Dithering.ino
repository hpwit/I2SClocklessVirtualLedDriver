#include "FastLED.h"
#define NBIS2SERIALPINS 2 //the number of virtual pins
#define STATIC_COLOR_GRB 1 //set the strip color
#define NUM_LEDS_PER_STRIP 50
#define NUM_LEDS (NUM_LEDS_PER_STRIP*NBIS2SERIALPINS*8)
#define CLOCK_PIN 16
#define LATCH_PIN 4
#define NUM_STRIPS 16
#include "I2SClocklessVirtualLedDriver.h"

 
//here we have 3 colors per pixel
//uint8_t leds[numstrips*ledsperstrip*3];
//this one below is same as the one above
CRGB leds1[NUM_STRIPS*NUM_LEDS_PER_STRIP];
CRGB leds2[NUM_STRIPS*NUM_LEDS_PER_STRIP];
CRGB leds3[NUM_STRIPS*NUM_LEDS_PER_STRIP];
CRGB leds4[NUM_STRIPS*NUM_LEDS_PER_STRIP];
CRGB leds5[NUM_STRIPS*NUM_LEDS_PER_STRIP];
CRGB leds6[NUM_STRIPS*NUM_LEDS_PER_STRIP];
int pins[2]={0,2};

I2SClocklessVirtualLedDriver driver;
void setup() {
    Serial.begin(115200);
    
    driver.initled((uint8_t*)leds,pins,CLOCK_PIN,LATCH_PIN);
    CRGB color=CRGB(10,0,0);
    
//led 0 equivalent of 10/6=1,67
//led 1 equivalent of 20/6=3,33
//led 2 equivalent of 30/6=5
//led 3 equivalent of 40/6=6,67
//led 4 equivalent of 50/6=8,33
//led 5 equivalent of 10

    for(int j=0;j<numstrips;j++)
    {
      
        leds1[5+j*ledsperstrip]=color; 

        leds2[4+j*ledsperstrip]=color;
        leds2[5+j*ledsperstrip]=color;
        
        leds3[3+j*ledsperstrip]=color;
        leds3[4+j*ledsperstrip]=color;
        leds3[5+j*ledsperstrip]=color;


        leds4[2+j*ledsperstrip]=color;
        leds4[3+j*ledsperstrip]=color;
        leds4[4+j*ledsperstrip]=color;
        leds4[5+j*ledsperstrip]=color;
           
        leds5[1+j*ledsperstrip]=color;
        leds5[2+j*ledsperstrip]=color;
        leds5[3+j*ledsperstrip]=color;
        leds5[4+j*ledsperstrip]=color;
        leds5[5+j*ledsperstrip]=color; 

        leds6[0+j*ledsperstrip]=color;
        leds6[1+j*ledsperstrip]=color;
        leds6[2+j*ledsperstrip]=color;
        leds6[3+j*ledsperstrip]=color;
        leds6[4+j*ledsperstrip]=color;
        leds6[5+j*ledsperstrip]=color; 
        
    }
    
}


void loop() {
  
    while(1)
    {
       driver.showPixels(); //equals to driver.showPixels((uint8_t *)leds1); as the leds are initialized with leds1
       driver.showPixels((uint8_t *)leds2);
       driver.showPixels((uint8_t *)leds3);
       driver.showPixels((uint8_t *)leds4);
       driver.showPixels((uint8_t *)leds5);
       driver.showPixels((uint8_t *)leds6);
      
    }
   
}
