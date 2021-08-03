#define FULL_DMA_BUFFER
#define NBIS2SERIALPINS 4 //the number of virtual pins
#define STATIC_COLOR_GRB 1 //set the strip color
#define NUM_LEDS_PER_STRIP 50
#define NUM_LEDS (NUM_LEDS_PER_STRIP*NBIS2SERIALPINS*8)
#define CLOCK_PIN 16
#define LATCH_PIN 26
#define NUM_STRIPS 32
#include "I2SClocklessVirtualLedDriver.h"
//here we have 3 colors per pixel
//uint8_t leds[numstrips*ledsperstrip*3];
//this one below is same as the one above
uint8_t leds[NUM_STRIPS*NUM_LEDS_PER_STRIP*3];

int pins[16]={0,2,4,5};

I2SClocklessVirtualLedDriver driver;
void setup() {
    Serial.begin(115200);
    
    driver.initled((uint8_t*)leds1,pins,CLOCK_PIN,LATCH_PIN);
    driver.setBrightness(10);
    driver.showPixelsFromBuffer(LOOP); //start displaying in  loop what ever is in the DMA buffer
    
}

int off=0;
void loop() {
    for(int j=0;j<NUM_STRIPS;j++)
    {
        
        for(int i=0;i<NUM_LEDS_PER_STRIP;i++)
        {
            
           driver.setPixelinBuffer((i+off)%NUM_LEDS_PER_STRIP+NUM_LEDS_PER_STRIP*j,(NUM_LEDS_PER_STRIP-i)*255/NUM_LEDS_PER_STRIP,i*255/NUM_LEDS_PER_STRIP,(((128-i)+255)%255)*255/NUM_LEDS_PER_STRIP);
            
        }
    }
    off++;
    if(off==500)
    {
      Serial.println("we stop the display loop");
      driver.stopDisplayLoop();
    }
    if(off==1500)
    {
      Serial.println("we restart the display loop");
      driver.showPixelsFromBuffer(LOOP);
    }
}