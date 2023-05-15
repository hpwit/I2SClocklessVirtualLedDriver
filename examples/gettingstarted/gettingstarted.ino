#define NBIS2SERIALPINS 4 //the number of virtual pins here mavimum 32 strips
#define STATIC_COLOR_GRB 1 //set the strip color
#define NUM_LEDS_PER_STRIP 60 
#define NUM_LEDS (NUM_LEDS_PER_STRIP*NBIS2SERIALPINS*8)
#define CLOCK_PIN 16
#define LATCH_PIN 26
#define NUM_STRIPS 32
#include "I2SClocklessVirtualLedDriver.h"
//here we have 3 colors per pixel
Pixel leds[NUM_STRIPS*NUM_LEDS_PER_STRIP];

int pins[16]={0,2,4,5};

I2SClocklessVirtualLedDriver driver;
void setup() {
    Serial.begin(115200);
    
  driver.initled(leds,pins,CLOCK_PIN,LATCH_PIN);
    driver.setBrightness(10);
    
}

int off=0;
long time1,time2,time3;
void loop() {
    time1=ESP.getCycleCount();
    for(int j=0;j<NUM_STRIPS;j++)
    {
        
        for(int i=0;i<NUM_LEDS_PER_STRIP;i++)
        {
            leds[0]=Pixel(0,0,0);
           // leds[(i+off)%NUM_LEDS_PER_STRIP+NUM_LEDS_PER_STRIP*j]=Pixel((NUM_LEDS_PER_STRIP-i)*255/NUM_LEDS_PER_STRIP,i*255/NUM_LEDS_PER_STRIP,(((128-i)+255)%255)*255/NUM_LEDS_PER_STRIP);
            
        }
    }
    time2=ESP.getCycleCount();
   driver.showPixels();
    time3=ESP.getCycleCount();
    Serial.printf("Calcul pixel fps:%.2f   showPixels fps:%.2f   Total fps:%.2f \n",(float)240000000/(time2-time1),(float)240000000/(time3-time2),(float)240000000/(time3-time1));
    off++;
    
}
