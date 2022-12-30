#include "FastLED.h"
#define NBIS2SERIALPINS 4 //the number of virtual pins here mavimum 32 strips
#define STATIC_COLOR_GRB 1 //set the strip color
#define NUM_LEDS_PER_STRIP 360
#define NUM_LEDS (NUM_LEDS_PER_STRIP*NBIS2SERIALPINS*8)
#define CLOCK_PIN 16
#define LATCH_PIN 26
#define NUM_STRIPS 32
#define  ENABLE_HARDWARE_SCROLL 
#include "I2SClocklessVirtualLedDriver.h"
//here we have 3 colors per pixel
uint8_t leds[NUM_STRIPS*NUM_LEDS_PER_STRIP*3];

int pins[16]={0,2,4,5};

I2SClocklessVirtualLedDriver driver;

OffsetDisplay offd;

void setup()
{
  Serial.begin(115200);

  driver.initled((uint8_t*)leds,pins,CLOCK_PIN,LATCH_PIN);
  driver.setBrightness(20);
  for (int j = 0; j < NUM_STRIPS; j++)
  {
     leds[NUM_LEDS_PER_STRIP * j] = CRGB::Red;
    for (int i = 1; i < j + 2; i++)
    {
      leds[i + NUM_LEDS_PER_STRIP * j] = CRGB::Green;
    }
    leds[17+NUM_LEDS_PER_STRIP * j] = CRGB::Blue;
  }
  driver.showPixels();
  delay(1000); 
  offd = driver.getDefaultOffset();
  offd.panel_width=120; 
  offd.panel_height=48;


}

int off = 0;
long time1, time2, time3;
void loop()
{
  
  offd.offsetx = 120*cos(3.14*off/360);
  offd.offsety = 120*sin(3.14*off/360);
  time2 = ESP.getCycleCount();
  driver.showPixels(offd); 
  time3 = ESP.getCycleCount();
  Serial.printf("Calcul pixel fps:%.2f   showPixels fps:%.2f   Total fps:%.2f \n", (float)240000000 / (time2 - time1), (float)240000000 / (time3 - time2), (float)240000000 / (time3 - time1));
  off++;
  delay(50);
}
