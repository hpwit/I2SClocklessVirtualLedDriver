#define NBIS2SERIALPINS 6
#define NUM_LEDS_PER_STRIPS 256
#define LED_WIDTH 128
#define LED_HEIGHT 96

#define I2S_MAPPING_MODE (I2S_MAPPING_MODE_OPTION_MAPPING_IN_MEMORY)
#include "I2SClocklessVirtualLedDriver.h"

#define NUM_LEDS (LED_HEIGHT * LED_WIDTH)
#define LATCH_PIN 27
#define CLOCK_PIN 26
Pixel leds[LED_HEIGHT * LED_WIDTH]; // 48*256=12288 leds 36,864 bytes

int Pins[6] = {14, 12, 13, 25, 33, 32};

I2SClocklessVirtualLedDriver driver;

uint16_t mapfunction(uint16_t pos)
{
  int panelnumber = pos / 256;
  int datainpanel = pos % 256;
  int yp = panelnumber / 8;
  int Xp = panelnumber % 8;
  int Y = yp;
  int X = Xp;

  int x = datainpanel % 16;
  int y = datainpanel / 16;

  if (y % 2 == 0)
  {
    Y = Y * 16 + y;
    X = X * 16 + x;
  }
  else
  {
    Y = Y * 16 + y;
    X = X * 16 + 16 - x - 1;
  }

  return Y * 16 * 8 + X;
}
int trainstart = 0;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  driver.initled(leds, Pins, CLOCK_PIN, LATCH_PIN);
  driver.setBrightness(30);
}

int offset = 0;
void loop()
{
  RUN_SKETCH_FOR("without mapping", 5000, {
    memset((uint8_t*)leds, 0, LED_HEIGHT * LED_WIDTH * 3);
    for (int i = 0; i < LED_HEIGHT; i++)
    {
      for (int j = 0; j < i + 1; j++)
      {
        leds[i * LED_WIDTH + (j + trainstart) % LED_WIDTH] = Pixel(255, 0, 0);
      }
    }

    driver.showPixels();
    trainstart++;
  });
  driver.setMapLed(&mapfunction);
  RUN_SKETCH_FOR("with mapping", 5000, {
     memset((uint8_t*)leds, 0, LED_HEIGHT * LED_WIDTH * 3);
    for (int i = 0; i < LED_HEIGHT; i++)
    {
      for (int j = 0; j < i + 1; j++)
      {
        leds[i * LED_WIDTH + (j + trainstart) % LED_WIDTH] = Pixel(255, 0, 0);
      }
    }

    driver.showPixels();
    trainstart++;
  });
  //remove the mapping
  driver.setMapLed(NULL);
}