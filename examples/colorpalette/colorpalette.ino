#define NBIS2SERIALPINS 6 // the number of virtual pins here mavimum 6x8=48 strips
#define NUM_LEDS_PER_STRIP 256
#define NUM_LEDS (NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8)
#define LED_WIDTH 128
#define LED_HEIGHT 96
#define NUM_STRIPS (NBIS2SERIALPINS * 8)
#define I2S_MAPPING_MODE (I2S_MAPPING_MODE_OPTION_MAPPING_IN_MEMORY)
#define _USE_PALETTE
#define USE_FASTLED // to have the CHSV mapping
#include "I2SClocklessVirtualLedDriver.h"
#define LATCH_PIN 27
#define CLOCK_PIN 26

int Pins[6] = {14, 12, 13, 25, 33, 32};

I2SClocklessVirtualLedDriver driver;
uint8_t leds[NUM_LEDS];
Pixel colors[256];
void createcolors(int offset)
{
  for (int i = 0; i < 256; i++)
  {
    CRGB d = CHSV((i + offset) % 256, 255, 255);
    colors[i] = d;
  }
}

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
int coloroffset = 0;

void setup()
{
  Serial.begin(115200);
  driver.initled(leds, Pins, CLOCK_PIN, LATCH_PIN);
  driver.setPalette((uint8_t *)colors);
  driver.setMapLed(&mapfunction);
  driver.setBrightness(40);
  createcolors(0);
  for (int i = 0; i < LED_WIDTH; i++)
  {
    for (int j = 0; j < LED_HEIGHT; j++)
    {
      leds[j * LED_WIDTH + i] = i + j;
    }
  }
}

void loop()
{

  driver.showPixels();
  coloroffset++;
  createcolors(coloroffset);
}