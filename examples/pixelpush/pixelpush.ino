#define NBIS2SERIALPINS 6 // the number of virtual pins here mavimum 6x8=48 strips
#define NUM_LEDS_PER_STRIP 256
#define NUM_LEDS (NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8)
#define LED_WIDTH 128
#define LED_HEIGHT 64
#define NUM_STRIPS (NBIS2SERIALPINS * 8)
#define I2S_MAPPING_MODE I2S_MAPPING_MODE_OPTION_DIRECT_CALCULATION
#include "I2SClocklessVirtualLedDriver.h"
#ifdef CONFIG_IDF_TARGET_ESP32S3
#define LATCH_PIN 46
#define CLOCK_PIN 3
#else

#define LATCH_PIN 27
#define CLOCK_PIN 26
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S3
int Pins[6] = {9, 10,12,8,18,17};
#else
int Pins[6] = {14, 12, 13, 25, 33, 32};
#endif



I2SClocklessVirtualLedDriver driver;

Pixel colors[256];
int offset = 0;
void createcolors()
{
  for (int i = 0; i < 256; i++)
  {

    colors[i] = Pixel(i, 0, 0);
  }
}

Pixel functionCalc(uint16_t ledtodisp, int pin, int virtualpin)
{
  // the pixel is the led ledtodisp on the strip number = pin*8+virtualpin
  long Y, X;
  int x = ledtodisp % 16;
  int y = (ledtodisp >> 4);

  if (y % 2 == 0)
  {
    Y = (pin << 4) + y;
    X = (virtualpin) + x;
  }
  else
  {
    Y = (pin << 4) + y;
    X = (virtualpin) + 16 - x - 1;
  }

  return colors[(X + Y + offset) % 256];
}

void setup()
{
  Serial.begin(115200);
  driver.setPixelCalc(&functionCalc);
  driver.initled(Pins, CLOCK_PIN, LATCH_PIN);
  driver.setBrightness(40);
  createcolors();
}

void loop()
{

  HOW_LONG("display", {
    driver.showPixels();
  });
  offset++;
}