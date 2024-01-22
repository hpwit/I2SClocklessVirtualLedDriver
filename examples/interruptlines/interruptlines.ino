#define NBIS2SERIALPINS 6
#define NUM_LEDS_PER_STRIPS 256
#define LED_WIDTH 123
#define LED_HEIGHT 48

#define I2S_MAPPING_MODE (I2S_MAPPING_MODE_OPTION_SCROLL_MAPPING_ALL_IN_MEMORY | I2S_MAPPING_MODE_OPTION_INTERRUPT_LINE)
#include "I2SClocklessVirtualLedDriver.h"

#define NUM_LEDS (LED_HEIGHT * LED_WIDTH)
#include "pics.h"
#define LATCH_PIN 27
#define CLOCK_PIN 26
Pixel leds[LED_HEIGHT * LED_WIDTH]; // 48*256=12288 leds 36,864 bytes

int Pins[6] = {14, 12, 13, 25, 33, 32};

I2SClocklessVirtualLedDriver driver;
OffsetDisplay offd;

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
  // driver.setPixelCalc(&functionCalc);

  driver.initled(lapin001, Pins, CLOCK_PIN, LATCH_PIN);
  driver.setMapLed(&mapfunction);
  driver.setBrightness(30);
  offd = driver.getDefaultOffset();
  offd.panel_width = 128;
  offd.panel_height = 96;
  offd.image_height = LED_HEIGHT;
  offd.image_width = LED_WIDTH;
  offd.offsetx = 128 / 2 - LED_WIDTH / 2;
  offd.offsety = 96 / 2 - LED_HEIGHT / 2;
}

void resetOffSetDisplay()
{
  offd.xc = 64;
  offd.yc = 48;

  offd.offsetx = 128 / 2 - LED_WIDTH / 2;
  offd.offsety = 96 / 2 - LED_HEIGHT / 2;
  offd.rotation = 0;
  offd.enableLoopx = false;
  offd.enableLoopy = false;
  driver.resetInterruptsParameters();
}

int offset = 0;
void loop()
{
  resetOffSetDisplay();
  RUN_SKETCH_FOR("offset per line", 5000, {
    for (int i = 0; i < 96; i++)
    {
      driver.offsetsx[i] = 64 * sin((offset + i) * PI / 200);
    }
    driver.showPixels(offd);
    offset++;
  });
  resetOffSetDisplay();
  offset = 0;
  RUN_SKETCH_FOR("scaling per line", 5000, {
    for (int i = 0; i < 96; i++)
    {
      driver.scalingx[i] = sin((offset + i) * PI / 200);
    }
    driver.showPixels(offd);
    offset++;
  });
  resetOffSetDisplay();
  offset = 0;
  // offd.offsety
  RUN_SKETCH_FOR("scaling per line", 10000, {
    for (int i = 0; i < 96; i++)
    {
      driver.scalingy[i] = sin(((offset + i) % 96) * PI / 96);
    }
    driver.showPixels(offd);
    offset++;
  });
}
