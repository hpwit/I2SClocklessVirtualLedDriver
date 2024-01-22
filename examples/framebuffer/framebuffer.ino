// change this to 0 to see the difference
#define TEST_USE_FRAMEBUFFER 1

#define NBIS2SERIALPINS 6 // the number of virtual pins here mavimum 6x8=48 strips
#define NUM_LEDS_PER_STRIP 256
#define NUM_LEDS (NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8)
#define NUM_STRIPS (NBIS2SERIALPINS * 8)
#include "I2SClocklessVirtualLedDriver.h"
#define _LATCH_PIN 27
#define _CLOCK_PIN 26

#if TEST_USE_FRAMEBUFFER == 1
frameBuffer leds = frameBuffer(NUM_LEDS);
#else
Pixel leds[NUM_LEDS];
#endif
int Pins[6] = {14, 12, 13, 25, 33, 32};

I2SClocklessVirtualLedDriver driver;
int offset = 0;
void setup()
{
    Serial.begin(115200);
#if TEST_USE_FRAMEBUFFER == 1
    driver.initled(&leds, Pins, _CLOCK_PIN, _LATCH_PIN);
#else
    driver.initled(leds, Pins, _CLOCK_PIN, _LATCH_PIN);
#endif
    driver.setBrightness(20);
}
Pixel colors[3] = {Pixel(255, 0, 0), Pixel(0, 255, 0), Pixel(0, 0, 255)};
void loop()
{
    for (int i = 0; i < NBIS2SERIALPINS * 16; i++)
    {
        for (int j = 0; j < 8 * 16; j++)
        {
            leds[i * (8 * 16) + (j) % (8 * 16)] = colors[offset % 3];
        }
    }
    offset++;
    delay(1000);
    driver.showPixels(NO_WAIT); // by default when using the framebuffer the showPixels() will be called with NO_WAIT
}