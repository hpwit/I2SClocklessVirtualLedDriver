#define NBIS2SERIALPINS 6 // the number of virtual pins here mavimum 6x8=48 strips
#define NUM_LEDS_PER_STRIP 256
#define NUM_LEDS (NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8)
#define LATCH_PIN 27
#define CLOCK_PIN 26
#define NUM_STRIPS (NBIS2SERIALPINS * 8)
#include "I2SClocklessVirtualLedDriver.h"
Pixel leds[NUM_STRIPS * NUM_LEDS_PER_STRIP];

int Pins[6] = {14, 12, 13, 25, 33, 32};

I2SClocklessVirtualLedDriver driver;
void setup()
{
    Serial.begin(115200);

    driver.initled(leds, Pins, CLOCK_PIN, LATCH_PIN);
    driver.setBrightness(10);
}

int off = 0;
long time1, time2, time3;
void loop()
{
    HOW_LONG("Calculate array", {
        for (int j = 0; j < NUM_STRIPS; j++)
        {

            for (int i = 0; i < NUM_LEDS_PER_STRIP; i++)
            {

                leds[(i + off) % NUM_LEDS_PER_STRIP + NUM_LEDS_PER_STRIP * j] = Pixel((NUM_LEDS_PER_STRIP - i) * 255 / NUM_LEDS_PER_STRIP, i * 255 / NUM_LEDS_PER_STRIP, (((128 - i) + 255) % 255) * 255 / NUM_LEDS_PER_STRIP);
            }
        }
    });
    HOW_LONG("ShowPixel", {
        driver.showPixels();
    });

    off++;
}
