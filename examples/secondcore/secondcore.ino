#define NBIS2SERIALPINS 6 // the number of virtual pins here mavimum 6x8=48 strips
#define NUM_LEDS_PER_STRIP 256
#define NUM_LEDS (NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8)
#define LATCH_PIN 27
#define CLOCK_PIN 26
#define NUM_STRIPS (NBIS2SERIALPINS * 8)
#define USE_FASTLED
#include "I2SClocklessVirtualLedDriver.h"
Pixel leds[NUM_STRIPS * NUM_LEDS_PER_STRIP];

int Pins[6] = {14, 12, 13, 25, 33, 32};

I2SClocklessVirtualLedDriver driver;
void setup()
{
    Serial.begin(115200);

    driver.initled(leds, Pins, CLOCK_PIN, LATCH_PIN);
    driver.setBrightness(20);
}

void loop()
{
    RUN_SKETCH_FOR("running on core 1 ", 1000, {
        HOW_LONG("total without second core", {
            for (int i = 0; i < NBIS2SERIALPINS * 16; i++)
            {
                for (int j = 0; j < 8 * 16; j++)
                {
                    leds[i * (8 * 16) + (j + offset) % (8 * 16)] = CHSV((i + j), 255, 255);
                }
            }
            driver.showPixels();
        });
    });

    driver.enableShowPixelsOnCore(0);
    RUN_SKETCH_FOR("running on core 0 ", 1000, {
        HOW_LONG("total without second core", {
            for (int i = 0; i < NBIS2SERIALPINS * 16; i++)
            {
                for (int j = 0; j < 8 * 16; j++)
                {
                    leds[i * (8 * 16) + (j + offset) % (8 * 16)] = CHSV((i + j), 255, 255);
                }
            }
            driver.showPixels();
        });
    });
    // go back to core 1;
    driver.enableShowPixelsOnCore(1);
}