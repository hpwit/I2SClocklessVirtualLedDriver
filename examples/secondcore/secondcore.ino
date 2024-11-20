#define NBIS2SERIALPINS 6 // the number of virtual pins here mavimum 6x8=48 strips
#define NUM_LEDS_PER_STRIP 256
#define NUM_LEDS (NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8)
#define I2S_MAPPING_MODE (I2S_MAPPING_MODE_OPTION_MAPPING_IN_MEMORY)
#define NUM_STRIPS (NBIS2SERIALPINS * 8)
#define USE_FASTLED
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
Pixel leds[NUM_STRIPS * NUM_LEDS_PER_STRIP];


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

I2SClocklessVirtualLedDriver driver;
int offset = 0;
void setup()
{
    Serial.begin(115200);

    driver.initled(leds, Pins, CLOCK_PIN, LATCH_PIN);
    driver.setBrightness(20);
    driver.setMapLed(&mapfunction);
    delay(2000);
    RUN_SKETCH_N_TIMES("On one core", 10, {
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
    RUN_SKETCH_N_TIMES("On two cores", 10, {
        HOW_LONG("total with second core", {
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
}

void loop()
{
}