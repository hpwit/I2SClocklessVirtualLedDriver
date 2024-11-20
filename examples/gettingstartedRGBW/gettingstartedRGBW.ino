#define NBIS2SERIALPINS 4 // the number of virtual pins here mavimum 32 strips
#define NUM_LEDS_PER_STRIP 50
#define NUM_LEDS (NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8)
#define NUM_STRIPS 32
#define COLOR_RGBW
// here we have 4 colors per pixel
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



I2SClocklessVirtualLedDriver driver;
void setup()
{
    Serial.begin(115200);

    driver.initled(leds, pins, CLOCK_PIN, LATCH_PIN);
    driver.setBrightness(10);
}

int off = 0;
long time1, time2, time3;
void loop()
{
    time1 = ESP.getCycleCount();
    for (int j = 0; j < NUM_STRIPS; j++)
    {

        for (int i = 0; i < NUM_LEDS_PER_STRIP; i++)
        {

            leds[(i + off) % NUM_LEDS_PER_STRIP + NUM_LEDS_PER_STRIP * j] = Pixel(255 - i, i, ((128 - i) + 255) % 255, i / 25);
        }
    }
    time2 = ESP.getCycleCount();
    driver.showPixels();
    time3 = ESP.getCycleCount();
    Serial.printf("Calcul pixel fps:%.2f   showPixels fps:%.2f   Total fps:%.2f \n", (float)240000000 / (time2 - time1), (float)240000000 / (time3 - time2), (float)240000000 / (time3 - time1));
    off++;
}