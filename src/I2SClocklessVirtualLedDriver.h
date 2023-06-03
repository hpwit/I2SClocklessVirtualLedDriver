
/*
 
 */

#pragma once

#include "esp_heap_caps.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "rom/lldesc.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <rom/ets_sys.h>
#include "esp32-hal-log.h"
#include <soc/rtc.h>
#include "math.h"
#define I2S_DEVICE 0

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#define NUM_VIRT_PINS 7

#ifndef NBIS2SERIALPINS
#define NBIS2SERIALPINS 1
#endif

#ifndef NUM_LEDS_PER_STRIP
#define NUM_LEDS_PER_STRIP 256
#endif

#ifndef HARDWARESPRITES
#define HARDWARESPRITES 0
#endif


//to define coleor different per strip
#ifndef STATICCOLOR
#define STATICCOLOR 1
#endif

#ifdef COLOR_RGBW
#define p_r 1
#define p_g 0
#define p_b 2
#define nb_components 4
#else
#ifdef COLOR_RGB
#define p_r 0
#define p_g 1
#define p_b 2
#define nb_components 3
#else
#ifdef COLOR_RBG
#define p_r 0
#define p_g 2
#define p_b 1
#define nb_components 3
#else
#ifdef COLOR_GBR
#define p_r 2
#define p_g 0
#define p_b 1
#define nb_components 3
#else
#ifdef COLOR_BGR
#define p_r 2
#define p_g 1
#define p_b 0
#define nb_components 3
#else
#ifdef COLOR_BRG
#define p_r 1
#define p_g 2
#define p_b 0
#define nb_components 3
#else
#ifdef COLOR_GRB
#define p_r 1
#define p_g 0
#define p_b 2
#define nb_components 3
#else

#define p_r 1
#define p_g 0
#define p_b 2
#define nb_components 3
#endif
#endif
#endif
#endif
#endif
#endif
#endif

#ifdef _USE_PALETTE
    #ifndef PALETTE_SIZE
        #define _palette_size 1
    #else
      #define _palette_size PALETTE_SIZE
    #endif
#else
    #define _palette_size nb_components
#endif

#define OFFSET (NUM_VIRT_PINS + 1)
#define I2S_OFF (((NUM_VIRT_PINS + 1) * NUM_LEDS_PER_STRIP) * _palette_size)
#define I2S_OFF2 ((I2S_OFF * NBIS2SERIALPINS - NUM_LEDS_PER_STRIP * _palette_size))
#define I2S_OFF3 ((I2S_OFF * NBIS2SERIALPINS + NUM_LEDS_PER_STRIP * _palette_size))
#define I2S_OFF4 ((I2S_OFF * NBIS2SERIALPINS - 3 * NUM_LEDS_PER_STRIP * _palette_size))
#define I2S_OFF_MAP (((NUM_VIRT_PINS + 1) * NUM_LEDS_PER_STRIP))
#define I2S_OFF2_MAP ((I2S_OFF_MAP * NBIS2SERIALPINS - NUM_LEDS_PER_STRIP))
#define I2S_OFF3_MAP ((I2S_OFF_MAP * NBIS2SERIALPINS + NUM_LEDS_PER_STRIP))
#define I2S_OFF4_MAP ((I2S_OFF_MAP * NBIS2SERIALPINS - 3 * NUM_LEDS_PER_STRIP))
#define BUFFOFF ((NBIS2SERIALPINS * 8) - 1)
#define AAA (0x00AA00AAL)
#define CCC (0x0000CCCCL)
#define FFF (0xF0F0F0F0L)
#define FFF2 (0xF0F0F0FL)



#ifdef HARDWARE_SCROLL
#define __HARDWARE_S
#define __HARDWARE_MAP
#endif

#ifdef _HARDWARE_SCROLL_MAP
#define __HARDWARE_S
#define __HARDWARE_MAP
#endif


#ifdef __HARDWARE_MAP
#define _LEDMAPPING
#endif
#ifdef __SOFTWARE_MAP
#define _LEDMAPPING
#endif
#ifdef __HARDWARE_MAP_PROGMEM
#define _LEDMAPPING
#endif

#ifdef USE_PIXELSLIB
#include "pixelslib.h"
#else
#include "___pixeltypes.h"
#endif

#if HARDWARESPRITES == 1
#include "hardwareSprite.h"
#endif

#define __delay (((NUM_LEDS_PER_STRIP * 125 * 8 * nb_components) /100000) +1 )

#include "framebuffer.h"
typedef union
{
    uint8_t bytes[16*8];
    uint32_t shorts[16*2];
    //uint32_t raw[2];
} Lines;

class I2SClocklessVirtualLedDriver;
struct OffsetDisplay
{
    int offsetx;
    int offsety;
    int panel_height;
    int panel_width;
    int image_height;
    int image_width;
    int window_width;
    int window_height;
    int _offx;
    int xc;
    int yc;
    float rotation;
    bool enableLoopx;
    bool enableLoopy;
    bool enableRotation;
    float scallingx;
    float scallingy;
        int _scallingx;
    int _scallingy;
    long _deltax;
    long _deltay;
    int _defaultvalue;

   // float _cos;
   // float _sin;
   int _cos;
   int  _sin;
    int _offy;
};

static const char *TAG = "I2SClocklessVirtualLedDriver";
static void IRAM_ATTR _I2SClocklessVirtualLedDriverinterruptHandler(void *arg);
 static void IRAM_ATTR transpose16x1_noinline2(unsigned char *A, uint8_t *B);
//static void IRAM_ATTR loadAndTranspose(uint8_t *ledt, OffsetDisplay offdisp, uint16_t *buffer, int ledtodisp, uint8_t *mapg, uint8_t *mapr, uint8_t *mapb, uint8_t *mapw, uint8_t *r_map, uint8_t *g_map, uint8_t *b_map);
static void IRAM_ATTR loadAndTranspose(I2SClocklessVirtualLedDriver * driver);
//static void IRAM_ATTR loadAndTranspose2(uint8_t *ledt, uint8_t **ledsstrips, uint16_t *buff, int ledtodisp, uint8_t *mapg, uint8_t *mapr, uint8_t *mapb, uint8_t *mapw);
//static void IRAM_ATTR transpose16x1_noinline22(uint32_t *A, uint8_t *B);
    static TaskHandle_t I2SClocklessVirtualLedDriver_dispTaskHandle = 0;
    static TaskHandle_t I2SClocklessVirtualLedDriver_returnTaskHandle = 0;
    static void showPixelsTask(void *pvParameters);
int interruptSource;

enum colorarrangment
{
    ORDER_GRBW,
    ORDER_RGB,
    ORDER_RBG,
    ORDER_GRB,
    ORDER_GBR,
    ORDER_BRG,
    ORDER_BGR,
};

enum displayMode
{
    NO_WAIT,
    WAIT,
    LOOP,
    LOOP_INTERUPT,
};

/*
class AllLeds
{

public:
    AllLeds() {}
    void init(uint8_t **ledspointerarraya = NULL)
    {
        ledspointerarray = ledspointerarraya;
    }

    //uint8_t &operator[](uint16_t);
    uint8_t &operator[](uint16_t i)
    {
        uint8_t strip = i / (NUM_LEDS_PER_STRIP * nb_components);
        uint8_t *offset = ledspointerarray[(strip % 2 == 0) ? strip + 1 : strip - 1] + (i % (NUM_LEDS_PER_STRIP * nb_components));
        return *(offset);
    }

private:
    uint8_t **ledspointerarray;
};
*/
/*
struct pixel
{
    uint8_t raw[nb_components];
    inline pixel &operator=(const pixel &rhs) __attribute__((always_inline))
    {
        raw[0] = rhs.raw[0];
        raw[1] = rhs.raw[1];
        raw[2] = rhs.raw[2];
        return *this;
    }
#ifdef USE_FASTLED
    inline pixel &operator=(const CRGB &rhs) __attribute__((always_inline))
    {
        raw[0] = rhs.raw[0];
        raw[1] = rhs.raw[1];
        raw[2] = rhs.raw[2];
        return *this;
    }
    inline pixel &operator=(const CHSV &rhs) __attribute__((always_inline))
    {
        CRGB temp;
        hsv2rgb_rainbow(rhs, temp);
        raw[0] = temp.raw[0];
        raw[1] = temp.raw[1];
        raw[2] = temp.raw[2];
        return *this;
    }
#endif
};
*/
/*
class AllLedsObjects
{

public:
    AllLedsObjects() {}
    void init(uint8_t **ledspointerarraya = NULL)
    {
        ledspointerarray = ledspointerarraya;
    }

    // pixel &operator[](uint16_t);
    pixel &operator[](uint16_t i)
    {

        uint8_t strip = i / (NUM_LEDS_PER_STRIP);
        uint8_t *offset = ledspointerarray[(strip % 2 == 0) ? strip + 1 : strip - 1] + (i % NUM_LEDS_PER_STRIP) * nb_components;
        return *((pixel *)offset);
    }

private:
    uint8_t **ledspointerarray;
};
*/
  
class I2SClocklessVirtualLedDriver
{

    struct I2SClocklessVirtualLedDriverDMABuffer
    {
        lldesc_t descriptor;
        uint8_t *buffer;
    };

    const int deviceBaseIndex[2] = {I2S0O_DATA_OUT0_IDX, I2S1O_DATA_OUT0_IDX};
    const int deviceClockIndex[2] = {I2S0O_BCK_OUT_IDX, I2S1O_BCK_OUT_IDX};
    const int deviceWordSelectIndex[2] = {I2S0O_WS_OUT_IDX, I2S1O_WS_OUT_IDX};
    const periph_module_t deviceModule[2] = {PERIPH_I2S0_MODULE, PERIPH_I2S1_MODULE};

public:
     Lines firstPixel[nb_components];
    i2s_dev_t *i2s;
    
    uint8_t __green_map[256];
    uint8_t __blue_map[256];
    uint8_t __red_map[256];
    uint8_t __white_map[256];
    uint8_t g_map[120];
    uint8_t r_map[120];
    uint8_t b_map[120];
     uint8_t * palette;
    intr_handle_t _gI2SClocklessDriver_intr_handle=NULL;
    uint8_t _brightness;
    int startleds;
   // int linewidth;
    float _gammar, _gammab, _gammag, _gammaw;
    OffsetDisplay _offsetDisplay, _defaultOffsetDisplay;
    volatile xSemaphoreHandle I2SClocklessVirtualLedDriver_sem = NULL;
    volatile xSemaphoreHandle I2SClocklessVirtualLedDriver_semSync = NULL;
    volatile xSemaphoreHandle I2SClocklessVirtualLedDriver_semDisp = NULL;
    volatile xSemaphoreHandle I2SClocklessVirtualLedDriver_waitDisp = NULL;
    bool isOffsetDisplay;
    bool isRunOnCore=false;
    int runCore;
    volatile long tims;
 #if CORE_DEBUG_LEVEL>=4
 uint32_t _times[NUM_LEDS_PER_STRIP];
 #endif
    frameBuffer * framebuff;
    bool useFrame=false;
   #ifdef __HARDWARE_MAP
         uint16_t * _hmap,*_defaulthmap;
        uint16_t * _hmapoff;
           void setHmap( uint16_t * map)
    {
        _defaulthmap=map;
    }
   
    #ifdef _HARDWARE_SCROLL_MAP
      uint16_t * _hmapscroll;
     //uint16_t * _hmaptmp;
    #endif

    #endif
 #ifdef _LEDMAPPING  
    uint16_t (*mapLed)(uint16_t led);


 inline void setMapLed(uint16_t (*newMapLed)(uint16_t led))
  {
     mapLed = newMapLed;
  }

#endif

/*
#ifdef MULTIPLE_LEDSBUFFER
    uint8_t *ledsstrips[120];
    uint8_t *ledsstripsorigin[120];
    AllLeds allleds;
    AllLedsObjects LEDS;
#endif
*/
    bool driverInit = false;

    /*
     This flag is used when using the NO_WAIT modeÒÒ
     */
    volatile bool isDisplaying = false;
    volatile bool isWaiting = true;
    volatile bool framesync = false;
    volatile bool wasWaitingtofinish = false;
    volatile int counti;

    I2SClocklessVirtualLedDriver(){};

    void setPins(int *Pins, int clock_pin, int latch_pin)
    {
        for (int i = 0; i < NBIS2SERIALPINS; i++)
        {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[Pins[i]], PIN_FUNC_GPIO);
            gpio_set_direction((gpio_num_t)Pins[i], (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
            gpio_matrix_out(Pins[i], deviceBaseIndex[I2S_DEVICE] + i + 8, false, false);
        }
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[latch_pin], PIN_FUNC_GPIO);
        gpio_set_direction((gpio_num_t)latch_pin, (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
        gpio_matrix_out(latch_pin, deviceBaseIndex[I2S_DEVICE] + NBIS2SERIALPINS + 8, false, false);
        gpio_set_direction((gpio_num_t)clock_pin, (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
        gpio_matrix_out(clock_pin, deviceClockIndex[I2S_DEVICE], false, false);
      

    }

    void setColorOrderPerStrip(int stripnumber, colorarrangment arr)
    {
        switch (arr)
        {
        case ORDER_RGB:

            r_map[stripnumber] = 0;
            g_map[stripnumber] = 1;
            b_map[stripnumber] = 2;
            break;
        case ORDER_RBG:
            r_map[stripnumber] = 0;
            g_map[stripnumber] = 2;
            b_map[stripnumber] = 1;

            break;
        case ORDER_GRB:
            r_map[stripnumber] = 1;
            g_map[stripnumber] = 0;
            b_map[stripnumber] = 2;
            break;
        case ORDER_GBR:
            r_map[stripnumber] = 2;
            g_map[stripnumber] = 0;
            b_map[stripnumber] = 1;

            break;
        case ORDER_BRG:
            r_map[stripnumber] = 1;
            g_map[stripnumber] = 2;
            b_map[stripnumber] = 0;

            break;
        case ORDER_BGR:
            r_map[stripnumber] = 2;
            g_map[stripnumber] = 1;
            b_map[stripnumber] = 0;
            break;
        case ORDER_GRBW:
            r_map[stripnumber] = 1;
            g_map[stripnumber] = 0;
            b_map[stripnumber] = 2;
            break;
        }
    }
    void setBrightness(int brightness)
    {
        _brightness = brightness;
        float tmp;
        for (int i = 0; i < 256; i++)
        {
            tmp = powf((float)(i)/ 255,  _gammag);
            __green_map[i] = (uint8_t)(tmp * brightness);
            tmp = powf((float)( i)/ 255,  _gammab);
            __blue_map[i] = (uint8_t)(tmp * brightness);
            tmp = powf((float)(i) / 255,  _gammar);
            __red_map[i] = (uint8_t)(tmp * brightness);
            tmp = powf((float)(i) / 255,  _gammaw);
            __white_map[i] = (uint8_t)(tmp * brightness);
        }
    }

    void setGamma(float gammar, float gammab, float gammag, float gammaw)
    {
        _gammag = gammag;
        _gammar = gammar;
        _gammaw = gammaw;
        _gammab = gammab;
        setBrightness(_brightness);
    }

    void setGamma(float gammar, float gammab, float gammag)
    {
        _gammag = gammag;
        _gammar = gammar;
        _gammab = gammab;
        setBrightness(_brightness);
    }
    void i2sInit()
    {
        
        if (I2S_DEVICE == 0)
        {
            i2s = &I2S0;
            periph_module_enable(PERIPH_I2S0_MODULE);
            interruptSource = ETS_I2S0_INTR_SOURCE;
            i2s_base_pin_index = I2S0O_DATA_OUT0_IDX;
        }
        else
        {
            i2s = &I2S1;
            periph_module_enable(PERIPH_I2S1_MODULE);
            interruptSource = ETS_I2S1_INTR_SOURCE;
            i2s_base_pin_index = I2S1O_DATA_OUT0_IDX;
        }
        i2sReset();
        i2sReset_DMA();
        i2sReset_FIFO();
        i2s->conf.tx_right_first = 0;

        // -- Set parallel mode
        i2s->conf2.val = 0;
        i2s->conf2.lcd_en = 1;
        i2s->conf2.lcd_tx_wrx2_en = 1; // 0 for 16 or 32 parallel output
        i2s->conf2.lcd_tx_sdx2_en = 0; // HN

        // -- Set up the clock rate and sampling
        i2s->sample_rate_conf.val = 0;
        i2s->sample_rate_conf.tx_bits_mod = 16; // Number of parallel bits/pins
        i2s->clkm_conf.val = 0;
        //i2s->sample_rate_conf.tx_bck_div_num = 1;
#ifdef DL_CLK
        // Serial.println("norml clock");
        i2s->clkm_conf.clka_en = 0;
        //rtc_clk_apll_enable(true, 31, 133,7, 1); //19.2Mhz 7 pins +1 latchrtc_clk_apll_enable(true, 31, 133,7, 1); //19.2Mhz 7 pins +1 latch

        // -- Data clock is computed as Base/(div_num + (div_b/div_a))
        //    Base is 80Mhz, so 80/(3+ 7/6) = 19.2Mhz

        i2s->clkm_conf.clkm_div_a = 6;   // CLOCK_DIVIDER_A;
        i2s->clkm_conf.clkm_div_b = 7;   //CLOCK_DIVIDER_B;
        i2s->clkm_conf.clkm_div_num = 3; //CLOCK_DIVIDER_N;

#else
        //Serial.println("precise clock");
        
       
        #ifndef _20_MHZ_CLK
        rtc_clk_apll_enable(true, 31, 133, 7, 1); //19.2Mhz 7 pins +1 latchrtc_clk_apll_enable(true, 31, 133,7, 1); //19.2Mhz 7 pins +1 latch
       #else
        rtc_clk_apll_enable(true, 0, 0, 8, 1); 
        #endif
        i2s->clkm_conf.clka_en = 1;
        i2s->clkm_conf.clkm_div_a = 1;   // CLOCK_DIVIDER_A;
        i2s->clkm_conf.clkm_div_b = 0;   //CLOCK_DIVIDER_B;
        i2s->clkm_conf.clkm_div_num = 1; //CLOCK_DIVIDER_N;
#endif
        i2s->fifo_conf.val = 0;
        i2s->fifo_conf.tx_fifo_mod_force_en = 1;
        i2s->fifo_conf.tx_fifo_mod = 1;  // 16-bit single channel data
        i2s->fifo_conf.tx_data_num = 32; //32; // fifo length
        i2s->fifo_conf.dscr_en = 1;      // fifo will use dma
       
        i2s->sample_rate_conf.tx_bck_div_num = 1;
        i2s->conf1.val = 0;
        i2s->conf1.tx_stop_en = 0;
        i2s->conf1.tx_pcm_bypass = 1;
 
        i2s->conf_chan.val = 0;
        i2s->conf_chan.tx_chan_mod = 1; // Mono mode, with tx_msb_right = 1, everything goes to right-channel

        i2s->timing.val = 0;
        //i2s->int_ena.val = 0;
        /*
        // -- Allocate i2s interrupt
        SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_DEVICE), I2S_OUT_EOF_INT_ENA_V,1, I2S_OUT_EOF_INT_ENA_S);
        SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_DEVICE), I2S_OUT_TOTAL_EOF_INT_ENA_V, 1, I2S_OUT_TOTAL_EOF_INT_ENA_S);
        SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_DEVICE), I2S_OUT_TOTAL_EOF_INT_ENA_V, 1, I2S_OUT_TOTAL_EOF_INT_ENA_S);
        */
      /*
       if(_gI2SClocklessDriver_intr_handle!=NULL)
            esp_intr_free(_gI2SClocklessDriver_intr_handle);
        ESP_LOGV(TAG,"setting interupt handler");
        esp_err_t e = esp_intr_alloc(interruptSource, ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM, &_I2SClocklessVirtualLedDriverinterruptHandler, this, &_gI2SClocklessDriver_intr_handle);
        if (e!=ESP_OK)
        {
            ESP_LOGE(TAG,"Impossible to create interupt allocatio n");
            return;
        }
        ESP_LOGV(TAG,"interupt handler set");

        */
        // -- Create a semaphore to block execution until all the controllers are done

        if (I2SClocklessVirtualLedDriver_sem == NULL)
        {
            I2SClocklessVirtualLedDriver_sem = xSemaphoreCreateBinary();
        }

        if (I2SClocklessVirtualLedDriver_semSync == NULL)
        {
            I2SClocklessVirtualLedDriver_semSync = xSemaphoreCreateBinary();
        }
        if (I2SClocklessVirtualLedDriver_semDisp == NULL)
        {
            I2SClocklessVirtualLedDriver_semDisp = xSemaphoreCreateBinary();
        }
    }

    void initDMABuffers()
    {
        DMABuffersTampon[0] = allocateDMABuffer((NUM_VIRT_PINS + 1) * nb_components * 8 * 3 * 2); //the buffers for the
        DMABuffersTampon[1] = allocateDMABuffer((NUM_VIRT_PINS + 1) * nb_components * 8 * 3 * 2);
        DMABuffersTampon[2] = allocateDMABuffer((NUM_VIRT_PINS + 1) * nb_components * 8 * 3 * 2);
        DMABuffersTampon[3] = allocateDMABuffer((NUM_VIRT_PINS + 1) * nb_components * 8 * 3 * 2);

        putdefaultlatch((uint16_t *)DMABuffersTampon[2]->buffer);
        putdefaultlatch((uint16_t *)DMABuffersTampon[3]->buffer);
        putdefaultlatch((uint16_t *)DMABuffersTampon[0]->buffer);
        putdefaultlatch((uint16_t *)DMABuffersTampon[1]->buffer);
        putdefaultones((uint16_t *)DMABuffersTampon[0]->buffer);
        putdefaultones((uint16_t *)DMABuffersTampon[1]->buffer);

    }




    void setPixel(uint32_t pos, uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
    {

        uint8_t *offset = leds + (pos << 2); //faster than doing * 4



        *(offset) = red;
        *(++offset) = green;
        *(++offset) = blue;
        *(++offset) = white;
    }

    void setPixel(uint32_t pos, uint8_t red, uint8_t green, uint8_t blue)
    {
        if (!driverInit)
        {
            //printf("Driver not initialized\n");
            return;
        }
      uint8_t *offset = leds + (pos << 1) + pos;


        if (nb_components == 3)
        {
           // uint8_t *offset = leds + (pos << 1) + pos;
            *(offset) = red;
            *(++offset) = green;
            *(++offset) = blue;
        }
        else
        {
            /*
                Code to transform RBG into RGBW thanks to @Jonathanese https://github.com/Jonathanese/NodeMCUPoleDriver/blob/master/LED_Framework.cpp
            */
            uint8_t W = MIN(red, green);
            W = MIN(W, blue);
            red = red - W;
            green = green - W;
            blue = blue - W;
            setPixel(pos, red, green, blue, W);
        }
    }


void setPalette(uint8_t * pal)
{
    palette=pal;
}
    OffsetDisplay getDefaultOffset()
    {
        return _defaultOffsetDisplay;
    }


 void waitDisplay()
 {
    if(isDisplaying == true ) //and __displayMode==NO_WAIT)
            {
                wasWaitingtofinish = true;
               // ESP_LOGD(TAG, "already displaying... wait");
               // tims=ESP.getCycleCount();
                //#ifndef _HARDWARE_SCROLL_MAP
                //const TickType_t xDelay = __delay ; 
                //#endif
                //#else
                const TickType_t xDelay = 60 ; 
                
                //#endif
                xSemaphoreTake(I2SClocklessVirtualLedDriver_waitDisp,xDelay);
               // ESP_LOGD(TAG, "back after:%.4fms",(float)(ESP.getCycleCount()-tims)/240000);
            
            }
    isDisplaying=true;
 }


void calculateOffsetDisplay(OffsetDisplay offdisp)
{
    #ifdef __HARDWARE_MAP
                if(offdisp.image_width==0 or offdisp.image_width==30000  )
        {
            offdisp.image_width=offdisp.panel_width;
        }
                if(offdisp.image_height==0 or offdisp.image_height==30000)
        {
            offdisp.image_height=offdisp.panel_height;
        }
        if(offdisp.window_width==0 or offdisp.window_width==30000)
        {

            offdisp.window_width=offdisp.image_width;
        }
                if(offdisp.window_height==0 or offdisp.window_height==30000)
        {
            offdisp.window_height=offdisp.image_height;
        }

        #ifdef _NOCROP
               offdisp._offx=  -offdisp.offsetx + 30 * offdisp.image_width;
         offdisp._offy= -offdisp.offsety + 30 * offdisp.image_height;
#else
               offdisp._offx=  -offdisp.offsetx + 30 * offdisp.window_width;
         offdisp._offy= -offdisp.offsety + 30 * offdisp.window_height;
         #endif
         
                  if(offdisp.scallingx<0.1)
            offdisp.scallingx=0.1;
               if(offdisp.scallingy<0.1)
            offdisp.scallingy=0.1;
            
         offdisp._cos=(int) (float)(128*cos (-offdisp.rotation)/offdisp.scallingx);
         
         offdisp._sin=(int) (float)(128*sin (-offdisp.rotation)/offdisp.scallingy);


            offdisp._scallingx=16/ offdisp.scallingx;
             offdisp._scallingy= 16/ offdisp.scallingy;
                offdisp._deltax=(offdisp.yc*offdisp._sin-offdisp.xc*offdisp._cos)+offdisp.xc*128;
         offdisp._deltay=-(offdisp.xc*offdisp._sin+offdisp.yc*offdisp._cos) +offdisp.yc*128;
         offdisp._defaultvalue=offdisp.image_width*offdisp.image_height+1;
            // Serial.println(offdisp._cos);
        _offsetDisplay = offdisp;
        _hmap=_defaulthmap;
    #ifdef _HARDWARE_SCROLL_MAP
     calculateMapping2(offdisp);
       
    
     _hmap= _hmapscroll;
      #else

      #ifndef HARDWARE_SCROLL
      calculateMapping(offdisp);
       
      #endif
      #endif
      #endif
}

    void showPixels(displayMode dispmode, OffsetDisplay offdisp)
    {
        waitDisplay();
        #ifdef __HARDWARE_MAP
        _offsetDisplay=offdisp;
   isOffsetDisplay=true;
        __displayMode=dispmode;
        if(useFrame)
        {
            leds =framebuff->getFrametoDisplay();
        }
        else
        {
        leds = saveleds;
        }
        __showPixels();
        #endif
    }


    void showPixels(uint8_t *newleds)
    {

        waitDisplay();
         #ifdef __HARDWARE_MAP
          _hmap=_defaulthmap;
        #endif

        leds = newleds;
         __displayMode=WAIT;
        _offsetDisplay=_defaultOffsetDisplay;
        __showPixels();
  
    }
void showPixels(OffsetDisplay offdisp)
{
            waitDisplay();
         #ifdef __HARDWARE_MAP
                 _offsetDisplay=offdisp;
   isOffsetDisplay=true;
        
        if(useFrame)
        {
            leds =framebuff->getFrametoDisplay();
            __displayMode=NO_WAIT;
        }
        else
        {
        leds = saveleds;
         __displayMode=WAIT;
        }
        
       
    __showPixels();
    #endif
}
 void showPixels(displayMode dispmode,uint8_t *newleds,OffsetDisplay offd)
 {
        waitDisplay();
        #ifdef __HARDWARE_MAP
          _offsetDisplay=offd;
   isOffsetDisplay=true;
        __displayMode=dispmode;
        leds=newleds;
        __showPixels();
        // calculateMapping(_defaultOffsetDisplay);
       // _offsetDisplay = _defaultOffsetDisplay;
        #endif
 }
    void showPixels(displayMode dispmode)
    {
        
    waitDisplay();
                 #ifdef __HARDWARE_MAP
          _hmap=_defaulthmap;
        #endif
        //leds = newleds;
        leds = saveleds;
        __displayMode=dispmode;
        _offsetDisplay=_defaultOffsetDisplay;
          __showPixels();
    }
 void showPixels(uint8_t *newleds,OffsetDisplay offd)
 {
        waitDisplay();
        #ifdef __HARDWARE_MAP
                _offsetDisplay=offd;
   isOffsetDisplay=true;
        __displayMode=WAIT;
        leds=newleds;
        __showPixels();

        #endif
 }

        void showPixels(displayMode dispmode,uint8_t *newleds)
    {

        
    waitDisplay();

                 #ifdef __HARDWARE_MAP
          _hmap=_defaulthmap;
        #endif
        leds = newleds;
        __displayMode=dispmode;
        _offsetDisplay=_defaultOffsetDisplay;
          __showPixels();

    }


  void showPixels()
    {
        waitDisplay();
                 #ifdef __HARDWARE_MAP
          _hmap=_defaulthmap;
        #endif
        if(useFrame)
        {

                        leds =framebuff->getFrametoDisplay();
            __displayMode=NO_WAIT;
           // __displayMode=WAIT;
        }
        else
        {
  
         leds = saveleds;
        __displayMode=WAIT;
        }
        _offsetDisplay=_defaultOffsetDisplay;
        
            __showPixels();
        
    }

void _runShowPixelsOnCore()
{
   
    if (I2SClocklessVirtualLedDriver_returnTaskHandle == 0) {

        I2SClocklessVirtualLedDriver_returnTaskHandle = xTaskGetCurrentTaskHandle();
        
        // -- Trigger the show task
        xTaskNotifyGive(I2SClocklessVirtualLedDriver_dispTaskHandle);

        // -- Wait to be notified that it's done
        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //delay(100);
        //interrupts();
        I2SClocklessVirtualLedDriver_returnTaskHandle = 0;
    }
}



void disableShowPixelsOnCore()
{
    if(I2SClocklessVirtualLedDriver_dispTaskHandle)
    {
        vTaskDelete(I2SClocklessVirtualLedDriver_dispTaskHandle);
    }
    if(_gI2SClocklessDriver_intr_handle!=NULL)
  {
            esp_intr_free(_gI2SClocklessDriver_intr_handle);
       
  }
  _gI2SClocklessDriver_intr_handle=NULL;
    runCore=0;
    isRunOnCore= false;
}
void enableShowPixelsOnCore(int corenum)
{
    if(corenum > 1)
    {
        ESP_LOGE(TAG,"enableShowPixelsOnCore error corenum > 1 core should be 0 or 1");
        return;
    }
    if(!driverInit)
    {
        ESP_LOGE(TAG,"Driver not initiated this will have no effect ... it will be executed at first call");
        return;
    }
    if(I2SClocklessVirtualLedDriver_dispTaskHandle)
    {
        vTaskDelete(I2SClocklessVirtualLedDriver_dispTaskHandle);
    }
    runCore=corenum;
    isRunOnCore= true;
        xTaskCreatePinnedToCore(showPixelsTask, "showPixelsTask", 2000, this,3, &I2SClocklessVirtualLedDriver_dispTaskHandle, corenum);
}

void  __showPixels()
{
 if(isRunOnCore)
 {
        if(!I2SClocklessVirtualLedDriver_dispTaskHandle)
        {
            ESP_LOGI(TAG,"No running core defined, rexecuting enable");
            enableShowPixelsOnCore(runCore);
            vTaskDelay(10);
            // ___showPixels();
        }
        //else
       // {
           // ESP_LOGI(TAG,"No running on core;%d",runCore);
             _runShowPixelsOnCore();
        //}
 }
 else
 {
    ___showPixels();
 }
}

    void ___showPixels()
    {
       // leddt=leds;
        
  if(_gI2SClocklessDriver_intr_handle==NULL)
  {
            //esp_intr_free(_gI2SClocklessDriver_intr_handle);
        ESP_LOGV(TAG,"setting interupt handler");
        esp_err_t e = esp_intr_alloc(interruptSource, ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM, &_I2SClocklessVirtualLedDriverinterruptHandler, this, &_gI2SClocklessDriver_intr_handle);
        if (e!=ESP_OK)
        {
            ESP_LOGE(TAG,"Impossible to create interupt allocation");
            return;
        }
        ESP_LOGV(TAG,"interupt handler set on core %d",xPortGetCoreID() );
  }
 
        
//ESP_LOGD(TAG,"Running on core:%d",xPortGetCoreID() );
#ifdef __HARDWARE_MAP
 //ESP_LOGI(TAG,"JJJ");
        if(isOffsetDisplay)
        {
           
                calculateOffsetDisplay(_offsetDisplay);
               //  _hmap=_defaulthmap;
        }
        isOffsetDisplay = false;
           _hmapoff=_hmap;
        
    #endif
#if HARDWARESPRITES == 1
        memset(target, 0, NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8 * 2);
        for (int i = 0; i < 8; i++)
        {
            sprites[i].reorder(_offsetDisplay.panel_width, _offsetDisplay.panel_height);
        }
#endif
        if (!driverInit)
        {
            ESP_LOGE(TAG,"Driver not initialized");
            return;
        }

        if (leds == NULL)
        {
            ESP_LOGE(TAG, "no leds buffer defined");
            return;
        }

        ledToDisplay = 0;
        transpose = true;

        DMABuffersTampon[0]->descriptor.qe.stqe_next = &(DMABuffersTampon[1]->descriptor);
        DMABuffersTampon[1]->descriptor.qe.stqe_next = &(DMABuffersTampon[0]->descriptor);
        DMABuffersTampon[2]->descriptor.qe.stqe_next = &(DMABuffersTampon[0]->descriptor);
        DMABuffersTampon[3]->descriptor.qe.stqe_next = 0;
        dmaBufferActive = 0;
       // loadAndTranspose(leds, _offsetDisplay, (uint16_t *)DMABuffersTampon[0]->buffer, ledToDisplay, __green_map, __red_map, __blue_map, __white_map, r_map, g_map, b_map);
       loadAndTranspose(this);

 //__displayMode=dispmode;
        dmaBufferActive = 1;
         isDisplaying = true;
        i2sStart(DMABuffersTampon[2]);
            if (__displayMode == WAIT)
        {
            isWaiting = true;
            if(  I2SClocklessVirtualLedDriver_sem==NULL)
            I2SClocklessVirtualLedDriver_sem=xSemaphoreCreateBinary();
            xSemaphoreTake(I2SClocklessVirtualLedDriver_sem, portMAX_DELAY);
        }
        else{
        isWaiting = false;
       // isDisplaying = true;
        }

        //vTaskDelay(1/portTICK_PERIOD_MS);
        //delay(1);
        #if CORE_DEBUG_LEVEL>=4
        uint32_t total=0;
        for(int _time=0;_time<NUM_LEDS_PER_STRIP;_time++)
        {
            total+=_times[_time];
        }
        ESP_LOGV(TAG,"interput time: %d cycles %.2fus",total,(float)total/240/NUM_LEDS_PER_STRIP);
        #endif
    }

    //list of the leds strips

/*
Calculate the Mapping
*/

#ifdef _SOFT_MAP_CALC
int remap(int val, OffsetDisplay off)
{
    int xr,yr;//,newx,newy;
    int xe=(val % off.panel_width);//+off._offx);//%off.window_width;
    int ye=(val/off.panel_width);//+off._offy);//%off.window_height;  
   

         xr=((xe-off.xc)*off._cos-(ye-off.yc)*off._sin)/128+off.xc+off._offx;
         yr=((xe-off.xc)*off._sin+(ye-off.yc)*off._cos)/128+off.yc+off._offy;
         //._deltax=(off.yc*off.sin-off.xc*off._cos)/128+off.xc;
         //._deltay=-(off.xc*off._sin+off.yc*off._cos)/128+off.yc;

         //xr=(xe*off._cos-ye*off._sin)/128+off._deltax;
         //yr=(xe*off._sin+ye*off._cos)/128+off._deltay;
   /*
    }
    //else
    {
        //xr=(xe*off._cos)*20/128/off.scallingx;
       // yr=(ye*off._cos)*20/128/off.scallingx;
       xr=xe*off._scallingx/16;
       yr=ye*off._scallingy/16;
    }*/
    /*
    if(off.enableLoopx)
    {
        xr+=off._offx;
    }
    else
    {
        xr-=off.offsetx;
        if(xr<0 or xr>=off.image_width)
            return  off.image_width*off.image_height+1;
    }
    if(off.enableLoopy)
    {
        yr+=off._offy;
    }
    else
    {
        yr-=off.offsety;
        if(yr<0 or yr>=off.image_height)
            return  off.image_width*off.image_height+1;
    }*/
    return xr%off.image_width+(yr%off.image_height)*off.image_width;
}//

#else
int remap(int val, OffsetDisplay off)
{
    int xr,yr;//,newx,newy;
    int xe=(val % off.panel_width);//+off._offx);//%off.window_width;
    int ye=(val/off.panel_width);//+off._offy);//%off.window_height;  
#ifdef _ROTATION
             xr=((xe-off.xc)*off._cos-(ye-off.yc)*off._sin)/128+off.xc;
            yr=((xe-off.xc)*off._sin+(ye-off.yc)*off._cos)/128+off.yc;
#else
       xr=xe;//*off._scallingx/16;
       yr=ye;//*off._scallingy/16;
#endif
#ifdef _LOOPX
    xr+=off._offx;
#else
        xr-=off.offsetx;
        if(xr<0 or xr>=off.image_width)
        {
            return  off.image_width*off.image_height+1;
        }
#endif
#ifdef _LOOPY
    yr+=off._offy;
#else
        yr-=off.offsety;
        if(yr<0 or yr>=off.image_height)
        {
            return  off.image_width*off.image_height+1;
        }
#endif
return xr%off.image_width+(yr%off.image_height)*off.image_width;
}
#endif



#ifdef __HARDWARE_MAP
     //   _offsetDisplay = offdisp;
#ifdef _HARDWARE_SCROLL_MAP
/*
void calculateMapping2(OffsetDisplay off)
{
    if(!_hmapscroll)
    {
      ESP_LOGE(TAG, "No more memory\n");
        return;
    }

         for(uint16_t leddisp=0;leddisp<NUM_LEDS_PER_STRIP*NBIS2SERIALPINS*8;leddisp++)
         {
             _hmapscroll[leddisp]=remap(_defaulthmap[leddisp],off); 
         }
}
*/
void calculateMapping2(OffsetDisplay off)
{
        int xr,yr;//,newx,newy;
            if(!_hmapscroll)
    {
      ESP_LOGE(TAG, "No more memory\n");
        return;
    }
    for(uint16_t leddisp=0;leddisp<NUM_LEDS_PER_STRIP*NBIS2SERIALPINS*8;leddisp++)
         {
            int xe=(_defaulthmap[leddisp] % off.panel_width);//+off._offx);//%off.window_width;
            int ye=(_defaulthmap[leddisp]/off.panel_width);//+off._offy);//%off.window_height;  

        #ifdef _SOFT_MAP_CALC
             xr=((xe-off.xc)*off._cos-(ye-off.yc)*off._sin)/128+off.xc;//+off._offx;
             yr=((xe-off.xc)*off._sin+(ye-off.yc)*off._cos)/128+off.yc;//+off._offy;
    if(off.enableLoopx)
    {
        xr+=off._offx;
    }
    else
    {
        xr-=off.offsetx;
        if(xr<0 or xr>=off.image_width)
        {
            _hmapscroll[leddisp]=  off.image_width*off.image_height+1;
                            continue;
        }
    }
    if(off.enableLoopy)
    {
        yr+=off._offy;
    }
    else
    {
        yr-=off.offsety;
        if(yr<0 or yr>=off.image_height)
        {
            _hmapscroll[leddisp]=  off.image_width*off.image_height+1;
                            continue;
        }
    }

_hmapscroll[leddisp]=(xr%off.image_width+(yr%off.image_height)*off.image_width);
        #else
                #ifdef _ROTATION
                            xr=((xe-off.xc)*off._cos-(ye-off.yc)*off._sin)/128+off.xc;
                            yr=((xe-off.xc)*off._sin+(ye-off.yc)*off._cos)/128+off.yc;

                #else
                    xr=xe;//*off._scallingx/16;
                    yr=ye;//*off._scallingy/16;
                #endif
                #ifdef _LOOPX
                    xr+=off._offx;
                #else
                        xr-=off.offsetx;
                        if(xr<0 or xr>=off.image_width)
                        {
                            _hmapscroll[leddisp]=  off.image_width*off.image_height+1;
                            continue;
                        }
                #endif
                #ifdef _LOOPY
                    yr+=off._offy;
                #else
                        yr-=off.offsety;
                        if(yr<0 or yr>=off.image_height)
                        {
                            _hmapscroll[leddisp]=off.image_width*off.image_height+1;
                            continue;
                        }
                #endif
                _hmapscroll[leddisp]=xr%off.image_width+(yr%off.image_height)*off.image_width;

        #endif
         }
}
#endif
void calculateMapping(OffsetDisplay off)
{
      if(!_defaulthmap)
      {
        ESP_LOGE(TAG, "No more memory\n");
        return;
      }
       uint16_t offset2=0;
         for(uint16_t leddisp=0;leddisp<NUM_LEDS_PER_STRIP;leddisp++)
            {
                uint16_t led_tmp=NUM_LEDS_PER_STRIP+leddisp;

                 for (uint16_t i = 0; i < NBIS2SERIALPINS; i++)
                 {

                        #ifdef __HARDWARE_S
                         _defaulthmap[offset2]=remap(mapLed(led_tmp),off);

                         #else
                          _defaulthmap[offset2]=remap(mapLed(led_tmp),off)*_palette_size;
                         #endif
                         led_tmp+=I2S_OFF_MAP;
                         offset2++;
                }
                 led_tmp-=I2S_OFF3_MAP;

               for (uint16_t i = 0; i < NBIS2SERIALPINS; i++)
                 {

                        #ifdef __HARDWARE_S
                         _defaulthmap[offset2]=remap(mapLed(led_tmp),off);

                         #else
                          _defaulthmap[offset2]=remap(mapLed(led_tmp),off)*_palette_size;
                         #endif
                         led_tmp+=I2S_OFF_MAP;
                         offset2++;
                }
                 led_tmp-=I2S_OFF4_MAP;
                  for (uint16_t i = 0; i < NBIS2SERIALPINS; i++)
                 {

                        #ifdef __HARDWARE_S
                         _defaulthmap[offset2]=remap(mapLed(led_tmp),off);

                         #else
                          _defaulthmap[offset2]=remap(mapLed(led_tmp),off)*_palette_size;
                         #endif
                         led_tmp+=I2S_OFF_MAP;
                         offset2++;
                }
                led_tmp-=I2S_OFF3_MAP;
                               for (uint16_t i = 0; i < NBIS2SERIALPINS; i++)
                 {
                        #ifdef __HARDWARE_S
                         _defaulthmap[offset2]=remap(mapLed(led_tmp),off);

                         #else
                          _defaulthmap[offset2]=remap(mapLed(led_tmp),off)*_palette_size;
                         #endif
                         led_tmp+=I2S_OFF_MAP;
                         offset2++;
                }
                 led_tmp-=I2S_OFF4_MAP;
                 for (uint16_t i = 0; i < NBIS2SERIALPINS; i++)
                 {
                        #ifdef __HARDWARE_S
                         _defaulthmap[offset2]=remap(mapLed(led_tmp),off);

                         #else
                          _defaulthmap[offset2]=remap(mapLed(led_tmp),off)*_palette_size;
                         #endif
                         led_tmp+=I2S_OFF_MAP;
                         offset2++;
                }
                 led_tmp-=I2S_OFF3_MAP;

               for (uint16_t i = 0; i < NBIS2SERIALPINS; i++)
                 {

                        #ifdef __HARDWARE_S
                         _defaulthmap[offset2]=remap(mapLed(led_tmp),off);

                         #else
                          _defaulthmap[offset2]=remap(mapLed(led_tmp),off)*_palette_size;
                         #endif
                         led_tmp+=I2S_OFF_MAP;
                         offset2++;
                }
                 led_tmp-=I2S_OFF4_MAP;
                  for (uint16_t i = 0; i < NBIS2SERIALPINS; i++)
                 {

                        #ifdef __HARDWARE_S
                         _defaulthmap[offset2]=remap(mapLed(led_tmp),off);

                         #else
                          _defaulthmap[offset2]=remap(mapLed(led_tmp),off)*_palette_size;
                         #endif
                         led_tmp+=I2S_OFF_MAP;
                         offset2++;
                }
                led_tmp-=I2S_OFF3_MAP;
                               for (uint16_t i = 0; i < NBIS2SERIALPINS; i++)
                 {

                        #ifdef __HARDWARE_S
                         _defaulthmap[offset2]=remap(mapLed(led_tmp),off);

                         #else
                          _defaulthmap[offset2]=remap(mapLed(led_tmp),off)*_palette_size;
                         #endif
                         led_tmp+=I2S_OFF_MAP;
                         offset2++;
                }
                 led_tmp-=I2S_OFF4_MAP;
            }
}

#endif

#ifdef USE_FASTLED
void initled(CRGB *leds, int *Pinsq, int clock_pin, int latch_pin)
{
    initled((uint8_t *)leds,Pinsq, clock_pin, latch_pin);

}
#endif

void initled(Pixel *leds, int *Pinsq, int clock_pin, int latch_pin)
{
    initled((uint8_t *)leds,Pinsq, clock_pin, latch_pin);

}


    void initled(uint8_t *leds, int *Pinsq, int clock_pin, int latch_pin)
    {
        ESP_LOGI(TAG,"Start driver");
        driverInit = false;
        isOffsetDisplay=false;
#ifdef MULTIPLE_LEDSBUFFER
        allleds.init(ledsstripsorigin);
        LEDS.init(ledsstripsorigin);
#endif
        /*
        switch(cArr)
        {
            case ORDER_RGB:
                nb_components=3;
                p_r=0;
                p_g=1;
                p_b=2;
                break;
            case ORDER_RBG:
                nb_components=3;
                p_r=0;
                p_g=2;
                p_b=1;
                break;
            case ORDER_GRB:
                nb_components=3;
                p_r=1;
                p_g=0;
                p_b=2;
                break;
            case ORDER_GBR:
                nb_components=3;
                p_r=2;
                p_g=0;
                p_b=1;
                break;
            case ORDER_BRG:
                nb_components=3;
                p_r=1;
                p_g=2;
                p_b=0;
                break;
            case ORDER_BGR:
                nb_components=3;
                p_r=2;
                p_g=1;
                p_b=0;
                break;
            case ORDER_GRBW:
                nb_components=4;
                p_r=1;
                p_g=0;
                p_b=2;
                break;
        }
*/
        _gammab = 1;
        _gammar = 1;
        _gammag = 1;
        _gammaw = 1;
        setBrightness(255);
        startleds = 0;
        dmaBufferCount = 2;
        //linewidth = NUM_LEDS_PER_STRIP;
        this->num_led_per_strip = NUM_LEDS_PER_STRIP;
         ESP_LOGD(TAG,"offset initiation");
        _offsetDisplay.offsetx = 0;
        _offsetDisplay.offsety = 0;
        _offsetDisplay.rotation=0;
        _offsetDisplay._cos=128;
          _offsetDisplay._sin=0;
       _offsetDisplay.panel_width = 30000;
        _offsetDisplay.panel_height = 30000; //maximum
         _offsetDisplay.image_height = 30000;
        _offsetDisplay.image_width = 30000; //maximum
        _offsetDisplay.window_height = 30000;
        _offsetDisplay.window_width = 30000;
        _offsetDisplay._offx=  0;//_offsetDisplay.offsetx + 4 * _offsetDisplay.window_width;
         _offsetDisplay._offy=0;// _offsetDisplay.offsety + 4 * _offsetDisplay.window_height;
      _offsetDisplay.enableLoopx=false;
      _offsetDisplay.enableLoopy=false;
      _offsetDisplay.enableRotation=false;
       _offsetDisplay.scallingx=1;
        _offsetDisplay.scallingy=1;
               _offsetDisplay._scallingx=16;
        _offsetDisplay._scallingy=16;

        _defaultOffsetDisplay = _offsetDisplay;
        __defaultDisplayMode = WAIT;

        this->leds = leds;
        this->saveleds = leds;
memset( firstPixel[0].bytes,0,16*8);
memset( firstPixel[1].bytes,0,16*8);
memset( firstPixel[2].bytes,0,16*8);
    firstPixel[0].bytes[16+NBIS2SERIALPINS] = 255;
    firstPixel[1].bytes[16+NBIS2SERIALPINS] = 255;
    firstPixel[2].bytes[16+NBIS2SERIALPINS] = 255;
#if nb_components > 3
    firstPixel[3].bytes[16+NBIS2SERIALPINS] = 255;
    
#endif

        //isRunOnCore=false;
        runCore = 3;
#if HARDWARESPRITES == 1
       // Serial.println(NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8);
        target = (uint16_t *)malloc(NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8 * 2 + 2);
#endif

#ifdef __HARDWARE_MAP
    ESP_LOGD(TAG,"creating map array");
    _defaulthmap=(uint16_t *)malloc(  NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8 * 2+2);
    if(!_defaulthmap)
    {
        Serial.printf("no memory\n");
    }
    else
    {
        ESP_LOGD(TAG,"calculate mapping");
        calculateMapping(_defaultOffsetDisplay);
    }
   #ifdef  _HARDWARE_SCROLL_MAP
          _hmapscroll=(uint16_t *)malloc(  NUM_LEDS_PER_STRIP * NBIS2SERIALPINS * 8 * 2+2);
            if(!_hmapscroll)
            {
                Serial.printf("no memory\n");
            }
   #endif
#endif
ESP_LOGD(TAG,"semaphore init");
                if(I2SClocklessVirtualLedDriver_waitDisp==NULL)
                {
                    I2SClocklessVirtualLedDriver_waitDisp = xSemaphoreCreateCounting(10,0);
                }
        //this->num_strips=num_strips;
        this->dmaBufferCount = dmaBufferCount;
        ESP_LOGD(TAG,"Pins initiation");
        setPins(Pinsq, clock_pin, latch_pin);
         ESP_LOGD(TAG,"I2S init");
        i2sInit();
         ESP_LOGD(TAG,"DMA initiation");
        initDMABuffers();
        ESP_LOGD(TAG,"End DMA initiation");
        driverInit = true;
         ESP_LOGI(TAG,"driver initiated");
    }


 void initled(frameBuffer * framb, int *Pinsq, int clock_pin, int latch_pin)
 {
        framebuff=framb;
        useFrame=true;
        initled(framb->frames[0],  Pinsq,  clock_pin,  latch_pin);

 }
    //private:
    volatile int dmaBufferActive = 0;
    volatile bool wait;
    displayMode __displayMode,__defaultDisplayMode;
    volatile int ledToDisplay;
    // volatile int oo=0;   
    uint8_t *leds,*saveleds;

    int dmaBufferCount = 2; //we use two buffers
    volatile bool transpose = false;

    volatile int num_strips;
    volatile int num_led_per_strip;
    //int clock_pin;
    int brigthness;

    int i2s_base_pin_index;

    //int nb_components;

    I2SClocklessVirtualLedDriverDMABuffer **DMABuffersTransposed = NULL;
    //buffer array for the regular way
    I2SClocklessVirtualLedDriverDMABuffer *DMABuffersTampon[4];

    I2SClocklessVirtualLedDriverDMABuffer *allocateDMABuffer(int bytes)
    {
        I2SClocklessVirtualLedDriverDMABuffer *b = (I2SClocklessVirtualLedDriverDMABuffer *)heap_caps_malloc(sizeof(I2SClocklessVirtualLedDriverDMABuffer), MALLOC_CAP_DMA);
        if (!b)
        {
            ESP_LOGE(TAG, "No more memory\n");
            return NULL;
        }

        b->buffer = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_DMA);
        if (!b->buffer)
        {
            ESP_LOGE(TAG, "No more memory\n");
            return NULL;
        }
        memset(b->buffer, 0, bytes);

        b->descriptor.length = bytes;
        b->descriptor.size = bytes;
        b->descriptor.owner = 1;
        b->descriptor.sosf = 1;
        b->descriptor.buf = b->buffer;
        b->descriptor.offset = 0;
        b->descriptor.empty = 0;
        b->descriptor.eof = 1;
        b->descriptor.qe.stqe_next = 0;

        return b;
    }

    void i2sReset_DMA()
    {

        (&I2S0)->lc_conf.out_rst = 1;
        (&I2S0)->lc_conf.out_rst = 0;
    }

    void i2sReset_FIFO()
    {

        (&I2S0)->conf.tx_fifo_reset = 1;
        (&I2S0)->conf.tx_fifo_reset = 0;
    }

    static void i2sStop(I2SClocklessVirtualLedDriver *cont)
    {

        //delay(1);
        esp_intr_disable(cont->_gI2SClocklessDriver_intr_handle);
        ets_delay_us(16);
        (&I2S0)->conf.tx_start = 0;
        while ((&I2S0)->conf.tx_start == 1)
        {
        }

        cont->i2sReset();

        cont->isDisplaying = false;
       // cont->leds=cont->saveleds;
        /*
         We have finished to display the strips
         */
        //ets_delay_us(1000);
        if(cont->wasWaitingtofinish == true)//and cont->__displayMode==NO_WAIT 
        {
            cont->wasWaitingtofinish=false;
             xSemaphoreGive(cont->I2SClocklessVirtualLedDriver_waitDisp);
        }
        if (cont->isWaiting)
        {
           // printf("on debloqu\n");
            xSemaphoreGive(cont->I2SClocklessVirtualLedDriver_sem);
        }
       // printf("hehe\n");
       
    }

    void putdefaultlatch(uint16_t *buff)
    {
        //printf("dd%d\n",NBIS2SERIALPINS);
        uint16_t mask1 = 1 << NBIS2SERIALPINS;
        for (int i = 0; i < 24 * nb_components; i++)
        {
            buff[NUM_VIRT_PINS + i * (NUM_VIRT_PINS + 1) - 1 - 5] = mask1; //0x8000;
            //buff[NUM_VIRT_PINS+i*(NUM_VIRT_PINS+1)]=0x02;
        }
    }

    void putdefaultones(uint16_t *buff)
    {

        uint16_t mas = 0xFFFF & (~(0xffff << (NBIS2SERIALPINS)));
        //printf("mas%d\n",mas);
        for (int j = 0; j < 8 * nb_components; j++)
        {

            buff[1 + j * (3 * (NUM_VIRT_PINS + 1))] = 0xFFFF;
            buff[0 + j * (3 * (NUM_VIRT_PINS + 1))] = mas;
            buff[3 + j * (3 * (NUM_VIRT_PINS + 1))] = mas;
            buff[2 + j * (3 * (NUM_VIRT_PINS + 1))] = mas;
            buff[5 + j * (3 * (NUM_VIRT_PINS + 1))] = mas;
            buff[4 + j * (3 * (NUM_VIRT_PINS + 1))] = mas;
            buff[7 + j * (3 * (NUM_VIRT_PINS + 1))] = mas;
            buff[6 + j * (3 * (NUM_VIRT_PINS + 1))] = mas;
        }
    }

    /*
     Transpose the pixel, as the function is static and all the variables are not static or global, we need to provide all of them.
     */

  

    void i2sStart(I2SClocklessVirtualLedDriverDMABuffer *startBuffer)
    {

        i2sReset();
        framesync = false;
        counti = 0;

        (&I2S0)->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;

        (&I2S0)->out_link.addr = (uint32_t) & (startBuffer->descriptor);

        (&I2S0)->out_link.start = 1;

        (&I2S0)->int_clr.val = (&I2S0)->int_raw.val;

        (&I2S0)->int_clr.val = (&I2S0)->int_raw.val;
        (&I2S0)->int_ena.val = 0;

        /*
         If we do not use the regular showpixels, then no need to activate the interupt at the end of each pixels
         */
        //if(transpose)
        (&I2S0)->int_ena.out_eof = 1;

        (&I2S0)->int_ena.out_total_eof = 1;
        esp_intr_enable(_gI2SClocklessDriver_intr_handle);

        //We start the I2S
        (&I2S0)->conf.tx_start = 1;

        //Set the mode to indicate that we've started
        isDisplaying = true;
    }

    void i2sReset()
    {
        const unsigned long lc_conf_reset_flags = I2S_IN_RST_M | I2S_OUT_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
        (&I2S0)->lc_conf.val |= lc_conf_reset_flags;
        (&I2S0)->lc_conf.val &= ~lc_conf_reset_flags;
        const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
        (&I2S0)->conf.val |= conf_reset_flags;
        (&I2S0)->conf.val &= ~conf_reset_flags;
    }

    // static void IRAM_ATTR interruptHandler(void *arg);
};


static void IRAM_ATTR _I2SClocklessVirtualLedDriverinterruptHandler(void *arg)
{

    // REG_WRITE(I2S_INT_CLR_REG(0), (REG_READ(I2S_INT_RAW_REG( 0 )) & 0xffffffc0) | 0x3f);
    //return;
    I2SClocklessVirtualLedDriver *cont = (I2SClocklessVirtualLedDriver *)arg;

    if (GET_PERI_REG_BITS(I2S_INT_ST_REG(I2S_DEVICE), I2S_OUT_EOF_INT_ST_S, I2S_OUT_EOF_INT_ST_S))
    {
        cont->framesync = !cont->framesync;

        if (((I2SClocklessVirtualLedDriver *)arg)->transpose)
        {
            cont->ledToDisplay++;
            if (cont->ledToDisplay < cont->num_led_per_strip)
            {

               loadAndTranspose(cont);

                if (cont->ledToDisplay == (cont->num_led_per_strip - 3)) //here it's not -1 because it takes time top have the change into account and it reread the buufer
                {
                    cont->DMABuffersTampon[cont->dmaBufferActive]->descriptor.qe.stqe_next = &(cont->DMABuffersTampon[3]->descriptor);
                }
                cont->dmaBufferActive = (cont->dmaBufferActive + 1) % 2;
            }
        }
        else
        {
            if (cont->framesync)
            {
                portBASE_TYPE HPTaskAwoken = 0;
                xSemaphoreGiveFromISR(cont->I2SClocklessVirtualLedDriver_semSync, &HPTaskAwoken);
                if (HPTaskAwoken == pdTRUE)
                    portYIELD_FROM_ISR();
            }
        }
    }

    if (GET_PERI_REG_BITS(I2S_INT_ST_REG(I2S_DEVICE), I2S_OUT_TOTAL_EOF_INT_ST_S, I2S_OUT_TOTAL_EOF_INT_ST_S))
    {


        cont->i2sStop(cont);

    }
    REG_WRITE(I2S_INT_CLR_REG(0), (REG_READ(I2S_INT_RAW_REG(0)) & 0xffffffc0) | 0x3f);
}


 static inline __attribute__((always_inline))  void IRAM_ATTR transpose16x1_noinline2(unsigned char *A, uint8_t *B)
{

   uint32_t x, y,t;
#if NBIS2SERIALPINS >= 8
    uint32_t  x1, y1;
#endif
    uint32_t aa,cc,ff;
    uint32_t ff2;
    aa=AAA;
    cc=CCC;
    ff=FFF;
   ff2=FFF2;
   //ff2=ff>>4;
   //int vpin=0;
  //while(vpin++<8)
  //for(int vpin=0;vpin<2;vpin++)
  //{

   // for(int col=0;col<nb_components;col++)
   // {
    y = *(unsigned int *)(A);
#if NBIS2SERIALPINS >= 4
    x = *(unsigned int *)(A + 4);
#else
   x = 0;
#endif
#if NBIS2SERIALPINS >= 8
    y1 = *(unsigned int *)(A + 8);
    #if NBIS2SERIALPINS >= 12
          x1 = *(unsigned int *)(A + 12);
      #else
         x1 = 0;
      #endif

#endif


    // pre-transform x
#if NBIS2SERIALPINS >= 4
    t = (x ^ (x >> 7)) & aa;
    x = x ^ t ^ (t << 7);
    t = (x ^ (x >> 14)) & cc;
    x = x ^ t ^ (t << 14);
#endif
#if NBIS2SERIALPINS >= 12
    t = (x1 ^ (x1 >> 7)) & aa;
    x1 = x1 ^ t ^ (t << 7);
    t = (x1 ^ (x1 >> 14)) & cc;
    x1 = x1 ^ t ^ (t << 14);
#endif
    // pre-transform y
    t = (y ^ (y >> 7)) & aa;
    y = y ^ t ^ (t << 7);
    t = (y ^ (y >> 14)) & cc;
    y = y ^ t ^ (t << 14);

#if NBIS2SERIALPINS >= 8
    t = (y1 ^ (y1 >> 7)) & aa;
    y1 = y1 ^ t ^ (t << 7);
    t = (y1 ^ (y1 >> 14)) & cc;
    y1 = y1 ^ t ^ (t << 14);
#endif
 


    // final transform
#if NBIS2SERIALPINS >= 4
t = (x & ff) | ((y >> 4) & ff2);

    y = ((x << 4) & ff) | (y & ff2);

    x = t;
 #else
    x = ((y >> 4) & ff2);
    y =  (y & ff2);
  
 
#endif

#if NBIS2SERIALPINS >= 8

#if NBIS2SERIALPINS >= 12
    t = (x1 & ff) | ((y1 >> 4) & ff2);
    y1 = ((x1 << 4) & ff) | (y1 & ff2);
    x1 = t;
   #else
      x1 = ((y1 >> 4) & ff2);
      y1 = (y1 & ff2);
 #endif
  #endif

  
#if NBIS2SERIALPINS >= 8

    *((uint16_t *)(B)) = (uint16_t)(((x & 0xff000000) >> 8 | ((x1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 48)) = (uint16_t)(((x & 0xff0000) >> 16 | ((x1 & 0xff0000) >> 8)));
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)(((x & 0xff00) | ((x1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)((x & 0xff) | ((x1 & 0xff) << 8));
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)(((y & 0xff000000) >> 8 | ((y1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)(((y & 0xff0000) | ((y1 & 0xff0000) << 8)) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)(((y & 0xff00) | ((y1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)((y & 0xff) | ((y1 & 0xff) << 8));
   #else

    *((uint16_t *)(B)) = (uint16_t)((x) >> 24);
    *((uint16_t *)(B + 48)) = (uint16_t)((x) >> 16);
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)((x)  >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)(x  );
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)((y) >> 24);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)((y) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)((y)  >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)(y ) ;
 
     #endif
     B+=2;
     A+=16;

      y = *(unsigned int *)(A);
#if NBIS2SERIALPINS >= 4
    x = *(unsigned int *)(A + 4);
#else
   x = 0;
#endif
#if NBIS2SERIALPINS >= 8
    y1 = *(unsigned int *)(A + 8);
    #if NBIS2SERIALPINS >= 12
          x1 = *(unsigned int *)(A + 12);
      #else
         x1 = 0;
      #endif

#endif


    // pre-transform x
#if NBIS2SERIALPINS >= 4
    t = (x ^ (x >> 7)) & aa;
    x = x ^ t ^ (t << 7);
    t = (x ^ (x >> 14)) & cc;
    x = x ^ t ^ (t << 14);
#endif
#if NBIS2SERIALPINS >= 12
    t = (x1 ^ (x1 >> 7)) & aa;
    x1 = x1 ^ t ^ (t << 7);
    t = (x1 ^ (x1 >> 14)) & cc;
    x1 = x1 ^ t ^ (t << 14);
#endif
    // pre-transform y
    t = (y ^ (y >> 7)) & aa;
    y = y ^ t ^ (t << 7);
    t = (y ^ (y >> 14)) & cc;
    y = y ^ t ^ (t << 14);

#if NBIS2SERIALPINS >= 8
    t = (y1 ^ (y1 >> 7)) & aa;
    y1 = y1 ^ t ^ (t << 7);
    t = (y1 ^ (y1 >> 14)) & cc;
    y1 = y1 ^ t ^ (t << 14);
#endif
 


    // final transform
#if NBIS2SERIALPINS >= 4
t = (x & ff) | ((y >> 4) & ff2);

    y = ((x << 4) & ff) | (y & ff2);

    x = t;
 #else
    x = ((y >> 4) & ff2);
    y =  (y & ff2);
  
 
#endif

#if NBIS2SERIALPINS >= 8

#if NBIS2SERIALPINS >= 12
    t = (x1 & ff) | ((y1 >> 4) & ff2);
    y1 = ((x1 << 4) & ff) | (y1 & ff2);
    x1 = t;
   #else
      x1 = ((y1 >> 4) & ff2);
      y1 = (y1 & ff2);
 #endif
  #endif

  
#if NBIS2SERIALPINS >= 8

    *((uint16_t *)(B)) = (uint16_t)(((x & 0xff000000) >> 8 | ((x1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 48)) = (uint16_t)(((x & 0xff0000) >> 16 | ((x1 & 0xff0000) >> 8)));
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)(((x & 0xff00) | ((x1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)((x & 0xff) | ((x1 & 0xff) << 8));
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)(((y & 0xff000000) >> 8 | ((y1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)(((y & 0xff0000) | ((y1 & 0xff0000) << 8)) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)(((y & 0xff00) | ((y1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)((y & 0xff) | ((y1 & 0xff) << 8));
   #else

    *((uint16_t *)(B)) = (uint16_t)((x) >> 24);
    *((uint16_t *)(B + 48)) = (uint16_t)((x) >> 16);
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)((x)  >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)(x  );
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)((y) >> 24);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)((y) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)((y)  >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)(y ) ;
 
     #endif
     B+=2;
     A+=16;

//******
y = *(unsigned int *)(A);
#if NBIS2SERIALPINS >= 4
    x = *(unsigned int *)(A + 4);
#else
   x = 0;
#endif
#if NBIS2SERIALPINS >= 8
    y1 = *(unsigned int *)(A + 8);
    #if NBIS2SERIALPINS >= 12
          x1 = *(unsigned int *)(A + 12);
      #else
         x1 = 0;
      #endif

#endif


    // pre-transform x
#if NBIS2SERIALPINS >= 4
    t = (x ^ (x >> 7)) & aa;
    x = x ^ t ^ (t << 7);
    t = (x ^ (x >> 14)) & cc;
    x = x ^ t ^ (t << 14);
#endif
#if NBIS2SERIALPINS >= 12
    t = (x1 ^ (x1 >> 7)) & aa;
    x1 = x1 ^ t ^ (t << 7);
    t = (x1 ^ (x1 >> 14)) & cc;
    x1 = x1 ^ t ^ (t << 14);
#endif
    // pre-transform y
    t = (y ^ (y >> 7)) & aa;
    y = y ^ t ^ (t << 7);
    t = (y ^ (y >> 14)) & cc;
    y = y ^ t ^ (t << 14);

#if NBIS2SERIALPINS >= 8
    t = (y1 ^ (y1 >> 7)) & aa;
    y1 = y1 ^ t ^ (t << 7);
    t = (y1 ^ (y1 >> 14)) & cc;
    y1 = y1 ^ t ^ (t << 14);
#endif
 


    // final transform
#if NBIS2SERIALPINS >= 4
t = (x & ff) | ((y >> 4) & ff2);

    y = ((x << 4) & ff) | (y & ff2);

    x = t;
 #else
    x = ((y >> 4) & ff2);
    y =  (y & ff2);
  
 
#endif

#if NBIS2SERIALPINS >= 8

#if NBIS2SERIALPINS >= 12
    t = (x1 & ff) | ((y1 >> 4) & ff2);
    y1 = ((x1 << 4) & ff) | (y1 & ff2);
    x1 = t;
   #else
      x1 = ((y1 >> 4) & ff2);
      y1 = (y1 & ff2);
 #endif
  #endif

  
#if NBIS2SERIALPINS >= 8

    *((uint16_t *)(B)) = (uint16_t)(((x & 0xff000000) >> 8 | ((x1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 48)) = (uint16_t)(((x & 0xff0000) >> 16 | ((x1 & 0xff0000) >> 8)));
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)(((x & 0xff00) | ((x1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)((x & 0xff) | ((x1 & 0xff) << 8));
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)(((y & 0xff000000) >> 8 | ((y1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)(((y & 0xff0000) | ((y1 & 0xff0000) << 8)) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)(((y & 0xff00) | ((y1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)((y & 0xff) | ((y1 & 0xff) << 8));
   #else

    *((uint16_t *)(B)) = (uint16_t)((x) >> 24);
    *((uint16_t *)(B + 48)) = (uint16_t)((x) >> 16);
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)((x)  >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)(x  );
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)((y) >> 24);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)((y) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)((y)  >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)(y ) ;
 
     #endif
     B+=2;
     A+=16;

      y = *(unsigned int *)(A);
#if NBIS2SERIALPINS >= 4
    x = *(unsigned int *)(A + 4);
#else
   x = 0;
#endif
#if NBIS2SERIALPINS >= 8
    y1 = *(unsigned int *)(A + 8);
    #if NBIS2SERIALPINS >= 12
          x1 = *(unsigned int *)(A + 12);
      #else
         x1 = 0;
      #endif

#endif


    // pre-transform x
#if NBIS2SERIALPINS >= 4
    t = (x ^ (x >> 7)) & aa;
    x = x ^ t ^ (t << 7);
    t = (x ^ (x >> 14)) & cc;
    x = x ^ t ^ (t << 14);
#endif
#if NBIS2SERIALPINS >= 12
    t = (x1 ^ (x1 >> 7)) & aa;
    x1 = x1 ^ t ^ (t << 7);
    t = (x1 ^ (x1 >> 14)) & cc;
    x1 = x1 ^ t ^ (t << 14);
#endif
    // pre-transform y
    t = (y ^ (y >> 7)) & aa;
    y = y ^ t ^ (t << 7);
    t = (y ^ (y >> 14)) & cc;
    y = y ^ t ^ (t << 14);

#if NBIS2SERIALPINS >= 8
    t = (y1 ^ (y1 >> 7)) & aa;
    y1 = y1 ^ t ^ (t << 7);
    t = (y1 ^ (y1 >> 14)) & cc;
    y1 = y1 ^ t ^ (t << 14);
#endif
 


    // final transform
#if NBIS2SERIALPINS >= 4
t = (x & ff) | ((y >> 4) & ff2);

    y = ((x << 4) & ff) | (y & ff2);

    x = t;
 #else
    x = ((y >> 4) & ff2);
    y =  (y & ff2);
  
 
#endif

#if NBIS2SERIALPINS >= 8

#if NBIS2SERIALPINS >= 12
    t = (x1 & ff) | ((y1 >> 4) & ff2);
    y1 = ((x1 << 4) & ff) | (y1 & ff2);
    x1 = t;
   #else
      x1 = ((y1 >> 4) & ff2);
      y1 = (y1 & ff2);
 #endif
  #endif

  
#if NBIS2SERIALPINS >= 8

    *((uint16_t *)(B)) = (uint16_t)(((x & 0xff000000) >> 8 | ((x1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 48)) = (uint16_t)(((x & 0xff0000) >> 16 | ((x1 & 0xff0000) >> 8)));
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)(((x & 0xff00) | ((x1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)((x & 0xff) | ((x1 & 0xff) << 8));
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)(((y & 0xff000000) >> 8 | ((y1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)(((y & 0xff0000) | ((y1 & 0xff0000) << 8)) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)(((y & 0xff00) | ((y1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)((y & 0xff) | ((y1 & 0xff) << 8));
   #else

    *((uint16_t *)(B)) = (uint16_t)((x) >> 24);
    *((uint16_t *)(B + 48)) = (uint16_t)((x) >> 16);
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)((x)  >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)(x  );
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)((y) >> 24);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)((y) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)((y)  >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)(y ) ;
 
     #endif
     B+=2;
     A+=16;


//*************


y = *(unsigned int *)(A);
#if NBIS2SERIALPINS >= 4
    x = *(unsigned int *)(A + 4);
#else
   x = 0;
#endif
#if NBIS2SERIALPINS >= 8
    y1 = *(unsigned int *)(A + 8);
    #if NBIS2SERIALPINS >= 12
          x1 = *(unsigned int *)(A + 12);
      #else
         x1 = 0;
      #endif

#endif


    // pre-transform x
#if NBIS2SERIALPINS >= 4
    t = (x ^ (x >> 7)) & aa;
    x = x ^ t ^ (t << 7);
    t = (x ^ (x >> 14)) & cc;
    x = x ^ t ^ (t << 14);
#endif
#if NBIS2SERIALPINS >= 12
    t = (x1 ^ (x1 >> 7)) & aa;
    x1 = x1 ^ t ^ (t << 7);
    t = (x1 ^ (x1 >> 14)) & cc;
    x1 = x1 ^ t ^ (t << 14);
#endif
    // pre-transform y
    t = (y ^ (y >> 7)) & aa;
    y = y ^ t ^ (t << 7);
    t = (y ^ (y >> 14)) & cc;
    y = y ^ t ^ (t << 14);

#if NBIS2SERIALPINS >= 8
    t = (y1 ^ (y1 >> 7)) & aa;
    y1 = y1 ^ t ^ (t << 7);
    t = (y1 ^ (y1 >> 14)) & cc;
    y1 = y1 ^ t ^ (t << 14);
#endif
 


    // final transform
#if NBIS2SERIALPINS >= 4
t = (x & ff) | ((y >> 4) & ff2);

    y = ((x << 4) & ff) | (y & ff2);

    x = t;
 #else
    x = ((y >> 4) & ff2);
    y =  (y & ff2);
  
 
#endif

#if NBIS2SERIALPINS >= 8

#if NBIS2SERIALPINS >= 12
    t = (x1 & ff) | ((y1 >> 4) & ff2);
    y1 = ((x1 << 4) & ff) | (y1 & ff2);
    x1 = t;
   #else
      x1 = ((y1 >> 4) & ff2);
      y1 = (y1 & ff2);
 #endif
  #endif

  
#if NBIS2SERIALPINS >= 8

    *((uint16_t *)(B)) = (uint16_t)(((x & 0xff000000) >> 8 | ((x1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 48)) = (uint16_t)(((x & 0xff0000) >> 16 | ((x1 & 0xff0000) >> 8)));
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)(((x & 0xff00) | ((x1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)((x & 0xff) | ((x1 & 0xff) << 8));
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)(((y & 0xff000000) >> 8 | ((y1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)(((y & 0xff0000) | ((y1 & 0xff0000) << 8)) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)(((y & 0xff00) | ((y1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)((y & 0xff) | ((y1 & 0xff) << 8));
   #else

    *((uint16_t *)(B)) = (uint16_t)((x) >> 24);
    *((uint16_t *)(B + 48)) = (uint16_t)((x) >> 16);
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)((x)  >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)(x  );
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)((y) >> 24);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)((y) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)((y)  >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)(y ) ;
 
     #endif
     B+=2;
     A+=16;

      y = *(unsigned int *)(A);
#if NBIS2SERIALPINS >= 4
    x = *(unsigned int *)(A + 4);
#else
   x = 0;
#endif
#if NBIS2SERIALPINS >= 8
    y1 = *(unsigned int *)(A + 8);
    #if NBIS2SERIALPINS >= 12
          x1 = *(unsigned int *)(A + 12);
      #else
         x1 = 0;
      #endif

#endif


    // pre-transform x
#if NBIS2SERIALPINS >= 4
    t = (x ^ (x >> 7)) & aa;
    x = x ^ t ^ (t << 7);
    t = (x ^ (x >> 14)) & cc;
    x = x ^ t ^ (t << 14);
#endif
#if NBIS2SERIALPINS >= 12
    t = (x1 ^ (x1 >> 7)) & aa;
    x1 = x1 ^ t ^ (t << 7);
    t = (x1 ^ (x1 >> 14)) & cc;
    x1 = x1 ^ t ^ (t << 14);
#endif
    // pre-transform y
    t = (y ^ (y >> 7)) & aa;
    y = y ^ t ^ (t << 7);
    t = (y ^ (y >> 14)) & cc;
    y = y ^ t ^ (t << 14);

#if NBIS2SERIALPINS >= 8
    t = (y1 ^ (y1 >> 7)) & aa;
    y1 = y1 ^ t ^ (t << 7);
    t = (y1 ^ (y1 >> 14)) & cc;
    y1 = y1 ^ t ^ (t << 14);
#endif
 


    // final transform
#if NBIS2SERIALPINS >= 4
t = (x & ff) | ((y >> 4) & ff2);

    y = ((x << 4) & ff) | (y & ff2);

    x = t;
 #else
    x = ((y >> 4) & ff2);
    y =  (y & ff2);
  
 
#endif

#if NBIS2SERIALPINS >= 8

#if NBIS2SERIALPINS >= 12
    t = (x1 & ff) | ((y1 >> 4) & ff2);
    y1 = ((x1 << 4) & ff) | (y1 & ff2);
    x1 = t;
   #else
      x1 = ((y1 >> 4) & ff2);
      y1 = (y1 & ff2);
 #endif
  #endif

  
#if NBIS2SERIALPINS >= 8

    *((uint16_t *)(B)) = (uint16_t)(((x & 0xff000000) >> 8 | ((x1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 48)) = (uint16_t)(((x & 0xff0000) >> 16 | ((x1 & 0xff0000) >> 8)));
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)(((x & 0xff00) | ((x1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)((x & 0xff) | ((x1 & 0xff) << 8));
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)(((y & 0xff000000) >> 8 | ((y1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)(((y & 0xff0000) | ((y1 & 0xff0000) << 8)) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)(((y & 0xff00) | ((y1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)((y & 0xff) | ((y1 & 0xff) << 8));
   #else

    *((uint16_t *)(B)) = (uint16_t)((x) >> 24);
    *((uint16_t *)(B + 48)) = (uint16_t)((x) >> 16);
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)((x)  >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)(x  );
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)((y) >> 24);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)((y) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)((y)  >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)(y ) ;
 
     #endif
     B+=2;
     A+=16;

//******
y = *(unsigned int *)(A);
#if NBIS2SERIALPINS >= 4
    x = *(unsigned int *)(A + 4);
#else
   x = 0;
#endif
#if NBIS2SERIALPINS >= 8
    y1 = *(unsigned int *)(A + 8);
    #if NBIS2SERIALPINS >= 12
          x1 = *(unsigned int *)(A + 12);
      #else
         x1 = 0;
      #endif

#endif


    // pre-transform x
#if NBIS2SERIALPINS >= 4
    t = (x ^ (x >> 7)) & aa;
    x = x ^ t ^ (t << 7);
    t = (x ^ (x >> 14)) & cc;
    x = x ^ t ^ (t << 14);
#endif
#if NBIS2SERIALPINS >= 12
    t = (x1 ^ (x1 >> 7)) & aa;
    x1 = x1 ^ t ^ (t << 7);
    t = (x1 ^ (x1 >> 14)) & cc;
    x1 = x1 ^ t ^ (t << 14);
#endif
    // pre-transform y
    t = (y ^ (y >> 7)) & aa;
    y = y ^ t ^ (t << 7);
    t = (y ^ (y >> 14)) & cc;
    y = y ^ t ^ (t << 14);

#if NBIS2SERIALPINS >= 8
    t = (y1 ^ (y1 >> 7)) & aa;
    y1 = y1 ^ t ^ (t << 7);
    t = (y1 ^ (y1 >> 14)) & cc;
    y1 = y1 ^ t ^ (t << 14);
#endif
 


    // final transform
#if NBIS2SERIALPINS >= 4
t = (x & ff) | ((y >> 4) & ff2);

    y = ((x << 4) & ff) | (y & ff2);

    x = t;
 #else
    x = ((y >> 4) & ff2);
    y =  (y & ff2);
  
 
#endif

#if NBIS2SERIALPINS >= 8

#if NBIS2SERIALPINS >= 12
    t = (x1 & ff) | ((y1 >> 4) & ff2);
    y1 = ((x1 << 4) & ff) | (y1 & ff2);
    x1 = t;
   #else
      x1 = ((y1 >> 4) & ff2);
      y1 = (y1 & ff2);
 #endif
  #endif

  
#if NBIS2SERIALPINS >= 8

    *((uint16_t *)(B)) = (uint16_t)(((x & 0xff000000) >> 8 | ((x1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 48)) = (uint16_t)(((x & 0xff0000) >> 16 | ((x1 & 0xff0000) >> 8)));
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)(((x & 0xff00) | ((x1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)((x & 0xff) | ((x1 & 0xff) << 8));
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)(((y & 0xff000000) >> 8 | ((y1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)(((y & 0xff0000) | ((y1 & 0xff0000) << 8)) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)(((y & 0xff00) | ((y1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)((y & 0xff) | ((y1 & 0xff) << 8));
   #else

    *((uint16_t *)(B)) = (uint16_t)((x) >> 24);
    *((uint16_t *)(B + 48)) = (uint16_t)((x) >> 16);
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)((x)  >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)(x  );
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)((y) >> 24);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)((y) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)((y)  >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)(y ) ;
 
     #endif
     B+=2;
     A+=16;

      y = *(unsigned int *)(A);
#if NBIS2SERIALPINS >= 4
    x = *(unsigned int *)(A + 4);
#else
   x = 0;
#endif
#if NBIS2SERIALPINS >= 8
    y1 = *(unsigned int *)(A + 8);
    #if NBIS2SERIALPINS >= 12
          x1 = *(unsigned int *)(A + 12);
      #else
         x1 = 0;
      #endif

#endif


    // pre-transform x
#if NBIS2SERIALPINS >= 4
    t = (x ^ (x >> 7)) & aa;
    x = x ^ t ^ (t << 7);
    t = (x ^ (x >> 14)) & cc;
    x = x ^ t ^ (t << 14);
#endif
#if NBIS2SERIALPINS >= 12
    t = (x1 ^ (x1 >> 7)) & aa;
    x1 = x1 ^ t ^ (t << 7);
    t = (x1 ^ (x1 >> 14)) & cc;
    x1 = x1 ^ t ^ (t << 14);
#endif
    // pre-transform y
    t = (y ^ (y >> 7)) & aa;
    y = y ^ t ^ (t << 7);
    t = (y ^ (y >> 14)) & cc;
    y = y ^ t ^ (t << 14);

#if NBIS2SERIALPINS >= 8
    t = (y1 ^ (y1 >> 7)) & aa;
    y1 = y1 ^ t ^ (t << 7);
    t = (y1 ^ (y1 >> 14)) & cc;
    y1 = y1 ^ t ^ (t << 14);
#endif
 


    // final transform
#if NBIS2SERIALPINS >= 4
t = (x & ff) | ((y >> 4) & ff2);

    y = ((x << 4) & ff) | (y & ff2);

    x = t;
 #else
    x = ((y >> 4) & ff2);
    y =  (y & ff2);
  
 
#endif

#if NBIS2SERIALPINS >= 8

#if NBIS2SERIALPINS >= 12
    t = (x1 & ff) | ((y1 >> 4) & ff2);
    y1 = ((x1 << 4) & ff) | (y1 & ff2);
    x1 = t;
   #else
      x1 = ((y1 >> 4) & ff2);
      y1 = (y1 & ff2);
 #endif
  #endif

  
#if NBIS2SERIALPINS >= 8

    *((uint16_t *)(B)) = (uint16_t)(((x & 0xff000000) >> 8 | ((x1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 48)) = (uint16_t)(((x & 0xff0000) >> 16 | ((x1 & 0xff0000) >> 8)));
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)(((x & 0xff00) | ((x1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)((x & 0xff) | ((x1 & 0xff) << 8));
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)(((y & 0xff000000) >> 8 | ((y1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)(((y & 0xff0000) | ((y1 & 0xff0000) << 8)) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)(((y & 0xff00) | ((y1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)((y & 0xff) | ((y1 & 0xff) << 8));
   #else

    *((uint16_t *)(B)) = (uint16_t)((x) >> 24);
    *((uint16_t *)(B + 48)) = (uint16_t)((x) >> 16);
    *((uint16_t *)(B + 2 * 48)) = (uint16_t)((x)  >> 8);
    *((uint16_t *)(B + 3 * 48)) = (uint16_t)(x  );
    *((uint16_t *)(B + 4 * 48)) = (uint16_t)((y) >> 24);
    *((uint16_t *)(B + 5 * 48)) = (uint16_t)((y) >> 16);
    *((uint16_t *)(B + 6 * 48)) = (uint16_t)((y)  >> 8);
    *((uint16_t *)(B + 7 * 48)) = (uint16_t)(y ) ;
 
     #endif
   //  B+=370;
    // A+=16;
 // }
}


//static  Lines firstPixel[nb_components];
//static void IRAM_ATTR loadAndTranspose(uint8_t *ledt, OffsetDisplay offdisp, uint16_t *buff, int ledtodisp, uint8_t *mapg, uint8_t *mapr, uint8_t *mapb, uint8_t *mapw, uint8_t *r_map, uint8_t *g_map, uint8_t *b_map)
static inline __attribute__((always_inline))  void IRAM_ATTR loadAndTranspose(I2SClocklessVirtualLedDriver *driver)
{

#if CORE_DEBUG_LEVEL>=4
driver->_times[driver->ledToDisplay]=ESP.getCycleCount();
#endif
uint8_t *ledt=driver->leds;
uint16_t *buff=(uint16_t *)driver->DMABuffersTampon[driver->dmaBufferActive]->buffer;
int ledtodisp=driver->ledToDisplay;
uint8_t *mapg=driver->__green_map;
uint8_t *mapr=driver->__red_map;
//uint8_t *mapb=driver->__blue_map;
#ifdef _USE_PALETTE
uint8_t *palette=driver->palette;
#endif
#if nb_components >3
uint8_t *mapw=driver->__white_map;
#endif
 #if STATICCOLOR == 0
uint8_t *r_map= driver->r_map;
uint8_t *g_map = driver->g_map;
uint8_t *b_map=driver->b_map;
#endif
Lines *firstPixel=driver->firstPixel;
 //uint16_t *hmaplocal=driver->_hmapoff;
/*
Lines firstPixel[nb_components];
//memset( firstPixel,0,16*8*3);
memset( firstPixel[0].bytes,0,16*8);
memset( firstPixel[1].bytes,0,16*8);
memset( firstPixel[2].bytes,0,16*8);
    firstPixel[0].bytes[16+NBIS2SERIALPINS] = 255;
    firstPixel[1].bytes[16+NBIS2SERIALPINS] = 255;
    firstPixel[2].bytes[16+NBIS2SERIALPINS] = 255;
#if nb_components > 3
    firstPixel[3].bytes[16+NBIS2SERIALPINS] = 255;
    
#endif
*/


 #ifdef _LEDMAPPING
  #ifdef __SOFTWARE_MAP
    uint16_t _led_tmp=ledtodisp;
    uint16_t led_tmp;
    #endif
 #endif
uint8_t * poli_b,* poli,*_poli;

        //#endif
   #ifndef _LEDMAPPING
        _poli = driver->leds+ ledtodisp * _palette_size;
       // driver->leddt+=_palette_size;
    #endif
    buff += OFFSET;
    //jump en deux



    //.for()

    for(int pin74HC595=0;pin74HC595<8;pin74HC595++)
    {
       
         #ifdef _LEDMAPPING
          #ifdef __SOFTWARE_MAP
                led_tmp=_led_tmp;
         #endif

         #else
             poli=_poli;
         #endif
         #ifdef __HARDWARE_MAP
         int pin=(pin74HC595 )<<4;
         #else
        int pin=(pin74HC595^1 )<<4;
        #endif
    for (int vpin = 0; vpin < NBIS2SERIALPINS; vpin++)
    {
         #ifdef _LEDMAPPING
            #ifdef __SOFTWARE_MAP
                poli = ledt + driver->mapLed(led_tmp) * _palette_size;
            #endif
            #ifdef __HARDWARE_MAP
                #ifdef HARDWARE_SCROLL
                   poli =ledt + driver->remap( *(driver->_hmapoff),offdisp)*_palette_size;
                #else
                #ifdef _HARDWARE_SCROLL_MAP
                 poli = ledt + *(driver->_hmapoff)*_palette_size;
                 #else
                poli = ledt + *(driver->_hmapoff);
                 #endif
               
                 #endif
            #endif
            #ifdef __HARDWARE_MAP_PROGMEM
                 poli = driver->leds + pgm_read_word_near(driver->_hmap + driver->_hmapoff);
            #endif
        #endif


#ifdef _USE_PALETTE
    poli_b=palette+*(poli)*nb_components;
 #else
    poli_b=poli;
#endif

        #if STATICCOLOR == 1
        
        firstPixel[p_g].bytes[pin+vpin] = mapg[*(poli_b + 1)];
        firstPixel[p_r].bytes[pin+vpin] = mapr[*(poli_b)];
        firstPixel[p_b].bytes[pin+vpin] = mapg[*(poli_b + 2)];

        #else
        firstPixel[g_map[8 * vpin]].bytes[pin+vpin] = mapg[*(poli_b + 1)];
        firstPixel[r_map[8 * vpin]].bytes[pin+vpin] = mapr[*(poli_b )];
        firstPixel[b_map[8 * vpin]].bytes[pin+vpin] = mapb[*(poli_b + 2)];
        #endif
#if nb_components > 3
        firstPixel[3].bytes[pin+vpin] = mapw[*(poli_b + 3)];
#endif
       // pin++;
 #ifdef _LEDMAPPING
            #ifdef __SOFTWARE_MAP
                led_tmp+=I2S_OFF_MAP;
            #endif
                    #ifdef __HARDWARE_MAP
    driver->_hmapoff++;
#endif
        #else
         poli += I2S_OFF;
        #endif

        
    }
    #ifdef _LEDMAPPING
            #ifdef __SOFTWARE_MAP
        _led_tmp+=NUM_LEDS_PER_STRIP;
    #endif
    #else
     _poli+=NUM_LEDS_PER_STRIP*_palette_size;
    #endif
        

   
}

    transpose16x1_noinline2(firstPixel[0].bytes, (uint8_t *)(buff));

 transpose16x1_noinline2(firstPixel[1].bytes, (uint8_t *)(buff + 192));
    transpose16x1_noinline2(firstPixel[2].bytes, (uint8_t *)(buff + 384));
#if nb_components > 3
    transpose16x1_noinline2(firstPixel[3].bytes, (uint8_t *)(buff + 576));
#endif


    



   
#if CORE_DEBUG_LEVEL>=4
driver->_times[driver->ledToDisplay]=ESP.getCycleCount()-driver->_times[driver->ledToDisplay];
#endif
}


static void showPixelsTask(void *pvParameters)
{
    I2SClocklessVirtualLedDriver *cont = (I2SClocklessVirtualLedDriver *)pvParameters;
      if(cont->_gI2SClocklessDriver_intr_handle!=NULL)
  {
            esp_intr_free(cont->_gI2SClocklessDriver_intr_handle);
       
  }
   ESP_LOGV(TAG,"setting interupt handler");
        esp_err_t e = esp_intr_alloc(interruptSource, ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM, &_I2SClocklessVirtualLedDriverinterruptHandler, cont, &cont->_gI2SClocklessDriver_intr_handle);
        if (e!=ESP_OK)
        {
            ESP_LOGE(TAG,"Impossible to create interupt allocation");
            return;
        }
       ESP_LOGV(TAG,"interupt handler set on core %d",xPortGetCoreID() );
  
    for(;;) {
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    cont-> ___showPixels();
      // xTaskNotifyGive(I2SClocklessVirtualLedDriver_returnTaskHandle);
    }
}