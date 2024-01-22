[![build status](https://github.com/hpwit/I2SClocklessVirtualLedDriver/actions/workflows/testcode.yml/badge.svg)](https://github.com/hpwit/I2SClocklessVirtualLedDriver/actions/workflows/testcode.yml) &nbsp;&nbsp;[![Badge Version](https://img.shields.io/github/v/release/hpwit/I2SClocklessVirtualLedDriver?label=latest%20release)](https://github.com/hpwit/I2SClocklessVirtualLedDriver/releases/latest) &nbsp;[![Badge Version](https://img.shields.io/badge/release_note-blue)](Releasenote.md)

# I2SClocklessVirtualLedDriver for esp32
## Introduction
Hello led afficionados !! Here is the new version of the Virtual pins library. In reality this version of the library had been sitting more or less finalized on my laptop for a while. I need to take the time and energy to write the correct examples and of course update the documentation. 
I have been writing led drivers for the past couple of years now while I was building [my 123x48 panel](https://hackaday.io/project/158268-5904-leds-panel). 
It inspired me to create the I2S driver implemented in FastLED and then the Virtual pins library.
I am also planning to merge all my different led libraries (4 of them in total)


### What kind of led do you want to drive
This library is a new take on driving ws2812 leds with I2S on an esp32. It allows to drive up to 120 strips !!! leds in parallel of  
* RGB:
    * WS2812,
    * WS2813,
    * WS2815 
* RGBW 
    * SK6812. 

If you are using RGB led type then this library is fully compatible with FastLED library objects

### Why have I rewritten the library ?
I have rewritten the library out of the FastLED framework to allow easier testing but also create a pixel pusher independant of the library you want to use. But the main reason is the way I wanted to drive the leds. I haev tried to put more functionalities in the driver than the usual 'leds pusher'. Indeed this driver integrates:
* led mapping
* color palette
* ease the use of the second core
* framebuffer
* non blocking capabilities
* 'live led' calculation
* scrolling, rotating, scaling
* duplication
* emulate 'line interrupts' (retro programers will understand)
* Options to avoid artifacts if you're have interrupt intensive code


I am trying to be kinda lenghtly on this readme. I hope to explain the why of some functions and for the user to use the one most suitable for its use case.
Some of the 'maybe most' of the functionailities will never been used be it was fun and intersting to implement.

## Let's start
### How can 120 strips can de driven in parallel with an esp32 !!!
The esp32 has 24 pins that can act as outputs pins. This is far away from 120 pins ...
The main idea is to drive 8 strips out of one single pin of the ESP32. and you can do this for 15 esp32 pins which gives you 8x15=120 strips !!!

To do this feat you will need to get some ICs
* 74HC245 : this is a bus used as a level shifter (you will need only one of them for LATCH and CLOCK)
* 74HC595 : this is an 8 bit shift register (you will need one 74HC595 for each Virtual pin)

You'll find the schematics within the 'extra' folder of this library.

I personally use it to drive my panels 5904=123x48 (16 strips of 369 leds) and a 12288=128x96 (48 16x16 panels)  but still have space for an I2C Nintenda controller and SD card reader.


### Pixel object
In FastLED ``CRGB`` and ``CHSV`` objects exist, they represent data in RGB or CHSV color space. These are made of 3 bytes (24 bits)

In this library there is a similar object ``Pixel`` which is equivalent to ``CRGB``. But it can be changed at compile time.
```C
#define COLOR_RGBW

#include "I2SClocklessVirtualLedDriver.h"
#define NUM_LEDS 200

Pixel leds[NUM_LEDS];
//here the Pixel is not RGB but RGBW hence this time it will be a 36 bits
//it's still ocmpatible with the CRGB object 
leds[0]=CRGB(233,23,44); 
```
in that case the following mapping will be done 
W = min(R, G, B);
R = R - W; 
G = G - W;
B = B - W;

In the examples you will find some that integrate the FastLED objects.

### MACROS that have nothing to do with led driving

* ``HOW_LONG(name, func)`` : This macro will the time taken by a part of the code
ex:
```C
HOW_LONG("Time taken", {
//insert the code i.e
driver.showPixels();

});
```

the result  in the serial output:
``The function *** Time taken *** took 100 ms or 10 fps``

*  ``RUN_SKETCH_FOR(name, duration, func)``:  Execute part of the code for the duration in millisecondes
ex:
```C
//it will run for 1000ms or 1s
int offset=0;
RUN_SKETCH_FOR("scroll", 1000 {
    leds[offset]=CRGB::Red;
    driver.showPixels();
    offset++;
});

```

*  ``RUN_SKETCH_N_TIMES(name, ntimes, func)``:  Execute part of the code N times
ex:
```C
//it will run 100 times
int offset=0;
RUN_SKETCH_FOR("scroll", 100 {
    leds[offset]=CRGB::Red;
    driver.showPixels();
    offset++;
});

```

### Array of strips
In most leds driver librairies you declare each strip attached to one pin, one line at a time.

example 1: For 4 strips in FastLED
```C
CRGB leds1[number_of_leds1];
CRGB leds2[number_of_leds2];
CRGB leds3[number_of_leds3];
CRGB leds4[number_of_leds4];

FasLED.addLeds<PIN1,ORDER>(leds1,number_of_leds1);
FasLED.addLeds<PIN2,ORDER>(leds2,number_of_leds2);
FasLED.addLeds<PIN3,ORDER>(leds3,number_of_leds3);
FasLED.addLeds<PIN4,ORDER>(leds4,number_of_leds4);
```

example 2: If you are using a large array of same length strips, you would do this:
```C
CRGB leds[4*NUM_LED_PER_STRIPS];
FasLED.addLeds<PIN1,ORDER>(leds,0,NUM_LED_PER_STRIPS);
FasLED.addLeds<PIN2,ORDER>(leds,NUM_LED_PER_STRIPS,NUM_LED_PER_STRIPS);
FasLED.addLeds<PIN3,ORDER>(leds,2*NUM_LED_PER_STRIPS,NUM_LED_PER_STRIPS);
FasLED.addLeds<PIN4,ORDER>(leds,3*NUM_LED_PER_STRIPS,NUM_LED_PER_STRIPS);
```

example 3: For information if you want to get (for development purpose ease) the leds1,leds2,...
```C
CRGB *leds1=leds;
CRGB *leds2=leds+NUM_LED_PER_STRIPS;
CRGB *leds3=leds+2*NUM_LED_PER_STRIPS;
CRGB *leds4=leds+3*NUM_LED_PER_STRIPS;
```

If all the strips of the first example are of the same size, then the 2 examples are the doing exactly the same. Hence when using strips of different lengths we cannot put them in a  big array ? **FALSE**. You cant create a large array when using `NUM_LED_PER_STRIP` being the largest of `number_of_leds`. Of course you array woul be larger than you actual numbre of leds but we can do with the lost of space.

### OK, but what is the link between an array of strips and this driver ?
Here is how we would declare the 4 strips in of our example:
```C
CRGB leds[4*NUM_LED_PER_STRIPS];
int pins[4]={PIN1,PIN2,PIN3,PIN4};
driver.initled((uint8_t*)leds,pins,4,NUM_LED_PER_STRIPS,ORDER_GRB);
```
We are declaring that my `leds` array represent 4 strips of `NUM_LED_PER_STRIPS` leds ,each strip being linked to the pins defined in the pins array `pins`. This is way easier to declare a lot of strips. As discussed before if your strips are not of the same lentgh just define `NUM_LED_PER_STRIPS` with the largest `number_of_leds`.


### First let's declare a new driver

```C

#define NBIS2SERIALPINS 2  //the number of virtual pins

//set the strip color order
//by default RGB
/* the other values
#define COLOR_RGB
#define COLOR_RBG
#define COLOR_GBR
#define COLOR_BGR
#define COLOR_BRG
#define COLOR_RGBW
*/

#define NUM_LEDS_PER_STRIP 256

#define NUM_LEDS (NUM_LEDS_PER_STRIP*NBIS2SERIALPINS*8)

#define CLOCK_PIN 16

#define LATCH_PIN 4

#include "I2SClocklessVirtualLedDriver.h"

I2SClocklessVirtualLedDriver driver;
```

### How to define my leds array ?
#### You are using RGB type leds
RGB type leds store the information of the led over 3 bytes. `Red,Green,Blue`.  Hence the size in bytes of a led array of `NUM_LEDS` is `3xNUM_LEDS`
```C
Pixels leds[NUM_LEDS];

//if you are using FastLED library this definition will be equivalent to the previous as the CRGB object is 3 bytes
CRGB leds[NUM_LEDS];

//you can use either of those
```
#### You are using RGBW type leds
This time to store the information of the led you will need 4 bytes `Red,Green,Blue,White` Hence the size in bytes of a led array of `NUM_LEDS` is `4xNUM_LEDS`
```C
Pixels leds[UM_LEDS];
```

### Driver functions
#### `void initled(uint8_t *leds, int *Pinsq, int CLOCK_PIN, int LATCH_PIN)`:

 This function initialize the strips.
* `*leds`: a pointer to the leds array
* `*Pins`: a pointer to the pins array
* `CLOCK_PIN`: pin for the clock output needs to be >=16
* `LATCH_PIN`: pin for the latch ouput

 
 example 4: declaring 16 strips of 256 leds in GRB 
 ```C
 
#define NBIS2SERIALPINS 2  //the number of virtual pins
#define COLOR_GRB 1 //set the strip color
#define NUM_LEDS_PER_STRIP 256
#define NUM_LEDS (NUM_LEDS_PER_STRIP*NBIS2SERIALPINS*8)
#define CLOCK_PIN 16
#define LATCH_PIN 4
#define NUM_STRIPS (NBIS2SERIALPINS*8)
 
 #include "I2SClocklessVirtualLedDriver.h"

 I2SClocklessVirtualLedDriver driver;
 
 Pixels leds[NUM_STRIPS*NUM_LEDS_PER_STRIP]; //equivalent of CRGB leds[NUM_LEDS_PER_STRIPS*NUM_STRIPS]
 int pins[NBIS2SERIALPINS] ={0,2};
 driver.initled((uint8_t*)leds,pins,CLOCK_PIN,LATCH_PIN);
 ```
 
 example 5: declaring 30 strips of 256 leds in RGBW
 ```C
 #define NBIS2SERIALPINS 4  //the number of virtual pins
#define COLOR_GRBW 1 //set the strip color to GRBW
#define NUM_LEDS_PER_STRIP 256
#define NUM_LEDS (NUM_LEDS_PER_STRIP*NBIS2SERIALPINS*8)
#define CLOCK_PIN 16
#define LATCH_PIN 4
#define NUM_STRIPS 30
 
 #include "I2SClocklessVirtualLedDriver.h"

 I2SClocklessVirtualLedDriver driver;
 
Pixel leds[NUM_STRIPS*NUM_LEDS_PER_STRIP]; 
 int pins[NBIS2SERIALPINS] ={0,2,12,13};
 driver.initled((uint8_t*)leds,pins,CLOCK_PIN,LATCH_PIN);
 ```

#### Other initialization functions:
* `void initled(Pixel leds, int *Pinsq, int CLOCK_PIN, int LATCH_PIN)`
*  `void initled(CRGB leds, int *Pinsq, int CLOCK_PIN, int LATCH_PIN)`
*  `void initled(frameBuffer *leds, int *Pinsq, int CLOCK_PIN, int LATCH_PIN)` this will be explained later on how to use a frameBuffer
*  `void initled(int *Pinsq, int CLOCK_PIN, int LATCH_PIN)`: you can initialized the driver without a led buffer.
this is also used in the case of direct pixel calculation
 ```C
 Pixel led1[NUM_LEDS];
 Pixel leds2[NUM_LEDS];
driver.initled(pins,CLOCK_PIN,LATCH_PIN);

...
driver.showPixels(led1);
...
driver.showPixels(led2);
 ```


#### `setBrightness(int brightness)`:
 
 This function sets the default brightness for 0->255



 
#### `setPixel(uint32_t pos, uint8_t red, uint8_t green, uint8_t blue)`:
 Set the color of a pixel 
 NB1: if you are using a RGBW led, this function will do and RGB->RGBW transformation with the following algotithm thanks to  @Jonathanese 
```C
W = min(R, G, B);
R = R - W; 
G = G - W;
B = B - W;
```
 NB: if you are familiar with FastLED it would be `leds[pos]=CRGB(red,green,blue)` as you will see in the examples
 
 
 
 #### `setPixel(uint32_t pos, uint8_t red, uint8_t green, uint8_t blue,uint8_t white)`:
 Set the color of a pixel for RGBW strips
 
 #### `showPixels()`:
 
 This function displays the pixels.

 #### `showPixels(uint8_t *leds)`:
This function allow set 'on the go' the pointer to the leds. This will help if you are using two buffers for your animation. 
It can also be used to ease dithering see example `Dithering` (I need to work on a hardware implementation btw)
```C
#define NBIS2SERIALPINS 4  //the number of virtual pins
#define COLOR_GRB 1 //set the strip color
#define NUM_LEDS_PER_STRIP 256
#define NUM_LEDS (NUM_LEDS_PER_STRIP*NBIS2SERIALPINS*8)
#define CLOCK_PIN 16
#define LATCH_PIN 4
#define NUM_STRIPS 30
#include "I2SClocklessVirtualLedDriver.h"

 I2SClocklessVirtualLedDriver driver;
 
 Pixel leds[NUM_STRIPS*NUM_LEDS_PER_STRIP];  
 Pixel leds2[NUM_STRIPS*NUM_LEDS_PER_STRIP]; 
 int pins[NBIS2SERIALPINS] ={0,2,12,13};
 driver.initled((uint8_t*)leds,pins,CLOCK_PIN,LATCH_PIN);

//displyaing the leds in leds1
driver.showPixels();

//displaying the leds in leds2
driver.showPixels(leds2);
```

## Let's talk about framerate
It's kind of a goal to display as fast as it's possible to have a good framerate. The  calculation of the framerate is not only base on the time to display the leds but also the time 'create' the led array.
i.e
```C
//display total  tiome/fps fpstotal
HOW_LONG("Total framerate", {
    //display time/fps to calculate the array fpscalc
    HOW_LONG("calcualtion led", {
    for(int i=0;i<NUM_LEDS;i++)
    {
        //do something
    }
    });
    //will display time/fps display the array fpsdisp
    HOW_LONG("display", {
        driver.showPixels();
    });
});

// 1/fpstotal=1/fpscalc+1/fpsdisp
```

## How do we improve the framerate

### On the same core 
Even if the showPixels() takes time, actually the CPU is not busy during the time. Indeed even if to send a pixel (RGB) it takes 30us (microseconds) but the calculation of the buffer takes for 5us to 23us depending on the number of virtual pins to calculate.
As a consequence we are loosing CPU time. Hopefully the esp32 rune several tasks at the sametime thanks to RTOS.

the driver allows to take advantage of this quite easily. `void showPixels(displayMode dispmode)` 

```C

......
 driver.showPixels(NO_WAIT);
....

```

In the example `waitnowaitmode.ino` we display 256 leds (max framerate 130fps). If you run the code (even without any leds attached) it will display `75fps` for the showPixels() and `112fps` for the `showPixels(NO_WAIT)`.
Why is it not 130 fps the max ? because the calculation of the leds array is done wiht the same ressource as the one to calculate the leds buffers to be sent. Both 'compiting' for the same CPU ressource. But we get a 50% speed increase.

### Enable the use of the second core
The esp32 has two cores and the driver simplify the usage of the second core.
The main core is core 1 on which runs the application bu default. The second core (core 0) is less used. We can tell the driver to run driver.showPixels() on the second core by `enableShowPixelsOnCore(int core)`

```C

......
//move the display on core 0
 driver.enableShowPixelsOnCore(0);
 //from now on all the showPixels() will be done on core 0
....


//move the display back on core 1
 driver.enableShowPixelsOnCore(1);
```
In the example `secondcore.ino`  without the second core we are still at `75fps` and `129fps` (close to the max) when using the two cores.
In the latest case both the calculation and the display occurs at the sametime.

:arrow_forward: NB: by construct using `driver.enableShowPixelsOnCore(1)` all the following `showPixels()` will be like `showPixels(NO_WAIT)`  




### 'HARDWARE SCROLLING'
Old term for a nice trick. The idea is to do a remapping of the leds within the driver directly so that the leds are displayed in another order. Pixels are pushed one at a time, and the normal way to do it is by going led 0,1,2,3 ....,N
Let's say that I want to 'scroll' by 5 pixels all the leds. Normally you would move leds 4->N-1 into 0,N-5 and then copy led 0=>led N-4 act. and then do the fastled.show().
The way I do it is to push within the driver led 4,5,6,7, ...., N-1,0,1,2,3 by calculating each time which pixels needs to be displayed using a simple algorithm about something along this `lednumber=> (lednumber+scroll)%N` (then a bit more complicated to take into account snake arrangement or not ,...)

#### `OffsetDisplay` object:
```C
struct OffsetDisplay
{
    int offsetx;
    int offsety;
    int panel_height;
    int panel_width;
    int image_height;
    int image_width;
    float scalling;
    int xc;
    int yc;
    float rotation;
    bool enableLoopx;
    bool enableLoopy;
    ....
};
```
At the initiation of the leds a default Offdisplay is created with certain values. You can get this default object with `getDefaultOffset();`.

I

#### Defining a panel
To be able to 'hardware scroll' in all directions you need to define how you panel is setup.
for instance if you have a panel 100 leds wide 20 leds height `panel_height=20` and `panel_witdh=100`.
If you are using mutilple strips you have two parameters 
NB: these parameters need to be put before `#include "I2SClocklessLedDriver.h"` :


#### `showPixels(OffDisplay offset)`:
This function can help you scroll your leds without doing a mem copy.

#### `showPixels(uint8_t * leds,OffDisplay offset)`:
Same function as before, where you can set the led buffer you want to display.

#### Is it reallly needed ?
Maybe not but fun (humm maybe not that fun lol) to make but great results.



 




## Artifacts, DMA, second core, transposition, ...

### Artifacts
The ESP32 is a great controller with Wi-FI ,Bluetooth, RTOS, ... but this can cause the program you're running to stop at certain point in time or interupts not behaving as usual. This happen for a very small amount of time that you usually don't notice but when driving leds this can cause artifacts as these leds are really timing specifics. These artifacts have been reported especially when using wi-fi. A lot of effort has been put in order to avoid the artifacts. And it seems to work quite fine with this version of the driver.

To avoid this issue the idea would be to delegate the signal sending to a peripheral that does not rely on the CPU.

#### DMA and I2S
On the ESP32 it's possible to link a specific type of memory DMA (direct memory access) to the I2S driver (or the SPI). What happen in that case is that the I2S will be fed by the DMA without use of the CPU.
The I2S driver will push the data to the pins at a spefic fixed rate without the use of the CPU too.

That is the technique which used.

#### Great BUT ...
The driver uses two DMA buffers which are linked to one another. (B1 and B2). When B1 is read and pushed by the I2S then it will B2 and push it then back to B1 etc ... . Each time a buffer has finished to be pushed a interrupt occurs. In the driver the first set of pixels is loaded in B1 and during the time that B1 is pushed by the I2S we load B2. Once B1 has been pushed it will move to read B2 and push an interupt that allows us to load the next batch in B1. Once B2 has finished to be pushed an interupt occurs and start reading B1 it then load the next set of pixels in B2 and so on and so forth until all the led has been pushed.

This process works very fine, except of the interupt gets stucks because of wi-fi or something else. The interrupt code is stored in a specific part od the memory IRAM_ATTR that is 'protected' from interupts. **But it can happen that the interupts gets 'pushed' by wi-fi or other internal ticks.**

### A solution : push everything in DMA
The idea is to create a 'big' DMA buffer already filled with all the leds and the tell the I2S to read from that huge buffer. Hence no interupt to take care and the CPU rests during that time.

#### Of course but I need memory
To transmit a RGB pixel we need to transmit 24 bits adn 32 bits for a RGBW pixel. To respect the timing requirements of the leds we send 'ticks' during wich the ourput is high or low. in our case we sent 3 ticks per bits. for
* a 0 bit , 1 0 0 are sent (the output stays high during 416ns and low 834ns)
* a 1 bit , 1 1 0 are sent (the output stays high during 834ns and low 416ns)

NB:  This is the common use approximation of the real timing but it works fine.

Hence to send 24 bit we need 3x24=72 'ticks'. Hence this big DMA buffer will need to need to be 3 times bigger than the led array.

But we are not sending 1 strip at a time but up to 16 strips.

#### Transposition
The driver whatever the number of strips, sends 16 bits (2 bytes ) to the I2S at each 'ticks'. That means for sending 16 parallel pixels (1 pixel of each 16 strips) you need a buffer of size 24x3x2=144 bytes instead of 16x3=48 bytes in the leds array for RGB leds.

The operation that loads the leds of each strips in serie and move it in parallel is called transposition.

As a consequence the size of the big DMA buffer is only link to the `NUM_LED_PER_STRIP` and not the `NUM_STRIPS`. For instance a DMA buffer for 4 strips of 256 leds will be of the same size of 16 strips of 256 leds.

### OK I have enough memory and what else ?
For most of your usage you will have enough memory. Hence the big buffer can be created allowing some new stuff

#### No need for second core
Normally to speed up things, you may program your animation on one core and display on the seconf core using two leds buffers. Here no need. When launching the new function described bellow, The CPU will not be used for the actual 'push' to the leds hence you CPU is free to do someting else.

### Enabling Full DMA buffer
```c
#define FULL_DMA_BUFFER //this will enable the full dma buffer

#define NUM_STRIPS 12
#define NUM_LED_PER_STRIP 256

#include "I2SClocklessLedDriver.h"

I2SClocklessLedDriver driver;

uint8_t leds[4*NUM_STRIPS*NUM_LED_PER_STRIP]; 
int pins[NUM_STRIPS] ={0,2,4,5,12,13,14,15,16,29,25,26};
driver.initled((uint8_t*)leds,pins,NUM_STRIPS,NUM_LED_PER_STRIP,ORDER_GRBW);

```
Now three new functions are available

#### `showPixelsFirstTranpose()` 
This function will transpose the entire led array and the display it. Has this function as en async function when lauching twice it will wait for the first one the finish

**It's like you are running it on a second core without using it**

Example: if you size of your strip is 500 leds it will take 18ms to display
```c
//the duration of the to commands below will be 18ms+18ms =36ms
showPixels();
delay(18);

//the duration of the two commands below will be 19ms 
//the full transposition in the buffer will take 1 ms more or less then the code 
//goes to the delay function has the displaying if the DMA buffer does not require CPU
showPixelsFirstTranpose();
delay(18);


//in the example below if the modifytheledfunction() lasts less than the time  need to display the leds 
//then the second call will wait before starting and then it's like the modifytheledfunction()
showPixelsFirstTranpose();
modifytheledfunction() ....
showPixelsFirstTranpose();

```

Example: `FullBufferFastLED.ino` this example is the equivalent of  `gettingstartedFastLED.ino` but using the buffer. It can be noticed that the overall fps is now higher. 

#### `setPixelinBuffer(uint32_t pos, uint8_t red, uint8_t green, uint8_t blue)`
This function put a pixel directly in the DMA buffer doing the transposition for RGB leds


#### `setPixelinBuffer(uint32_t pos, uint8_t red, uint8_t green, uint8_t blue,uint8_t white)`
This function put a pixel directly in the DMA buffer doing the transposition for RGBW leds

**If you are using these two functions and use `showPixelsFirstTranpose()` it will not work as this function will erase the DMA buffer while transposing the entire led buffer**

To display the content of the DMA buffer directly use

#### `showPixelsFromBuffer()`
This function directly show the leds from the DMA buffer without doing any transposition of the led buffer. 

Example: `FullBufferWithoutTransposition.ino` 

#### Remember what a video chip is ?
A video chip is  in continuously displaying the content of the video RAM (with some perks like for a game boy) without using the CPU at all.

Now you can consider the DMA buffer as video RAM and the video chip as the I2S. We just need to have hte showPixelsFromBuffer to  loop.

`showPixelsFromBuffer(LOOP)` : this function will display the content of the DMA buffer wihtout using the CPU

Example: `FullBufferLoop.ino`  In this example only with one show function

If you want to stop the loop `stopDisplayLoop()`  look at the example `FullBufferLoopStartStop.ino` . The lopp is stopped after 500 'turns' and restart afer 1500.

#### What about frame synchro ??

Using the loop functionality you don't know when you update the DMA buffer which pixel the I2s is currently displaying. As a consequence it can lead to not smooth animations.

If you're animation is not smooth enough we can sync using `waitSync()`.

Example: `FullBufferLoopSync.ino`  play with the `waitSync()` to see the difference.

## Conclusion
I guess I am getting crazy doing that lol. If you have the memory for it then use the DMA buffer and the `showPixelsFirstTranpose()`  if you can you are sure not to have issue with interupts.

If you wanna play old school, use `showPixelsFromBuffer(LOOP)`.  

In any case do not hesitate to contact me for features and remarks

## What is next ?
Add functionnalities to set the sync at any point plus interupt. (for the one old enough it will remind them of souvenirs)
Improve the speed of `setPixelinBuffer` the function is a bit slow for now.









 
 
 
 
 

