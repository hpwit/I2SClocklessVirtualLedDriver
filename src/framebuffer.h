#pragma once
//#include "_pixelslib.h"
#define _NB_FRAME 2

class frameBuffer
{

public:
Pixel * frames[_NB_FRAME];
uint8_t displayframe;
uint8_t writingframe;
uint16_t frame_size;
frameBuffer()
{

}
frameBuffer(Pixel * frame,uint16_t size)
{
    writingframe=0;
    displayframe=0;
    /*
    * we create the frames
    * to add the logic if the memory is not enough
    */
   frame_size = size;
  // printf("size:%d\n",frame_size);
   frames[0]=frame;
    for(int i=1;i<_NB_FRAME;i++)
    {
        frames[i] = (Pixel *)calloc(frame_size,1);
        if(!frames[i])
         printf("no memoory\n");
    }
}
    
    Pixel &operator[](int i)
    {
        return *(frames[0]+i);
    }
    uint8_t * getFrametoDisplay()
    {
        uint8_t  * tmp= (uint8_t *)frames[1];
       memcpy((uint8_t *)frames[1],(uint8_t *)frames[0],frame_size);
        return tmp;
    }
    void switchFrame()
    {
        //writingframe=(writingframe+1)%_NB_FRAME;
       // displayframe=
    }




};

