#include <U8x8lib.h>

#ifndef DISPLAY_H
#define DISPLAY_H


class Display{
    public:
        U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display

        void setup() {
            u8x8.begin();
            u8x8.setFlipMode(1);
        }
};

#endif