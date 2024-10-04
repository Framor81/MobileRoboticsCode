#include <U8x8lib.h>

#ifndef DISPLAY_H
#define DISPLAY_H

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display

class Display{
    // private:
        // const int SDA_PIN; 
        // const int SCL_PIN;
        // bool inverted;

    // Display(int sda_pin, int scl_pin, bool inverted) : SDA_PIN(sda_pin), SCL_PIN(scl_pin), inverted(inverted) {};
    public:
        void setup() {
            u8x8.begin();
            u8x8.setFlipMode(1);
        }

// i was trying to bring it back
        void loopStep(String message, int x, int y) {
            u8x8.setFont(u8x8_font_chroma48medium8_r);
            u8x8.setCursor(x, y);
            u8x8.print(message);
        }
};

#endif