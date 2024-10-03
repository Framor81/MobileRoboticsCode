#include <U8x8lib.h>

#ifndef DISPLAY_H
#define DISPLAY_H

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display

class Display{
    private:
        // const int SDA_PIN; 
        // const int SCL_PIN;
        // bool inverted;

    Display(int sda_pin, int scl_pin, bool inverted) : SDA_PIN(sda_pin), SCL_PIN(scl_pin), inverted(inverted) {};
    
    void setup() {
        u8x8.begin();
    }

    void loopStep(bool isEnabled, String message) {
        u8x8.setFont(u8x8_font_chroma48medium8_r);
        u8x8.setCursor(8, 16);
        u8x8.print(message);
    }
}



#endif