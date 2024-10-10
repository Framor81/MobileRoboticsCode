#include <U8x8lib.h>

#ifndef DISPLAY_H
#define DISPLAY_H


class Display{
    public:
      U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8;

      void setup() {
        u8x8 = U8X8_SSD1306_128X64_NONAME_HW_I2C(SCL, SDA, U8X8_PIN_NONE); 
        u8x8.begin();
        u8x8.setFlipMode(1);
        u8x8.setFont(u8x8_font_chroma48medium8_r);
      }

      void loopstep(int x, int y, const char* str) {
        u8x8.drawString(x, y, str);
      }
};

#endif