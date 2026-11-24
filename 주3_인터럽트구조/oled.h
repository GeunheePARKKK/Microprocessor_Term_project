// oled.h : SSD1306 OLED (I2C) on kit "OLED(I2C)" block, driven by S32K144 LPI2C0
//  wiring : J34 SCL (pin 1 or 2) -> PTA3 (LPI2C0_SCL, J1-03),  J34 SDA (pin 3 or 4) -> PTA2 (LPI2C0_SDA, J1-01)
//  text   : 8 rows (0..7) x 21 columns (0..20), 6x8 pixel font
#ifndef OLED_H
#define OLED_H

#include <stdint.h>

#define OLED_WIDTH    128
#define OLED_HEIGHT   64         // 0.96" module = 64, 0.91" module = 32
#define OLED_ROWS     (OLED_HEIGHT / 8)
#define OLED_COLS     (OLED_WIDTH / 6)

int  OLED_init(void);                                   // returns 0 on success, -1 if no ACK from the display
void OLED_clear(void);
void OLED_puts(uint8_t row, uint8_t col, const char *s);
void OLED_put_uint(uint8_t row, uint8_t col, uint32_t v, uint8_t width);   // right aligned in 'width' chars
void OLED_put_int(uint8_t row, uint8_t col, int32_t v, uint8_t width);
int  OLED_last_error(void);                             // 0 = ok, -1 = NACK / timeout on last transfer
uint8_t OLED_address(void);                             // 0x3C or 0x3D, whichever answered

#endif
