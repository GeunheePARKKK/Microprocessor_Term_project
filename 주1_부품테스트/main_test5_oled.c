// main_test5_oled.c : week1 test5 - OLED (SSD1306, I2C) check
//  wiring : J34 SCL -> PTA3, J34 SDA -> PTA2 (OLED module sits on U8 of the kit)
//  shows  : title, keypad number, SW1/SW2 state, a counter that increments every 0.5 s, ultrasonic cm if available
//  LED1 on + UART message if the display does not answer on I2C
#include "tp_board.h"
#include "oled.h"

int main(void)
{
    int key, prev = -1, number = 0;
    uint32_t cnt = 0, cm = 0;
    int loop = 0;

    board_init();
    uart_puts("\r\nTP week1 test5 : OLED\r\n");

    if (OLED_init() < 0) {
        uart_puts("OLED : no ACK on I2C (0x3C / 0x3D). check SCL->PTA3, SDA->PTA2, module on U8\r\n");
        LED1_ON();
        for (;;) { Seg_out_dash(); }
    }
    uart_puts("OLED : OK, address "); uart_put_uint(OLED_address()); uart_puts(" (60 = 0x3C, 61 = 0x3D)\r\n");

    OLED_puts(0, 0, "SMART PARKING v0.1");
    OLED_puts(2, 0, "KEY :");
    OLED_puts(3, 0, "DIST:        cm");
    OLED_puts(5, 0, "SW1:   SW2:");
    OLED_puts(7, 0, "CNT :");

    for (;;) {
        key = KeyScan();                                   // 3 ms
        if (key >= 0 && key <= 9 && prev == -1) {
            number = (number * 10 + key) % 10000;
            OLED_put_uint(2, 6, (uint32_t)number, 4);
        } else if (key == 11 && prev == -1) {
            number = 0;
            OLED_put_uint(2, 6, 0, 4);
        }
        prev = key;

        OLED_puts(5, 4, SW1_PRESSED() ? "ON " : "off");
        OLED_puts(5, 11, SW2_PRESSED() ? "ON " : "off");

        if (++loop >= 70) {                                // about 0.5 s (loop = 3 ms keypad + 4 ms FND)
            loop = 0;
            cnt++;
            OLED_put_uint(7, 6, cnt, 6);
            cm = Ultrasonic_measure_cm();
            if (cm == 0) OLED_puts(3, 6, "  ---");
            else         OLED_put_uint(3, 6, cm, 5);
            if (OLED_last_error()) { LED1_ON(); uart_puts("OLED : I2C error\r\n"); }
            else                   { LED1_OFF(); }
        }
        Seg_out(number);                                   // 4 ms
    }
}
