// main_test5_oled.c : week1 test5 - OLED (SSD1306, I2C) ONLY
//  wiring : J34 SCL -> PTA3,  J34 SDA -> PTA2   (2 wires). OLED module sits on U8 of the kit
//  action : title, a counter that increments every 0.5 s, upper / lower case font rows, a growing bar
//           EVB green LED (PTD16, on the EVB itself) : OLED answered on I2C, display running
//           EVB red LED   (PTD15)                    : no ACK on 0x3C / 0x3D -> check SCL/SDA wiring, module on U8
//  UART 9600 (OpenSDA, no wiring) : same status as text
#include "tp_board.h"
#include "oled.h"

#define EVB_RED     15
#define EVB_GREEN   16

static void evb_led_init(void)
{
    PORTD->PCR[EVB_RED]   = PORT_PCR_MUX(1);
    PORTD->PCR[EVB_GREEN] = PORT_PCR_MUX(1);
    PTD->PDDR |= (1u << EVB_RED) | (1u << EVB_GREEN);
    PTD->PSOR  = (1u << EVB_RED) | (1u << EVB_GREEN);      // 0 = ON, so set both OFF
}

int main(void)
{
    uint32_t cnt = 0;
    int bar = 0;
    char line[OLED_COLS + 1];

    board_init();
    evb_led_init();
    uart_puts("\r\nTP week1 test5 : OLED only\r\n");

    if (OLED_init() < 0) {
        uart_puts("OLED : no ACK on I2C (0x3C / 0x3D). check SCL->PTA3, SDA->PTA2, module on U8\r\n");
        PTD->PCOR = 1u << EVB_RED;
        for (;;) { }
    }
    PTD->PCOR = 1u << EVB_GREEN;
    uart_puts("OLED : OK, address "); uart_put_uint(OLED_address()); uart_puts(" (60 = 0x3C, 61 = 0x3D)\r\n");

    OLED_puts(0, 0, "SMART PARKING v0.1");
    OLED_puts(2, 0, "COUNT:");
    OLED_puts(4, 0, "ABCDEFGHIJKLMNOPQRSTU");
    OLED_puts(5, 0, "abcdefghijklmnopqrstu");
    OLED_puts(6, 0, "0123456789 +-*/=:.,#");

    for (;;) {
        cnt++;
        OLED_put_uint(2, 7, cnt, 6);

        for (int i = 0; i < OLED_COLS; i++) line[i] = (i < bar) ? '#' : ' ';
        line[OLED_COLS] = 0;
        OLED_puts(7, 0, line);
        if (++bar > OLED_COLS) bar = 0;

        if (OLED_last_error()) {
            PTD->PCOR = 1u << EVB_RED;
            uart_puts("OLED : I2C error\r\n");
        }
        delay_ms(500);
    }
}
