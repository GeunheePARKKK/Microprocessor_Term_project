#include "tp_board.h"

#define DETECT_CM   20

int main(void)
{
    uint32_t cm = 0, t0, meas_us;
    int tick = 0;

    board_init();
    uart_puts("\r\nTP week1 test2 : ultrasonic. FND = cm, LED0 = within 20 cm\r\n");

    for (;;) {
        for (int i = 0; i < 24; i++) {
            if (cm == 0) Seg_out_dash();
            else         Seg_out((int)cm);
        }

        t0 = dwt_now();
        cm = Ultrasonic_measure_cm();
        meas_us = dwt_elapsed_us(t0);

        if (cm > 0 && cm <= DETECT_CM) LED0_ON();
        else                           LED0_OFF();

        if (++tick >= 5) {
            tick = 0;
            uart_puts("DIST ");
            if (cm == 0) uart_puts("---");
            else         uart_put_uint(cm);
            uart_puts(" cm  (measure ");
            uart_put_uint(meas_us);
            uart_puts(" us)\r\n");
        }
    }
}
