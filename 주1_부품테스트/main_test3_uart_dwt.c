// main_test3_uart_dwt.c : week1 test3 - UART + DWT timing check, NO wiring needed
//  UART 9600 (OpenSDA USB) : prints a timing table (delay, Seg_out, KeyScan, UART, ultrasonic, stepper). type m to repeat
//  the measured functions run on unconnected pins; that is fine, we only measure their time here
#include "tp_board.h"

static void report(const char *name, uint32_t us)
{
    uart_puts("  ");
    uart_puts(name);
    uart_puts(" : ");
    uart_put_uint(us);
    uart_puts(" us\r\n");
}

static void measure_all(void)
{
    uint32_t t0, overhead;

    uart_puts("\r\n=== DWT measurement (80 MHz, 1 us = 80 cycles) ===\r\n");

    t0 = dwt_now();
    overhead = DWT_CYCCNT - t0;
    uart_puts("  empty section : "); uart_put_uint(overhead); uart_puts(" cycles\r\n");

    t0 = dwt_now(); delay_us(100);   report("delay_us(100)  ", dwt_elapsed_us(t0));
    t0 = dwt_now(); delay_ms(1);     report("delay_ms(1)    ", dwt_elapsed_us(t0));
    t0 = dwt_now(); delay_ms(10);    report("delay_ms(10)   ", dwt_elapsed_us(t0));

    t0 = dwt_now(); Seg_out(1234);   report("Seg_out (4 digit)", dwt_elapsed_us(t0));
    t0 = dwt_now(); (void)KeyScan(); report("KeyScan (3 col)  ", dwt_elapsed_us(t0));
    t0 = dwt_now(); uart_puts("12345678901234567890");
                                     report("\r\n  UART 20 chars   ", dwt_elapsed_us(t0));
    t0 = dwt_now(); (void)Ultrasonic_measure_cm();
                                     report("Ultrasonic (1 shot)", dwt_elapsed_us(t0));
    t0 = dwt_now(); Stepper_step(+1); Stepper_release();
                                     report("Stepper 1 step   ", dwt_elapsed_us(t0));

    TOGGLE_HIGH(); delay_ms(1); TOGGLE_LOW();
    uart_puts("  PTE16 : 1 ms pulse sent (check with scope)\r\n");
    uart_puts("=== end. type any key (echo), 'm' = measure again ===\r\n");
}

int main(void)
{
    int c;

    board_init();
    uart_puts("\r\nTP week1 test3 : UART OK\r\n");
    measure_all();

    for (;;) {
        c = uart_getc_nonblock();
        if (c >= 0) {
            uart_putc((char)c);
            if (c == 'm' || c == 'M') measure_all();
        }
    }
}
