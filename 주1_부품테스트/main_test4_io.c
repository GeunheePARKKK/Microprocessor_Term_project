#include "tp_board.h"

int main(void)
{
    int key, prev = -1;
    int number = 0;

    board_init();
    uart_puts("\r\nTP week1 test4 : IO check. keypad -> FND, * clear, # buzzer, SW1 -> LED1, SW2 -> LED0\r\n");

    for (;;) {
        key = KeyScan();                                  // 3 ms

        if (key >= 0 && key <= 9 && prev == -1) {
            number = (number * 10 + key) % 10000;
            uart_puts("key "); uart_put_int(key); uart_puts(" -> "); uart_put_int(number); uart_puts("\r\n");
        } else if (key == 11 && prev == -1) {             // '*'
            number = 0;
            uart_puts("key * -> clear\r\n");
        }

        if (key == 12) BUZZER_ON();
        else           BUZZER_OFF();

        if (SW1_PRESSED()) LED1_ON(); else LED1_OFF();
        if (SW2_PRESSED()) LED0_ON(); else LED0_OFF();

        prev = key;
        Seg_out(number);                                  // 4 ms
    }
}
