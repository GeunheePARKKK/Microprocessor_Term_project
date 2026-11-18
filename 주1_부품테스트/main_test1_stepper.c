#include "tp_board.h"

#define STEP_MS   2

int main(void)
{
    int position = 0;
    int remain;

    board_init();
    uart_puts("\r\nTP week1 test1 : stepper 90 deg up/down. SW1 = emergency stop, SW2 = release\r\n");

    for (;;) {
        LED0_ON();
        remain = Stepper_move(STEPS_90DEG, +1, STEP_MS);
        position += STEPS_90DEG - remain;
        if (remain) goto emergency;
        uart_puts("OPEN  (pos "); uart_put_int(position); uart_puts(")\r\n");
        delay_ms(1000);

        remain = Stepper_move(STEPS_90DEG, -1, STEP_MS);
        position -= STEPS_90DEG - remain;
        if (remain) goto emergency;
        LED0_OFF();
        uart_puts("CLOSE (pos "); uart_put_int(position); uart_puts(")\r\n");
        delay_ms(1000);
        continue;

emergency:
        BUZZER_ON();
        LED1_ON();
        uart_puts("EMERGENCY STOP at pos "); uart_put_int(position); uart_puts(" - press SW2\r\n");
        while (!SW2_PRESSED()) {
            Seg_out(position);
        }
        BUZZER_OFF();
        LED1_OFF();
        delay_ms(300);
        if (position > 0) {
            Stepper_move(position, -1, STEP_MS);
            position = 0;
        }
        LED0_OFF();
        delay_ms(1000);
    }
}
