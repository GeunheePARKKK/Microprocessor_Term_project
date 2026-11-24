// main_test1_stepper.c : week1 test1 - stepper motor ONLY
//  wiring : J20-1..4 -> PTB8..PTB11 (4 wires). motor plug on J22, kit power (PWR-JACK) ON
//  action : 90 deg up (512 steps), wait 1 s, 90 deg down, wait 1 s, repeat
//  UART 9600 (OpenSDA, no wiring) : "OPEN" / "CLOSE" log
//  note   : Stepper_move checks SW1 (PTC8). PTC8 has an internal pull-up, so with nothing wired it reads "not pressed".
#include "tp_board.h"

#define STEP_MS   2

int main(void)
{
    board_init();
    uart_puts("\r\nTP week1 test1 : stepper only. 90 deg up / down every 1 s\r\n");

    for (;;) {
        Stepper_move(STEPS_90DEG, +1, STEP_MS);
        uart_puts("OPEN\r\n");
        delay_ms(1000);

        Stepper_move(STEPS_90DEG, -1, STEP_MS);
        uart_puts("CLOSE\r\n");
        delay_ms(1000);
    }
}
