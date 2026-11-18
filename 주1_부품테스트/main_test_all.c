// main_test_all.c : week1 test4 + test1 + test2 in one program
//  keypad 0-9 : number into FND (shift left), '*' : clear
//  '#'        : short beep, then barrier OPEN (512 steps, LED1 on) -> 1 s -> CLOSE
//               during move, SW1 = emergency stop (buzzer + LED1), SW2 = release and return
//  ultrasonic : measured about every 100 ms, LED0 on when <= 20 cm
//  SW2 (held) : FND shows distance in cm instead of the number ("----" = out of range)
//  UART 9600  : key log, OPEN/CLOSE/EMERGENCY, distance every 0.5 s
#include "tp_board.h"

#define STEP_MS     2
#define DETECT_CM   20

static int  position = 0;   // 0 = closed, STEPS_90DEG = open

static void emergency_wait(void)
{
    BUZZER_ON();
    LED1_ON();
    uart_puts("EMERGENCY STOP at pos "); uart_put_int(position); uart_puts(" - press SW2\r\n");
    while (!SW2_PRESSED()) {
        Seg_out(position);
    }
    BUZZER_OFF();
    LED1_OFF();
    delay_ms(300);
    if (position > 0) {                     // return to closed position
        Stepper_move(position, -1, STEP_MS);
        position = 0;
    }
    uart_puts("RELEASED, back to pos 0\r\n");
}

static void barrier_cycle(void)
{
    int remain;

    BUZZER_ON(); delay_ms(100); BUZZER_OFF();

    LED1_ON();
    remain = Stepper_move(STEPS_90DEG, +1, STEP_MS);
    position += STEPS_90DEG - remain;
    if (remain) { emergency_wait(); return; }
    uart_puts("OPEN  (pos "); uart_put_int(position); uart_puts(")\r\n");
    delay_ms(1000);

    remain = Stepper_move(STEPS_90DEG, -1, STEP_MS);
    position -= STEPS_90DEG - remain;
    if (remain) { emergency_wait(); return; }
    LED1_OFF();
    uart_puts("CLOSE (pos "); uart_put_int(position); uart_puts(")\r\n");
}

int main(void)
{
    int key, prev = -1;
    int number = 0;
    int loop = 0, log_cnt = 0;
    uint32_t cm = 0, t0, meas_us = 0;

    board_init();
    uart_puts("\r\nTP week1 test_all : keypad->FND, * clear, # barrier cycle, SW1 emergency, SW2 = show cm\r\n");

    for (;;) {
        key = KeyScan();                                   // 3 ms

        if (key >= 0 && key <= 9 && prev == -1) {
            number = (number * 10 + key) % 10000;
            uart_puts("key "); uart_put_int(key); uart_puts(" -> "); uart_put_int(number); uart_puts("\r\n");
        } else if (key == 11 && prev == -1) {              // '*'
            number = 0;
            uart_puts("key * -> clear\r\n");
        } else if (key == 12 && prev == -1) {              // '#'
            uart_puts("key # -> barrier cycle\r\n");
            barrier_cycle();
            prev = -1;
            continue;
        }
        prev = key;

        // ultrasonic about every 100 ms (loop = KeyScan 3 ms + Seg_out 4 ms = 7 ms, 14 loops = 98 ms)
        if (++loop >= 14) {
            loop = 0;
            t0 = dwt_now();
            cm = Ultrasonic_measure_cm();
            meas_us = dwt_elapsed_us(t0);
            if (cm > 0 && cm <= DETECT_CM) LED0_ON();
            else                           LED0_OFF();
            if (++log_cnt >= 5) {                          // every 0.5 s
                log_cnt = 0;
                uart_puts("DIST ");
                if (cm == 0) uart_puts("---");
                else         uart_put_uint(cm);
                uart_puts(" cm (measure "); uart_put_uint(meas_us); uart_puts(" us)\r\n");
            }
        }

        if (SW2_PRESSED()) {                               // show distance while SW2 held
            if (cm == 0) Seg_out_dash();
            else         Seg_out((int)cm);
        } else {
            Seg_out(number);                               // 4 ms
        }
    }
}
