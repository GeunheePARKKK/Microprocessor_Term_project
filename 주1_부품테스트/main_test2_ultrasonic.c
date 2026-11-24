// main_test2_ultrasonic.c : week1 test2 - ultrasonic sensor ONLY (+ one kit LED)
//  wiring : J27-3 (UW_TRIG) -> PTC16,  J27-1 (UW_ECHO) -> PTC17,  J16-1 (LED0) -> PTE8   (3 wires)
//  action : measure every 100 ms
//           kit LED0 (PTE8)        : ON while something is within 20 cm
//           EVB green LED (PTD16)  : ON while the last echo was valid (any distance)   - no wiring, on the EVB itself
//           EVB red LED   (PTD15)  : ON while no echo came back (timeout)              - wiring / sensor problem
//  UART 9600 (OpenSDA, no wiring) : distance in cm every 0.5 s
#include "tp_board.h"

#define DETECT_CM   20

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
    uint32_t cm = 0;
    int tick = 0;

    board_init();
    evb_led_init();
    uart_puts("\r\nTP week1 test2 : ultrasonic only. LED0 = within 20 cm, EVB green = echo ok, EVB red = no echo\r\n");

    for (;;) {
        cm = Ultrasonic_measure_cm();                      // 0 = timeout (no echo or out of range)

        if (cm > 0 && cm <= DETECT_CM) LED0_ON();
        else                           LED0_OFF();

        if (cm > 0) { PTD->PCOR = 1u << EVB_GREEN; PTD->PSOR = 1u << EVB_RED;   }
        else        { PTD->PSOR = 1u << EVB_GREEN; PTD->PCOR = 1u << EVB_RED;   }

        if (++tick >= 5) {                                 // every 0.5 s
            tick = 0;
            uart_puts("DIST ");
            if (cm == 0) uart_puts("--- (no echo)");
            else       { uart_put_uint(cm); uart_puts(" cm"); }
            uart_puts("\r\n");
        }
        delay_ms(100);
    }
}
