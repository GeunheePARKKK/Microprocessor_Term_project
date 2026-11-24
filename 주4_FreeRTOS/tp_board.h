// tp_board.h : board support for structure (2) interrupt and (3) FreeRTOS  (week 3 / week 4)
//  same pins as week 1. differences from the week 1 version :
//   - delay_us / delay_ms use LPIT ch2 (ch0 is free for the 1 ms interrupt)
//   - FND : FND_set() + FND_tick() (one digit per call, no delay inside)
//   - keypad : KeyScan_fast() (50 us settle per column instead of 1 ms)
//   - UART : queued, non blocking output (uart_q_*) in addition to the blocking one
//   - ultrasonic : trigger only. the echo is measured in PORTC_IRQHandler (written in main.c)
//   - LPIT_ch0_irq_init / PORTC_irq_init / NVIC_enable_irq. the handler bodies live in main.c
#ifndef TP_BOARD_H
#define TP_BOARD_H

#include "device_registers.h"
#include "clocks_and_modes.h"
#include <stdint.h>

#define FND_SEG_SHIFT   1          // PTD1 ~ PTD7 = SEG A ~ G
#define FND_SEG_MASK    (0x7Fu << FND_SEG_SHIFT)
#define FND_COM_SHIFT   8          // PTD8 ~ PTD11 = CM1 ~ CM4
#define FND_COM_MASK    (0x0Fu << FND_COM_SHIFT)

#define KEY_COL1        12
#define KEY_COL2        14
#define KEY_COL3        15

#define STEP_SHIFT      8          // PTB8 ~ PTB11 = ST_IN1 ~ ST_IN4
#define STEP_MASK       (0x0Fu << STEP_SHIFT)

#define US_TRIG         16         // PTC16
#define US_ECHO         17         // PTC17

#define SW1_PIN         8          // PTC8
#define SW2_PIN         9          // PTC9

#define BUZZER_PIN      12         // PTD12
#define LED0_PIN        8          // PTE8
#define LED1_PIN        9          // PTE9
#define TOGGLE_PIN      16         // PTE16

#define SW1_PRESSED()   ((PTC->PDIR & (1u << SW1_PIN)) == 0)
#define SW2_PRESSED()   ((PTC->PDIR & (1u << SW2_PIN)) == 0)
#define BUZZER_ON()     (PTD->PSOR = 1u << BUZZER_PIN)
#define BUZZER_OFF()    (PTD->PCOR = 1u << BUZZER_PIN)
#define LED0_ON()       (PTE->PCOR = 1u << LED0_PIN)
#define LED0_OFF()      (PTE->PSOR = 1u << LED0_PIN)
#define LED1_ON()       (PTE->PCOR = 1u << LED1_PIN)
#define LED1_OFF()      (PTE->PSOR = 1u << LED1_PIN)
#define TOGGLE_HIGH()   (PTE->PSOR = 1u << TOGGLE_PIN)
#define TOGGLE_LOW()    (PTE->PCOR = 1u << TOGGLE_PIN)

#define DWT_CTRL        (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT      (*(volatile uint32_t *)0xE0001004u)
#define DEMCR           (*(volatile uint32_t *)0xE000EDFCu)
#define CYCLES_PER_US   80u
#define DWT_SPIN_US(us) do { uint32_t _t0 = DWT_CYCCNT; while ((DWT_CYCCNT - _t0) < (us) * CYCLES_PER_US) {} } while (0)

#define LPIT0_CH0_IRQ   48u
#define PORTC_IRQ       61u

#define STEPS_PER_REV   2048
#define STEPS_90DEG     (STEPS_PER_REV / 4)

void board_init(void);                    // 80 MHz, ports, LPIT (ch2 delay), DWT, LPUART1. no interrupt is enabled here

void delay_us(uint32_t us);               // LPIT ch2 polling. (3) : only before the scheduler starts
void delay_ms(uint32_t ms);

void     dwt_init(void);
uint32_t dwt_now(void);
uint32_t dwt_elapsed_us(uint32_t t0);

void FND_set(int number);                 // 0..9999 to show, -1 = "----"
void FND_tick(void);                      // call every 1 ms : lights the next digit

int  KeyScan_fast(void);                  // 0..9, 11 = '*', 12 = '#', -1 = none. about 150 us total

void LPUART1_init(void);
void uart_putc(char c);                   // blocking (waits for TDRE)
void uart_puts(const char *s);
void uart_put_uint(uint32_t v);
void uart_put_int(int32_t v);
int  uart_getc_nonblock(void);

void uart_q_puts(const char *s);          // into a 1 KB ring buffer, never blocks (drops if full)
void uart_q_put_uint(uint32_t v);
void uart_q_put_int(int32_t v);
int  uart_q_pump(void);                   // sends one queued char if the TX register is free. 1 = sent, 0 = nothing
int  uart_q_pending(void);                // chars still queued
uint32_t uart_q_dropped(void);            // chars dropped because the buffer was full

void Stepper_step(int dir);               // +1 / -1, one step
void Stepper_release(void);               // all 4 coils off

void     Ultrasonic_trigger(void);        // 10 us TRIG pulse
uint32_t Ultrasonic_cycles_to_cm(uint32_t cycles);   // DWT cycles of the echo pulse -> cm

void NVIC_enable_irq(uint32_t irq, uint8_t prio);  // prio 0 (highest) .. 15
void LPIT_ch0_irq_init(uint32_t period_us);        // periodic interrupt on LPIT0 ch0. body : void LPIT0_Ch0_IRQHandler(void) in main.c
void PORTC_irq_init(void);                         // ECHO both edges, SW1 falling edge.  body : void PORTC_IRQHandler(void) in main.c
#define LPIT_CH0_CLEAR_FLAG()   (LPIT0->MSR = LPIT_MSR_TIF0_MASK)
#define PORTC_ISF(pin)          (PORTC->ISFR & (1u << (pin)))
#define PORTC_CLEAR_ISF(pin)    (PORTC->ISFR = (1u << (pin)))

#endif
