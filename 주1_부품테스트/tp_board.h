#ifndef TP_BOARD_H
#define TP_BOARD_H

#include "device_registers.h"
#include "clocks_and_modes.h"
#include <stdint.h>

#define FND_SEG_SHIFT   1          // PTD1 ~ PTD7 = SEG A ~ G
#define FND_SEG_MASK    (0x7Fu << FND_SEG_SHIFT)
#define FND_COM_SHIFT   8          // PTD8 ~ PTD11 = CM1 ~ CM4
#define FND_COM_MASK    (0x0Fu << FND_COM_SHIFT)

#define KEY_ROW0        0
#define KEY_COL1        12
#define KEY_COL2        14         // PTE14 = COL2
#define KEY_COL3        15         // PTE15 = COL3

#define STEP_SHIFT      8          // PTB8 ~ PTB11 = ST_IN1 ~ ST_IN4
#define STEP_MASK       (0x0Fu << STEP_SHIFT)

#define US_TRIG         16
#define US_ECHO         17

#define SW1_PIN         8
#define SW2_PIN         9

#define BUZZER_PIN      12
#define LED0_PIN        8
#define LED1_PIN        9
#define TOGGLE_PIN      16

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

void board_init(void);

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

void     dwt_init(void);
uint32_t dwt_now(void);
uint32_t dwt_elapsed_us(uint32_t t0);

void Seg_out(int number);
void Seg_out_dash(void);
int  KeyScan(void);

void LPUART1_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_put_uint(uint32_t v);
void uart_put_int(int32_t v);
int  uart_getc_nonblock(void);

void Stepper_step(int dir);
void Stepper_release(void);
int  Stepper_move(int steps, int dir, uint32_t step_ms);
#define STEPS_PER_REV   2048
#define STEPS_90DEG     (STEPS_PER_REV / 4)

uint32_t Ultrasonic_measure_cm(void);

#endif
