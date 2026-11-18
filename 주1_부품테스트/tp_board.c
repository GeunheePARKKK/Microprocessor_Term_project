#include "tp_board.h"

static const uint16_t FND_DATA[11] = {0x7E, 0x0C, 0xB6, 0x9E, 0xCC, 0xDA, 0xFA, 0x4E, 0xFE, 0xCE,
                                      0x80};
static const uint16_t FND_SEL[4]   = {0x0100, 0x0200, 0x0400, 0x0800};   // CM1 ~ CM4 = PTD8 ~ PTD11

static void WDOG_disable(void)
{
    WDOG->CNT   = 0xD928C520;
    WDOG->TOVAL = 0x0000FFFF;
    WDOG->CS    = 0x00002100;
}

static void PORT_init(void)
{
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;

    for (int p = 1; p <= 12; p++) {
        PORTD->PCR[p] = PORT_PCR_MUX(1);
    }
    PTD->PDDR |= FND_SEG_MASK | FND_COM_MASK | (1u << BUZZER_PIN);
    PTD->PCOR  = FND_SEG_MASK | FND_COM_MASK | (1u << BUZZER_PIN);

    for (int p = 0; p <= 3; p++) {
        PORTE->PCR[p] = PORT_PCR_MUX(1) | PORT_PCR_PFE_MASK | PORT_PCR_PE(1) | PORT_PCR_PS(0);
        PTE->PDDR &= ~(1u << p);
    }
    PORTE->PCR[KEY_COL1] = PORT_PCR_MUX(1);
    PORTE->PCR[KEY_COL2] = PORT_PCR_MUX(1);
    PORTE->PCR[KEY_COL3] = PORT_PCR_MUX(1);
    PTE->PDDR |= (1u << KEY_COL1) | (1u << KEY_COL2) | (1u << KEY_COL3);
    PTE->PCOR  = (1u << KEY_COL1) | (1u << KEY_COL2) | (1u << KEY_COL3);

    PORTE->PCR[LED0_PIN]   = PORT_PCR_MUX(1);
    PORTE->PCR[LED1_PIN]   = PORT_PCR_MUX(1);
    PORTE->PCR[TOGGLE_PIN] = PORT_PCR_MUX(1);
    PTE->PDDR |= (1u << LED0_PIN) | (1u << LED1_PIN) | (1u << TOGGLE_PIN);
    PTE->PSOR  = (1u << LED0_PIN) | (1u << LED1_PIN);
    PTE->PCOR  = (1u << TOGGLE_PIN);

    for (int p = 8; p <= 11; p++) {
        PORTB->PCR[p] = PORT_PCR_MUX(1);
    }
    PTB->PDDR |= STEP_MASK;
    PTB->PCOR  = STEP_MASK;

    PORTC->PCR[US_TRIG] = PORT_PCR_MUX(1);
    PORTC->PCR[US_ECHO] = PORT_PCR_MUX(1);
    PTC->PDDR |=  (1u << US_TRIG);
    PTC->PDDR &= ~(1u << US_ECHO);
    PTC->PCOR  =  (1u << US_TRIG);

    PORTC->PCR[SW1_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PFE_MASK | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTC->PCR[SW2_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PFE_MASK | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PTC->PDDR &= ~((1u << SW1_PIN) | (1u << SW2_PIN));

    PORTC->PCR[6] = PORT_PCR_MUX(2);
    PORTC->PCR[7] = PORT_PCR_MUX(2);
}

static void LPIT_init(void)
{
    PCC->PCCn[PCC_LPIT_INDEX]  = PCC_PCCn_PCS(6);
    PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK;
    LPIT0->MCR = LPIT_MCR_M_CEN_MASK;
}

void board_init(void)
{
    WDOG_disable();
    SOSC_init_8MHz();
    SPLL_init_160MHz();        // PLL 160 MHz
    NormalRUNmode_80MHz();
    PORT_init();
    LPIT_init();
    dwt_init();
    LPUART1_init();
}

void delay_us(uint32_t us)
{
    LPIT0->TMR[0].TCTRL = 0;
    LPIT0->TMR[0].TVAL  = us * 40u;
    LPIT0->MSR          = LPIT_MSR_TIF0_MASK;
    LPIT0->TMR[0].TCTRL = LPIT_TMR_TCTRL_T_EN_MASK;
    while ((LPIT0->MSR & LPIT_MSR_TIF0_MASK) == 0) {}
    LPIT0->MSR = LPIT_MSR_TIF0_MASK;
}

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

void dwt_init(void)
{
    DEMCR     |= (1u << 24);
    DWT_CYCCNT = 0;
    DWT_CTRL  |= 1u;
}

uint32_t dwt_now(void)
{
    return DWT_CYCCNT;
}

uint32_t dwt_elapsed_us(uint32_t t0)
{
    return (DWT_CYCCNT - t0) / CYCLES_PER_US;
}

static void seg_digit(int pos, uint16_t pattern)
{
    PTD->PCOR = FND_SEG_MASK | FND_COM_MASK;
    PTD->PSOR = FND_SEL[pos] | pattern;
    delay_us(1000);
    PTD->PCOR = FND_SEG_MASK | FND_COM_MASK;
}

void Seg_out(int number)
{
    if (number < 0)      number = 0;
    if (number > 9999)   number = 9999;
    seg_digit(0, FND_DATA[(number / 1000) % 10]);
    seg_digit(1, FND_DATA[(number / 100) % 10]);
    seg_digit(2, FND_DATA[(number / 10) % 10]);
    seg_digit(3, FND_DATA[number % 10]);
}

void Seg_out_dash(void)
{
    for (int i = 0; i < 4; i++) {
        seg_digit(i, FND_DATA[10]);
    }
}

static int key_col(int col_pin, int k1, int k2, int k3, int k4)
{
    int k = -1;
    PTE->PSOR = 1u << col_pin;
    delay_us(1000);
    if (PTE->PDIR & (1u << 0)) k = k1;
    if (PTE->PDIR & (1u << 1)) k = k2;
    if (PTE->PDIR & (1u << 2)) k = k3;
    if (PTE->PDIR & (1u << 3)) k = k4;
    PTE->PCOR = 1u << col_pin;
    return k;
}

int KeyScan(void)
{
    int k;
    k = key_col(KEY_COL1, 1, 4, 7, 11);        // COL1 : 1 4 7 *
    if (k >= 0) return k;
    k = key_col(KEY_COL2, 2, 5, 8, 0);         // COL2 : 2 5 8 0
    if (k >= 0) return k;
    k = key_col(KEY_COL3, 3, 6, 9, 12);        // COL3 : 3 6 9 #
    return k;
}

void LPUART1_init(void)
{
    PCC->PCCn[PCC_LPUART1_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_LPUART1_INDEX] |= PCC_PCCn_PCS(1) | PCC_PCCn_CGC_MASK;
    LPUART1->BAUD = 0x0F000034;
    LPUART1->CTRL = 0x000C0000;
}

void uart_putc(char c)
{
    while ((LPUART1->STAT & LPUART_STAT_TDRE_MASK) == 0) {}
    LPUART1->DATA = (uint8_t)c;
}

void uart_puts(const char *s)
{
    while (*s) {
        uart_putc(*s++);
    }
}

void uart_put_uint(uint32_t v)
{
    char buf[11];
    int  i = 10;
    buf[i] = '\0';
    do {
        buf[--i] = (char)('0' + v % 10);
        v /= 10;
    } while (v);
    uart_puts(&buf[i]);
}

void uart_put_int(int32_t v)
{
    if (v < 0) {
        uart_putc('-');
        v = -v;
    }
    uart_put_uint((uint32_t)v);
}

int uart_getc_nonblock(void)
{
    if (LPUART1->STAT & LPUART_STAT_RDRF_MASK) {
        return (int)(LPUART1->DATA & 0xFF);
    }
    return -1;
}

static const uint8_t STEP_SEQ[4] = {0x3, 0x6, 0xC, 0x9};
static int step_idx = 0;

void Stepper_step(int dir)
{
    step_idx = (step_idx + (dir > 0 ? 1 : 3)) & 3;
    PTB->PCOR = STEP_MASK;
    PTB->PSOR = (uint32_t)STEP_SEQ[step_idx] << STEP_SHIFT;
}

void Stepper_release(void)
{
    PTB->PCOR = STEP_MASK;
}

int Stepper_move(int steps, int dir, uint32_t step_ms)
{
    for (int i = 0; i < steps; i++) {
        if (SW1_PRESSED()) {
            Stepper_release();
            return steps - i;
        }
        Stepper_step(dir);
        delay_ms(step_ms);
    }
    Stepper_release();
    return 0;
}

uint32_t Ultrasonic_measure_cm(void)
{
    uint32_t t0, width_us;

    PTC->PCOR = 1u << US_TRIG;
    delay_us(2);
    PTC->PSOR = 1u << US_TRIG;
    delay_us(10);
    PTC->PCOR = 1u << US_TRIG;

    t0 = dwt_now();
    while ((PTC->PDIR & (1u << US_ECHO)) == 0) {
        if (dwt_elapsed_us(t0) > 30000) return 0;
    }
    t0 = dwt_now();
    while (PTC->PDIR & (1u << US_ECHO)) {
        if (dwt_elapsed_us(t0) > 30000) return 0;
    }
    width_us = dwt_elapsed_us(t0);
    return width_us / 58u;
}
