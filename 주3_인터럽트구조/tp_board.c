// tp_board.c : board support for structure (2) and (3). see tp_board.h
#include "tp_board.h"

static const uint16_t FND_DATA[11] = {0x7E, 0x0C, 0xB6, 0x9E, 0xCC, 0xDA, 0xFA, 0x4E, 0xFE, 0xCE,
                                      0x80};                                  // index 10 = '-'
static const uint16_t FND_SEL[4]   = {0x0100, 0x0200, 0x0400, 0x0800};       // CM1 ~ CM4 = PTD8 ~ PTD11

// ---------------- clocks / ports ----------------
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

    for (int p = 1; p <= 12; p++) {                                   // FND + buzzer
        PORTD->PCR[p] = PORT_PCR_MUX(1);
    }
    PTD->PDDR |= FND_SEG_MASK | FND_COM_MASK | (1u << BUZZER_PIN);
    PTD->PCOR  = FND_SEG_MASK | FND_COM_MASK | (1u << BUZZER_PIN);

    for (int p = 0; p <= 3; p++) {                                    // keypad rows, pull-down
        PORTE->PCR[p] = PORT_PCR_MUX(1) | PORT_PCR_PFE_MASK | PORT_PCR_PE(1) | PORT_PCR_PS(0);
        PTE->PDDR &= ~(1u << p);
    }
    PORTE->PCR[KEY_COL1] = PORT_PCR_MUX(1);                           // keypad columns
    PORTE->PCR[KEY_COL2] = PORT_PCR_MUX(1);
    PORTE->PCR[KEY_COL3] = PORT_PCR_MUX(1);
    PTE->PDDR |= (1u << KEY_COL1) | (1u << KEY_COL2) | (1u << KEY_COL3);
    PTE->PCOR  = (1u << KEY_COL1) | (1u << KEY_COL2) | (1u << KEY_COL3);

    PORTE->PCR[LED0_PIN]   = PORT_PCR_MUX(1);                         // LEDs (0 = on), toggle pin
    PORTE->PCR[LED1_PIN]   = PORT_PCR_MUX(1);
    PORTE->PCR[TOGGLE_PIN] = PORT_PCR_MUX(1);
    PTE->PDDR |= (1u << LED0_PIN) | (1u << LED1_PIN) | (1u << TOGGLE_PIN);
    PTE->PSOR  = (1u << LED0_PIN) | (1u << LED1_PIN) | (1u << TOGGLE_PIN);   // LEDs off, toggle high (= SW1 line idle)

    for (int p = 8; p <= 11; p++) {                                   // stepper
        PORTB->PCR[p] = PORT_PCR_MUX(1);
    }
    PTB->PDDR |= STEP_MASK;
    PTB->PCOR  = STEP_MASK;

    PORTC->PCR[US_TRIG] = PORT_PCR_MUX(1);                            // ultrasonic
    PORTC->PCR[US_ECHO] = PORT_PCR_MUX(1);
    PTC->PDDR |=  (1u << US_TRIG);
    PTC->PDDR &= ~(1u << US_ECHO);
    PTC->PCOR  =  (1u << US_TRIG);

    PORTC->PCR[SW1_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PFE_MASK | PORT_PCR_PE(1) | PORT_PCR_PS(1);   // switches, pull-up
    PORTC->PCR[SW2_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PFE_MASK | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PTC->PDDR &= ~((1u << SW1_PIN) | (1u << SW2_PIN));

    PORTC->PCR[6] = PORT_PCR_MUX(2);                                  // LPUART1 RX / TX
    PORTC->PCR[7] = PORT_PCR_MUX(2);
}

static void LPIT_init(void)
{
    PCC->PCCn[PCC_LPIT_INDEX]  = PCC_PCCn_PCS(6);                     // SPLLDIV2 = 40 MHz
    PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK;
    LPIT0->MCR = LPIT_MCR_M_CEN_MASK;
}

void board_init(void)
{
    WDOG_disable();
    SOSC_init_8MHz();
    SPLL_init_160MHz();
    NormalRUNmode_80MHz();
    PORT_init();
    LPIT_init();
    dwt_init();
    LPUART1_init();
}

// ---------------- delay (LPIT ch2 polling) ----------------
void delay_us(uint32_t us)
{
    LPIT0->TMR[2].TCTRL = 0;
    LPIT0->TMR[2].TVAL  = us * 40u;
    LPIT0->MSR          = LPIT_MSR_TIF2_MASK;
    LPIT0->TMR[2].TCTRL = LPIT_TMR_TCTRL_T_EN_MASK;
    while ((LPIT0->MSR & LPIT_MSR_TIF2_MASK) == 0) {}
    LPIT0->MSR          = LPIT_MSR_TIF2_MASK;
    LPIT0->TMR[2].TCTRL = 0;
}

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

// ---------------- DWT ----------------
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

// ---------------- FND ----------------
static volatile int fnd_value = 0;
static int          fnd_pos   = 0;

void FND_set(int number)
{
    if (number > 9999) number = 9999;
    if (number < -1)   number = -1;
    fnd_value = number;
}

void FND_tick(void)
{
    int      v = fnd_value;
    uint16_t pattern;

    if (v < 0) {
        pattern = FND_DATA[10];
    } else {
        int d = (fnd_pos == 0) ? (v / 1000) % 10 :
                (fnd_pos == 1) ? (v / 100)  % 10 :
                (fnd_pos == 2) ? (v / 10)   % 10 : v % 10;
        pattern = FND_DATA[d];
    }
    PTD->PCOR = FND_SEG_MASK | FND_COM_MASK;
    PTD->PSOR = FND_SEL[fnd_pos] | pattern;
    fnd_pos = (fnd_pos + 1) & 3;
}

// ---------------- keypad ----------------
static int key_col(int col_pin, int k1, int k2, int k3, int k4)
{
    int k = -1;
    PTE->PSOR = 1u << col_pin;
    DWT_SPIN_US(50);
    if (PTE->PDIR & (1u << 0)) k = k1;
    if (PTE->PDIR & (1u << 1)) k = k2;
    if (PTE->PDIR & (1u << 2)) k = k3;
    if (PTE->PDIR & (1u << 3)) k = k4;
    PTE->PCOR = 1u << col_pin;
    return k;
}

int KeyScan_fast(void)
{
    int k;
    k = key_col(KEY_COL1, 1, 4, 7, 11);        // COL1 : 1 4 7 *
    if (k >= 0) return k;
    k = key_col(KEY_COL2, 2, 5, 8, 0);         // COL2 : 2 5 8 0
    if (k >= 0) return k;
    k = key_col(KEY_COL3, 3, 6, 9, 12);        // COL3 : 3 6 9 #
    return k;
}

// ---------------- UART : blocking ----------------
void LPUART1_init(void)
{
    PCC->PCCn[PCC_LPUART1_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_LPUART1_INDEX] |= PCC_PCCn_PCS(1);                  // SOSCDIV2 = 8 MHz
    PCC->PCCn[PCC_LPUART1_INDEX] |= PCC_PCCn_CGC_MASK;
    LPUART1->BAUD = 0x0F000034;                                       // 9600 : OSR 16, SBR 52
    LPUART1->CTRL = 0x000C0000;                                       // TE, RE, 8N1
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

static int u32_to_str(uint32_t v, char *buf)       // returns start index in buf[11]
{
    int i = 10;
    buf[i] = '\0';
    do {
        buf[--i] = (char)('0' + v % 10);
        v /= 10;
    } while (v);
    return i;
}

void uart_put_uint(uint32_t v)
{
    char buf[11];
    uart_puts(&buf[u32_to_str(v, buf)]);
}

void uart_put_int(int32_t v)
{
    if (v < 0) { uart_putc('-'); v = -v; }
    uart_put_uint((uint32_t)v);
}

int uart_getc_nonblock(void)
{
    if (LPUART1->STAT & LPUART_STAT_RDRF_MASK) {
        return (int)(LPUART1->DATA & 0xFF);
    }
    return -1;
}

// ---------------- UART : queued, non blocking ----------------
#define UQ_SIZE  1024
static char              uq_buf[UQ_SIZE];
static volatile uint32_t uq_head = 0;      // producer writes here
static volatile uint32_t uq_tail = 0;      // consumer (pump) reads here
static volatile uint32_t uq_drop = 0;

static void uq_putc(char c)
{
    uint32_t next = (uq_head + 1) % UQ_SIZE;
    if (next == uq_tail) { uq_drop++; return; }
    uq_buf[uq_head] = c;
    uq_head = next;
}

void uart_q_puts(const char *s)
{
    while (*s) uq_putc(*s++);
}

void uart_q_put_uint(uint32_t v)
{
    char buf[11];
    uart_q_puts(&buf[u32_to_str(v, buf)]);
}

void uart_q_put_int(int32_t v)
{
    if (v < 0) { uq_putc('-'); v = -v; }
    uart_q_put_uint((uint32_t)v);
}

int uart_q_pump(void)
{
    if (uq_tail == uq_head) return 0;
    if ((LPUART1->STAT & LPUART_STAT_TDRE_MASK) == 0) return 0;
    LPUART1->DATA = (uint8_t)uq_buf[uq_tail];
    uq_tail = (uq_tail + 1) % UQ_SIZE;
    return 1;
}

int uart_q_pending(void)
{
    return (int)((uq_head + UQ_SIZE - uq_tail) % UQ_SIZE);
}

uint32_t uart_q_dropped(void)
{
    return uq_drop;
}

// ---------------- stepper ----------------
static const uint8_t STEP_SEQ[4] = {0x3, 0x6, 0xC, 0x9};             // 2 phase full step
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

// ---------------- ultrasonic ----------------
void Ultrasonic_trigger(void)
{
    PTC->PSOR = 1u << US_TRIG;
    DWT_SPIN_US(10);
    PTC->PCOR = 1u << US_TRIG;
}

uint32_t Ultrasonic_cycles_to_cm(uint32_t cycles)
{
    return cycles / CYCLES_PER_US / 58u;
}

// ---------------- interrupts ----------------
void NVIC_enable_irq(uint32_t irq, uint8_t prio)
{
    S32_NVIC->ICPR[irq / 32] = 1u << (irq % 32);
    S32_NVIC->ISER[irq / 32] = 1u << (irq % 32);
    S32_NVIC->IP[irq]        = (uint8_t)(prio << 4);                  // 4 priority bits, upper nibble
}

void LPIT_ch0_irq_init(uint32_t period_us)
{
    LPIT0->TMR[0].TCTRL = 0;
    LPIT0->TMR[0].TVAL  = period_us * 40u - 1u;
    LPIT0->MSR          = LPIT_MSR_TIF0_MASK;
    LPIT0->MIER        |= LPIT_MIER_TIE0_MASK;
    LPIT0->TMR[0].TCTRL = LPIT_TMR_TCTRL_T_EN_MASK;                    // periodic 32 bit counter
}

void PORTC_irq_init(void)
{
    PORTC->PCR[US_ECHO] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(11);                                         // both edges
    PORTC->PCR[SW1_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PFE_MASK | PORT_PCR_PE(1) | PORT_PCR_PS(1)
                        | PORT_PCR_IRQC(10);                                                            // falling edge
    PORTC->ISFR = 0xFFFFFFFFu;
}
