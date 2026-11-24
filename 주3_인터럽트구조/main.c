// main.c : week3 - smart parking barrier, structure (2) INTERRUPT bare-metal (no RTOS)
//
//  same scenario and parameters as week2 main.c. what moved into interrupts (plan 3-2) :
//    LPIT0_Ch0_IRQHandler (1 ms) : FND one digit, motor one step every 2 ms, keypad flag every 20 ms,
//                                   ultrasonic trigger every 100 ms
//    PORTC_IRQHandler            : ECHO rising / falling edge -> distance (DWT),
//                                   SW1 falling edge -> motor released at once, buzzer, emergency flag
//    main loop                   : keypad scan (fast), state machine, OLED, UART one char per pass (never blocks)
//
//  wiring   : same 29 wires as week2. MEASURE = 1 : + PTE16 -> PTC8.  USE_OLED = 1 : + J34 SCL -> PTA3, SDA -> PTA2
//  MEASURE  : same stress run as week2 (100 x open / close, emergency fired while closing + 20 char log),
//             R1 = PTE16 low -> Stepper_release() in the SW1 ISR, R3 = step interval in the ISR,
//             R4 = FND tick interval in the ISR, R5 = triggers per 10 s. report on UART, FND = R1 average (us)
#include "tp_board.h"

#define USE_OLED        0
#define MEASURE         0

#if USE_OLED
#include "oled.h"
#endif

#define REG_PIN         1234
#define SLOTS_INIT      12
#define DETECT_CM       20
#define PASS_CLEAR_MS   2000
#define OPEN_STEPS      STEPS_90DEG

#define PRIO_PORTC      1           // SW1 + ECHO : highest
#define PRIO_LPIT       2

#if MEASURE
#define N_RUNS          100
#define MEAS_STEPS      128
#define FIRE_BASE_US    100000      // closing 128 steps takes 256 ms. fire at 100 ms + run % 10 * 700 us
#define AUTO_RELEASE_MS 50
#endif

typedef enum { ST_IDLE, ST_INPUT, ST_OPENING, ST_OPEN, ST_CLOSING, ST_EMERGENCY } state_t;
static const char *STATE_NAME[] = { "IDLE", "INPUT", "OPENING", "OPEN", "CLOSING", "EMERGENCY" };

// ---------------- shared with the ISRs ----------------
static volatile uint32_t ms_tick = 0;
static volatile int      motor_remaining = 0;    // steps still to do (ISR decrements). 0 = motor idle
static volatile int      motor_dir = 0;
static volatile int      motor_pos = 0;          // 0 = closed
static volatile int      motor_done = 0;         // ISR sets 1 when remaining reaches 0
static volatile int      keypad_flag = 0;
static volatile uint32_t dist_cm = 0;
static volatile int      new_dist = 0;
static volatile int      echo_armed = 0;         // trigger sent, echo not yet received
static volatile uint32_t t_rise = 0;
static volatile int      emergency_flag = 0;     // SW1 ISR stopped the motor
static volatile uint32_t t_stop = 0;             // DWT right after Stepper_release() in the ISR

#if MEASURE
static volatile int      measuring = 0;
static volatile uint32_t last_step_t = 0, r3_max = 0, r3_min = 0xFFFFFFFFu;
static volatile uint32_t last_fnd_t  = 0, r4_max = 0, r4_min = 0xFFFFFFFFu;
static volatile uint32_t us_count = 0;
#endif

// ---------------- main side ----------------
static state_t  state = ST_IDLE;
static state_t  resume_state = ST_CLOSING;
static int      slots = SLOTS_INIT;
static int      pin_val = 0, pin_len = 0;
static int      target = 0;
static int      car = 0;
static uint32_t car_gone_t0 = 0;
static int      car_gone_timing = 0;
static uint32_t beep_t0 = 0;
static int      beep_left = 0, beep_on = 0;
static uint32_t beep_ms = 100;

#if MEASURE
static uint32_t r1_us[N_RUNS];
static int      run = 0, fired = 0;
static uint32_t t_fire = 0, close_t0 = 0, emg_t0 = 0, meas_start_ms = 0;
static uint32_t r1_avg = 0;
#endif

// ================= interrupt handlers =================
void LPIT0_Ch0_IRQHandler(void)
{
    uint32_t now = dwt_now();

    LPIT_CH0_CLEAR_FLAG();
    ms_tick++;

    // FND : one digit per ms
#if MEASURE
    if (measuring && last_fnd_t) {
        uint32_t d = (now - last_fnd_t) / CYCLES_PER_US;
        if (d > r4_max) r4_max = d;
        if (d < r4_min) r4_min = d;
    }
    last_fnd_t = now;
#endif
    FND_tick();

    // motor : one step every 2 ms while remaining > 0
    if ((ms_tick & 1u) == 0 && motor_remaining > 0) {
#if MEASURE
        if (measuring && last_step_t && motor_dir < 0) {
            uint32_t d = (now - last_step_t) / CYCLES_PER_US;
            if (d > r3_max) r3_max = d;
            if (d < r3_min) r3_min = d;
        }
        last_step_t = now;
#endif
        Stepper_step(motor_dir);
        motor_pos += motor_dir;
        if (--motor_remaining == 0) {
            Stepper_release();
            motor_done = 1;
#if MEASURE
            last_step_t = 0;
#endif
        }
    }

    if (ms_tick % 20u == 0) keypad_flag = 1;

    if (ms_tick % 100u == 0) {                       // ultrasonic : trigger every 100 ms
        if (echo_armed) { dist_cm = 0; new_dist = 1; }   // previous echo never came back
        Ultrasonic_trigger();
        echo_armed = 1;
#if MEASURE
        if (measuring) us_count++;
#endif
    }
}

void PORTC_IRQHandler(void)
{
    uint32_t now = dwt_now();

    if (PORTC_ISF(US_ECHO)) {
        PORTC_CLEAR_ISF(US_ECHO);
        if (PTC->PDIR & (1u << US_ECHO)) {
            t_rise = now;                                       // rising edge
        } else if (echo_armed) {
            dist_cm    = Ultrasonic_cycles_to_cm(now - t_rise); // falling edge
            new_dist   = 1;
            echo_armed = 0;
        }
    }
    if (PORTC_ISF(SW1_PIN)) {
        PORTC_CLEAR_ISF(SW1_PIN);
        if (motor_remaining > 0) {                              // emergency only while moving
            Stepper_release();                                  // motor stops HERE
            t_stop = dwt_now();
            motor_remaining = 0;
            BUZZER_ON();
            LED1_ON();
            emergency_flag = 1;
        }
    }
}

// ================= helpers (main context) =================
static void log_state(const char *msg)
{
    uart_q_puts("["); uart_q_puts(STATE_NAME[state]); uart_q_puts("] "); uart_q_puts(msg); uart_q_puts("\r\n");
}

#if USE_OLED
static void oled_status(const char *line)
{
    OLED_puts(0, 0, "                     ");
    OLED_puts(0, 0, line);
    OLED_puts(2, 0, "DIST:");
    if (dist_cm == 0) OLED_puts(2, 6, "  ---");
    else              OLED_put_uint(2, 6, dist_cm, 5);
    OLED_puts(2, 12, "cm");
    OLED_puts(3, 0, "SLOT:");
    OLED_put_uint(3, 6, (uint32_t)slots, 5);
}
#else
#define oled_status(s)   ((void)0)
#endif

static void set_state(state_t s, const char *msg)
{
    state = s;
    log_state(msg);
    oled_status(msg);
}

static void beep_start(int times, uint32_t ms)      // non blocking beep, driven by beep_update()
{
    beep_left = times; beep_ms = ms;
    beep_on = 1; BUZZER_ON(); LED1_ON();
    beep_t0 = dwt_now();
}

static void beep_update(void)
{
    if (beep_left == 0) return;
    if (dwt_elapsed_us(beep_t0) < beep_ms * 1000u) return;
    beep_t0 = dwt_now();
    if (beep_on) { BUZZER_OFF(); beep_on = 0; if (--beep_left == 0) LED1_OFF(); }
    else         { BUZZER_ON();  beep_on = 1; }
}

static void motor_start(int to)
{
    target = to;
    if (to == motor_pos) { motor_done = 1; return; }
    motor_dir = (to > motor_pos) ? +1 : -1;
#if MEASURE
    last_step_t = 0;
#endif
    motor_remaining = (to > motor_pos) ? (to - motor_pos) : (motor_pos - to);   // ISR starts stepping on the next 2 ms tick
}

static void motor_stop(void)
{
    motor_remaining = 0;
    Stepper_release();
}

static void enter_emergency(const char *why)
{
    BUZZER_ON();
    LED1_ON();
    resume_state = state;
    set_state(ST_EMERGENCY, why);
    uart_q_puts("  stopped at pos "); uart_q_put_int(motor_pos); uart_q_puts(" - press SW2 to release\r\n");
}

static void leave_emergency(void)
{
    BUZZER_OFF();
    LED1_OFF();
    if (resume_state == ST_CLOSING && car) {
        set_state(ST_OPEN, "RELEASED, car still there -> wait");
        car_gone_timing = 0;
    } else if (resume_state == ST_CLOSING) {
        set_state(ST_CLOSING, "RELEASED, closing again");
        motor_start(0);
    } else {
        set_state(ST_OPENING, "RELEASED, opening again");
        motor_start(target);
    }
}

static void fnd_update(void)
{
#if MEASURE
    FND_set(run);
    return;
#endif
    switch (state) {
    case ST_IDLE:      FND_set(slots);                              break;
    case ST_INPUT:     FND_set(pin_len == 0 ? -1 : pin_val);        break;
    case ST_EMERGENCY: FND_set(-1);                                 break;
    default:           FND_set(motor_pos);                          break;
    }
}

// ================= state machine (main context) =================
static void logic(int key)
{
    switch (state) {

    case ST_IDLE:
        if (car) {
            pin_val = 0; pin_len = 0; car_gone_timing = 0;
            set_state(ST_INPUT, "CAR DETECTED, enter PIN");
        }
        break;

    case ST_INPUT:
        if (!car) {
            if (!car_gone_timing) { car_gone_timing = 1; car_gone_t0 = dwt_now(); }
            else if (dwt_elapsed_us(car_gone_t0) >= PASS_CLEAR_MS * 1000u) {
                car_gone_timing = 0;
                set_state(ST_IDLE, "CAR LEFT, back to idle");
                break;
            }
        } else {
            car_gone_timing = 0;
        }
        if (beep_left) break;                                     // ignore keys while beeping
        if (key >= 0 && key <= 9) {
            if (pin_len < 4) { pin_val = pin_val * 10 + key; pin_len++; }
            uart_q_puts("  key "); uart_q_put_int(key); uart_q_puts(" -> "); uart_q_put_int(pin_val); uart_q_puts("\r\n");
        } else if (key == 11) {
            pin_val = 0; pin_len = 0;
            uart_q_puts("  key * -> clear\r\n");
        } else if (key == 12) {
            if (slots <= 0) {
                log_state("FULL, no free slot"); beep_start(1, 500);
                pin_val = 0; pin_len = 0;
            } else if (pin_len == 4 && pin_val == REG_PIN) {
                log_state("PIN OK");
                set_state(ST_OPENING, "OPENING barrier");
                motor_start(OPEN_STEPS);
            } else {
                log_state("WRONG PIN, try again"); beep_start(3, 100);
                pin_val = 0; pin_len = 0;
            }
        }
        break;

    case ST_OPEN:
        if (!car) {
            if (!car_gone_timing) { car_gone_timing = 1; car_gone_t0 = dwt_now(); }
            else if (dwt_elapsed_us(car_gone_t0) >= PASS_CLEAR_MS * 1000u) {
                car_gone_timing = 0;
                slots--;
                uart_q_puts("  CAR PASSED, slots = "); uart_q_put_int(slots); uart_q_puts("\r\n");
                set_state(ST_CLOSING, "CLOSING barrier");
                motor_start(0);
            }
        } else {
            car_gone_timing = 0;
        }
        break;

    case ST_CLOSING:
#if !MEASURE
        if (car && motor_remaining > 0) {
            motor_stop();
            enter_emergency("EMERGENCY STOP : car under barrier");
        }
#endif
        break;

    case ST_EMERGENCY:
#if MEASURE
        if (dwt_elapsed_us(emg_t0) >= AUTO_RELEASE_MS * 1000u) leave_emergency();
#else
        if (SW2_PRESSED() && !SW1_PRESSED()) leave_emergency();
#endif
        break;

    default:
        break;
    }
}

static void motor_finished(void)
{
    if (state == ST_OPENING) {
#if MEASURE
        set_state(ST_CLOSING, "CLOSING (measure)");
        close_t0 = dwt_now();
        fired = 0;
        motor_start(0);
#else
        car_gone_timing = 0;
        set_state(ST_OPEN, "OPEN, waiting for the car to pass");
#endif
    } else if (state == ST_CLOSING) {
#if MEASURE
        run++;
        if (run < N_RUNS) { set_state(ST_OPENING, "OPENING (measure)"); motor_start(MEAS_STEPS); }
        else              { set_state(ST_IDLE, "measurement done"); }
#else
        set_state(ST_IDLE, "CLOSED, idle");
#endif
    }
}

// ================= measurement report =================
#if MEASURE
static uint32_t isqrt32(uint64_t v)
{
    uint64_t r = 0, bit = (uint64_t)1 << 62;
    while (bit > v) bit >>= 2;
    while (bit) {
        if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
        else              { r >>= 1; }
        bit >>= 2;
    }
    return (uint32_t)r;
}

static void report(void)
{
    uint64_t sum = 0, sq = 0;
    uint32_t mx = 0, mn = 0xFFFFFFFFu, sd, win_ms, per10;

    for (int i = 0; i < N_RUNS; i++) {
        sum += r1_us[i];
        if (r1_us[i] > mx) mx = r1_us[i];
        if (r1_us[i] < mn) mn = r1_us[i];
    }
    r1_avg = (uint32_t)(sum / N_RUNS);
    for (int i = 0; i < N_RUNS; i++) {
        int64_t d = (int64_t)r1_us[i] - (int64_t)r1_avg;
        sq += (uint64_t)(d * d);
    }
    sd     = isqrt32(sq / N_RUNS);
    win_ms = ms_tick - meas_start_ms;
    per10  = (win_ms > 0) ? (uint32_t)((uint64_t)us_count * 10000u / win_ms) : 0;

    while (uart_q_pump() || uart_q_pending()) {}                 // flush the queued log first

    uart_puts("\r\n=== structure (2) interrupt : stress result, N = "); uart_put_uint(N_RUNS); uart_puts(" ===\r\n");
    uart_puts("R1 emergency stop latency : avg "); uart_put_uint(r1_avg);
    uart_puts(" us, max "); uart_put_uint(mx); uart_puts(" us, min "); uart_put_uint(mn);
    uart_puts(" us, std "); uart_put_uint(sd); uart_puts(" us\r\n");
    uart_puts("R2 jitter (max - min)     : "); uart_put_uint(mx - mn); uart_puts(" us\r\n");
    uart_puts("R3 motor step interval    : target 2000 us, max "); uart_put_uint(r3_max);
    uart_puts(" us, min "); uart_put_uint(r3_min); uart_puts(" us, max deviation ");
    uart_put_uint(r3_max > 2000u ? r3_max - 2000u : 2000u - r3_min); uart_puts(" us\r\n");
    uart_puts("R4 FND digit period       : target 1000 us, max "); uart_put_uint(r4_max);
    uart_puts(" us, min "); uart_put_uint(r4_min); uart_puts(" us\r\n");
    uart_puts("R5 ultrasonic             : "); uart_put_uint(us_count); uart_puts(" triggers in ");
    uart_put_uint(win_ms / 1000u); uart_puts(" s = "); uart_put_uint(per10); uart_puts(" per 10 s (target 100)\r\n");
    uart_puts("UART chars dropped        : "); uart_put_uint(uart_q_dropped()); uart_puts("\r\n");
    uart_puts("raw R1 (us) :");
    for (int i = 0; i < N_RUNS; i++) { uart_putc(' '); uart_put_uint(r1_us[i]); }
    uart_puts("\r\n=== end. FND shows R1 average in us ===\r\n");
}
#endif

// ================= main =================
int main(void)
{
    int key = -1, prev_key = -1, key_event;

    board_init();
    uart_puts("\r\n=== SMART PARKING BARRIER - structure (2) interrupt ===\r\n");
#if USE_OLED
    if (OLED_init() < 0) uart_puts("OLED : no ACK, running without it\r\n");
#endif

    PORTC_irq_init();
    NVIC_enable_irq(PORTC_IRQ, PRIO_PORTC);
    LPIT_ch0_irq_init(1000);
    NVIC_enable_irq(LPIT0_CH0_IRQ, PRIO_LPIT);

#if MEASURE
    TOGGLE_HIGH();
    uart_puts("MEASURE mode : PTE16 -> PTC8 jumper required. "); uart_put_uint(N_RUNS); uart_puts(" runs\r\n");
    delay_ms(500);
    measuring = 1;
    meas_start_ms = ms_tick;
    set_state(ST_OPENING, "OPENING (measure)");
    motor_start(MEAS_STEPS);
#else
    set_state(ST_IDLE, "ready, PIN = 1234");
#endif

    for (;;) {
        // keypad : every 20 ms (flag from the 1 ms ISR), about 150 us
        key_event = -1;
        if (keypad_flag) {
            keypad_flag = 0;
            key = KeyScan_fast();
            if (key >= 0 && prev_key == -1) key_event = key;
            prev_key = key;
        }

        // distance : from the ECHO ISR
        if (new_dist) {
            new_dist = 0;
            car = (dist_cm > 0 && dist_cm <= DETECT_CM);
            if (car) LED0_ON(); else LED0_OFF();
#if USE_OLED
            if (dist_cm == 0) OLED_puts(2, 6, "  ---");
            else              OLED_put_uint(2, 6, dist_cm, 5);
#endif
        }

        // motor reached its target (from the 1 ms ISR)
        if (motor_done) {
            motor_done = 0;
            motor_finished();
        }

        // SW1 ISR already stopped the motor. here only the state / log
        if (emergency_flag) {
            emergency_flag = 0;
#if MEASURE
            if (fired) {
                TOGGLE_HIGH();
                if (run < N_RUNS) r1_us[run] = (t_stop - t_fire) / CYCLES_PER_US;
                emg_t0 = dwt_now();
            }
#endif
            if (state == ST_OPENING || state == ST_CLOSING) enter_emergency("EMERGENCY STOP : SW1");
        }

#if MEASURE
        if (state == ST_CLOSING && !fired && run < N_RUNS &&
            dwt_elapsed_us(close_t0) >= FIRE_BASE_US + (uint32_t)(run % 10) * 700u) {
            fired  = 1;
            t_fire = dwt_now();
            TOGGLE_LOW();                                       // "press" SW1 -> PORTC ISR
            uart_q_puts("STRESS LOG 0123456789\r\n");          // queued, goes out one char per loop pass
        }
        if (state == ST_IDLE) {
            measuring = 0;
            report();
            FND_set((int)(r1_avg > 9999u ? 9999u : r1_avg));
            for (;;) {}                                         // FND keeps running from the ISR
        }
#endif

        logic(key_event);
        beep_update();
        fnd_update();
        uart_q_pump();                                          // one char per pass, never waits
    }
}
