// main.c : week2 - smart parking barrier, structure (1) SEQUENTIAL LOOP (no interrupts, no RTOS)
//
//  REVISED version (folder week2 revised). differences from the first version :
//    - SENSOR FAULT : "sensor did not answer" (US_NO_ECHO) is told apart from "nothing in range" (US_OUT_OF_RANGE).
//      no answer for FAULT_COUNT measurements in a row (1 s) -> EVB red LED on, "SENSOR FAULT" log.
//      while the fault lasts the barrier is never lowered (fail safe) : OPEN stays open, CLOSING stops (EMERGENCY),
//      INPUT does not time out. first valid answer -> red LED off, "SENSOR OK", normal again.
//    - PTE16 starts HIGH (tp_board.c), so the measurement jumper may stay in with MEASURE 0
//    - MEASURE report : R5 also shows how many measurements got a valid answer
//
//  wiring  : same 29 wires as week1 main_test_all (see the wiring md in this folder)
//            MEASURE = 1 needs one more jumper : PTE16 -> PTC8 (the SW1 line), so the code can "press" SW1 by itself
//            USE_OLED = 1 needs oled.c / oled.h in the project and 2 more wires (J34 SCL -> PTA3, SDA -> PTA2)
//
//  scenario (plan 1-1)
//    IDLE      : FND = free slots. ultrasonic every 100 ms
//    INPUT     : car within 20 cm -> LED0 on, FND "----", "CAR DETECTED". keypad digits -> FND, '*' clear, '#' confirm
//                '#' : PIN == REG_PIN -> OPENING.  wrong -> LED1 + buzzer 3 short beeps, input again.  slots == 0 -> "FULL"
//                car gone for 2 s -> back to IDLE
//    OPENING   : one step per loop when 2 ms passed (target 2 ms / step). at 90 deg -> OPEN
//    OPEN      : wait until the car is gone for 2 s -> slots--, CLOSING
//    CLOSING   : like OPENING, downward. car detected again or SW1 -> EMERGENCY
//    EMERGENCY : motor released at once, buzzer + LED1, "EMERGENCY STOP". SW2 -> release, continue
//    SW1 is also honoured while OPENING. every state change goes out on UART (9600).
//
//  loop order is the one in plan 3-1 : FND -> keypad -> logic -> ultrasonic -> motor -> UART(in logic) -> SW1
//  everything blocks : Seg_out 4 ms, KeyScan 3 ms (every 3rd loop), ultrasonic up to 30 ms, UART 20 chars 21 ms.
//  that is the point of structure (1). the numbers R1..R5 show what it costs.
//
//  MEASURE = 1 : stress measurement (plan 4-1). no car / keypad needed.
//    runs N_RUNS cycles : open MEAS_STEPS -> close. during closing the code pulls PTE16 low (= SW1 pressed),
//    then immediately sends a 20 char UART log (so the "button" lands while UART is busy, as in the scenario).
//    R1 = time from PTE16 low to Stepper_release(), R3 = motor step interval, R4 = loop (FND) period,
//    R5 = ultrasonic measurements per 10 s. result table goes out on UART at the end and FND shows R1 average.
#include "tp_board.h"

#define USE_OLED        0
#define MEASURE         0

#if USE_OLED
#include "oled.h"
#endif

// ---------------- system parameters ----------------
#define REG_PIN         1234        // registered number (4 digits)
#define SLOTS_INIT      12
#define DETECT_CM       20
#define PASS_CLEAR_MS   2000        // car must be gone this long before the barrier closes (and before INPUT gives up)
#define STEP_US         2000        // target step interval
#define US_PERIOD_US    100000      // ultrasonic every 100 ms
#define KEY_EVERY       3           // KeyScan on every 3rd loop (about 20 ms)
#define OPEN_STEPS      STEPS_90DEG
#define FAULT_COUNT     10          // this many US_NO_ECHO in a row (10 x 100 ms = 1 s) -> SENSOR FAULT

#if MEASURE
#define N_RUNS          100
#define MEAS_STEPS      128         // shorter travel : 100 runs take about 3 min
#define FIRE_BASE_US    200000      // emergency fired this long after closing starts (+ run % 10 * 700 us so it lands on different loop phases)
#define AUTO_RELEASE_MS 50
#endif

typedef enum { ST_IDLE, ST_INPUT, ST_OPENING, ST_OPEN, ST_CLOSING, ST_EMERGENCY } state_t;

static const char *STATE_NAME[] = { "IDLE", "INPUT", "OPENING", "OPEN", "CLOSING", "EMERGENCY" };

// ---------------- system variables ----------------
static state_t  state = ST_IDLE;
static state_t  resume_state = ST_CLOSING;    // where EMERGENCY goes back to
static int      slots = SLOTS_INIT;
static int      pin_val = 0, pin_len = 0;
static int      position = 0;                 // 0 = closed, OPEN_STEPS = open
static int      target = 0;
static int      dir = 0;
static uint32_t step_t0 = 0;
static uint32_t dist_cm = 0;
static int      car = 0;                      // 1 = something within DETECT_CM
static uint32_t car_gone_t0 = 0;
static int      car_gone_timing = 0;
static uint32_t us_t0 = 0;
static int      us_fail = 0;                  // US_NO_ECHO in a row
static int      sensor_fault = 0;             // 1 = sensor silent for FAULT_COUNT measurements

#define CAR_OR_FAULT()  (car || sensor_fault)  // "can not prove the lane is empty" -> never lower the barrier

// ---------------- measurement variables ----------------
#if MEASURE
static uint32_t r1_us[N_RUNS];
static int      run = 0;
static int      fired = 0;
static uint32_t t_fire = 0, close_t0 = 0, emg_t0 = 0;
static uint32_t r3_max = 0, r3_min = 0xFFFFFFFFu;        // motor step interval (us)
static uint32_t r4_max = 0, r4_min = 0xFFFFFFFFu;        // loop period (us)
static uint32_t us_count = 0, us_valid = 0;            // measurements / measurements the sensor answered
static uint64_t meas_elapsed_us = 0;                   // summed loop periods (DWT alone wraps every 53 s)
static uint32_t loop_t0 = 0;
static uint32_t r1_avg_ms = 0;
static int      measuring = 0;
#endif

// ---------------- helpers ----------------
static void log_state(const char *msg)
{
    uart_puts("["); uart_puts(STATE_NAME[state]); uart_puts("] "); uart_puts(msg); uart_puts("\r\n");
}

#if USE_OLED
static void oled_dist(void)
{
    if (dist_cm == US_NO_ECHO)            OLED_puts(2, 6, "FAULT");
    else if (dist_cm == US_OUT_OF_RANGE)  OLED_puts(2, 6, "  far");
    else                                  OLED_put_uint(2, 6, dist_cm, 5);
}

static void oled_status(const char *line)
{
    OLED_puts(0, 0, "                     ");
    OLED_puts(0, 0, line);
    OLED_puts(2, 0, "DIST:");
    oled_dist();
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

static void beep(int times, uint32_t ms)      // blocking on purpose (structure 1)
{
    for (int i = 0; i < times; i++) {
        BUZZER_ON();  delay_ms(ms);
        BUZZER_OFF(); delay_ms(ms);
    }
}

static void start_move(int to, state_t s, const char *msg)
{
    target  = to;
    dir     = (to > position) ? +1 : -1;
    step_t0 = dwt_now();
    set_state(s, msg);
}

static void enter_emergency(const char *why)
{
    Stepper_release();                        // motor stops here
#if MEASURE
    if (fired) {
        TOGGLE_HIGH();                        // let go of the "button"
        if (run < N_RUNS) r1_us[run] = dwt_elapsed_us(t_fire);
        emg_t0 = dwt_now();
    }
#endif
    BUZZER_ON();
    LED1_ON();
    resume_state = state;
    set_state(ST_EMERGENCY, why);
    uart_puts("  stopped at pos "); uart_put_int(position); uart_puts(" - press SW2 to release\r\n");
}

static void leave_emergency(void)
{
    BUZZER_OFF();
    LED1_OFF();
    if (resume_state == ST_CLOSING && CAR_OR_FAULT()) {
        set_state(ST_OPEN, sensor_fault ? "RELEASED, sensor fault -> stay open"
                                        : "RELEASED, car still there -> wait");
        car_gone_timing = 0;
    } else if (resume_state == ST_CLOSING) {
        start_move(0, ST_CLOSING, "RELEASED, closing again");
    } else {
        start_move(target, ST_OPENING, "RELEASED, opening again");
    }
}

// ---------------- FND : what to show in each state ----------------
static void fnd_update(void)
{
#if MEASURE
    Seg_out(run);
    return;
#endif
    switch (state) {
    case ST_IDLE:      Seg_out(slots);                                 break;
    case ST_INPUT:     if (pin_len == 0) Seg_out_dash(); else Seg_out(pin_val); break;
    case ST_EMERGENCY: Seg_out_dash();                                 break;
    default:           Seg_out(position);                              break;   // moving / open : show position
    }
}

// ---------------- state machine ----------------
static void logic(int key)
{
    switch (state) {

    case ST_IDLE:
        if (car) {
            pin_val = 0; pin_len = 0;
            car_gone_timing = 0;
            set_state(ST_INPUT, "CAR DETECTED, enter PIN");
        }
        break;

    case ST_INPUT:
        if (!CAR_OR_FAULT()) {
            if (!car_gone_timing) { car_gone_timing = 1; car_gone_t0 = dwt_now(); }
            else if (dwt_elapsed_us(car_gone_t0) >= PASS_CLEAR_MS * 1000u) {
                car_gone_timing = 0;
                set_state(ST_IDLE, "CAR LEFT, back to idle");
                break;
            }
        } else {
            car_gone_timing = 0;
        }
        if (key >= 0 && key <= 9) {
            if (pin_len < 4) { pin_val = pin_val * 10 + key; pin_len++; }
            uart_puts("  key "); uart_put_int(key); uart_puts(" -> "); uart_put_int(pin_val); uart_puts("\r\n");
        } else if (key == 11) {                                   // '*'
            pin_val = 0; pin_len = 0;
            uart_puts("  key * -> clear\r\n");
        } else if (key == 12) {                                   // '#'
            if (slots <= 0) {
                LED1_ON(); log_state("FULL, no free slot"); beep(1, 500); LED1_OFF();
                pin_val = 0; pin_len = 0;
            } else if (pin_len == 4 && pin_val == REG_PIN) {
                log_state("PIN OK");
                start_move(OPEN_STEPS, ST_OPENING, "OPENING barrier");
            } else {
                LED1_ON(); log_state("WRONG PIN, try again"); beep(3, 100); LED1_OFF();
                pin_val = 0; pin_len = 0;
            }
        }
        break;

    case ST_OPEN:
        if (!CAR_OR_FAULT()) {                                    // sensor fault : stays open
            if (!car_gone_timing) { car_gone_timing = 1; car_gone_t0 = dwt_now(); }
            else if (dwt_elapsed_us(car_gone_t0) >= PASS_CLEAR_MS * 1000u) {
                car_gone_timing = 0;
                slots--;
                uart_puts("  CAR PASSED, slots = "); uart_put_int(slots); uart_puts("\r\n");
                start_move(0, ST_CLOSING, "CLOSING barrier");
            }
        } else {
            car_gone_timing = 0;
        }
        break;

    case ST_CLOSING:
#if !MEASURE
        if (car)               enter_emergency("EMERGENCY STOP : car under barrier");
        else if (sensor_fault) enter_emergency("EMERGENCY STOP : sensor fault");
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

// ---------------- motor : one step per loop at most ----------------
static void motor_update(void)
{
    uint32_t dt;

    if (state != ST_OPENING && state != ST_CLOSING) return;
    dt = dwt_elapsed_us(step_t0);
    if (dt < STEP_US) return;

#if MEASURE
    if (measuring && state == ST_CLOSING) {
        if (dt > r3_max) r3_max = dt;
        if (dt < r3_min) r3_min = dt;
    }
#endif
    step_t0 = dwt_now();
    Stepper_step(dir);
    position += dir;

    if (position == target) {
        Stepper_release();
        if (state == ST_OPENING) {
#if MEASURE
            start_move(0, ST_CLOSING, "CLOSING (measure)");
            close_t0 = dwt_now();
            fired = 0;
#else
            car_gone_timing = 0;
            set_state(ST_OPEN, "OPEN, waiting for the car to pass");
#endif
        } else {
#if MEASURE
            run++;
            if (run < N_RUNS) start_move(MEAS_STEPS, ST_OPENING, "OPENING (measure)");
            else              set_state(ST_IDLE, "measurement done");
#else
            set_state(ST_IDLE, "CLOSED, idle");
#endif
        }
    }
}

// ---------------- measurement report ----------------
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
    uint32_t mx = 0, mn = 0xFFFFFFFFu, avg, sd, win_s, per10;

    for (int i = 0; i < N_RUNS; i++) {
        sum += r1_us[i];
        if (r1_us[i] > mx) mx = r1_us[i];
        if (r1_us[i] < mn) mn = r1_us[i];
    }
    avg = (uint32_t)(sum / N_RUNS);
    for (int i = 0; i < N_RUNS; i++) {
        int64_t d = (int64_t)r1_us[i] - (int64_t)avg;
        sq += (uint64_t)(d * d);
    }
    sd = isqrt32(sq / N_RUNS);
    r1_avg_ms = avg / 1000u;
    win_s = (uint32_t)(meas_elapsed_us / 1000000u);
    per10 = (meas_elapsed_us > 0) ? (uint32_t)((uint64_t)us_count * 10000000u / meas_elapsed_us) : 0;

    uart_puts("\r\n=== structure (1) sequential loop : stress result, N = "); uart_put_uint(N_RUNS); uart_puts(" ===\r\n");
    uart_puts("R1 emergency stop latency : avg "); uart_put_uint(avg);
    uart_puts(" us, max "); uart_put_uint(mx); uart_puts(" us, min "); uart_put_uint(mn);
    uart_puts(" us, std "); uart_put_uint(sd); uart_puts(" us\r\n");
    uart_puts("R2 jitter (max - min)     : "); uart_put_uint(mx - mn); uart_puts(" us\r\n");
    uart_puts("R3 motor step interval    : target "); uart_put_uint(STEP_US);
    uart_puts(" us, max "); uart_put_uint(r3_max); uart_puts(" us, min "); uart_put_uint(r3_min);
    uart_puts(" us, max deviation "); uart_put_uint(r3_max - STEP_US); uart_puts(" us\r\n");
    uart_puts("R4 loop (FND) period      : Seg_out itself 4000 us, max "); uart_put_uint(r4_max);
    uart_puts(" us, min "); uart_put_uint(r4_min); uart_puts(" us (FND dark for period - 4000)\r\n");
    uart_puts("R5 ultrasonic             : "); uart_put_uint(us_count); uart_puts(" measurements in ");
    uart_put_uint(win_s); uart_puts(" s = "); uart_put_uint(per10); uart_puts(" per 10 s (target 100), sensor answered ");
    uart_put_uint(us_valid); uart_puts(" / "); uart_put_uint(us_count); uart_puts("\r\n");
    uart_puts("raw R1 (us) :");
    for (int i = 0; i < N_RUNS; i++) { uart_putc(' '); uart_put_uint(r1_us[i]); }
    uart_puts("\r\n=== end. FND shows R1 average in ms ===\r\n");
}
#endif

// ---------------- sensor fault : called after every ultrasonic measurement ----------------
static void sensor_check(void)
{
    if (dist_cm == US_NO_ECHO) {
        if (us_fail < FAULT_COUNT) us_fail++;
        if (us_fail >= FAULT_COUNT && !sensor_fault) {
            sensor_fault = 1;
            FAULT_LED_ON();
            log_state("SENSOR FAULT : no echo for 1 s, barrier will not close");
        }
    } else {
        us_fail = 0;
        if (sensor_fault) {
            sensor_fault = 0;
            FAULT_LED_OFF();
            log_state("SENSOR OK again");
        }
    }
}

// ---------------- main ----------------
int main(void)
{
    int key = -1, prev_key = -1, key_div = 0, key_event;

    board_init();
    uart_puts("\r\n=== SMART PARKING BARRIER - structure (1) sequential loop ===\r\n");
#if USE_OLED
    if (OLED_init() < 0) uart_puts("OLED : no ACK, running without it\r\n");
#endif
#if MEASURE
    TOGGLE_HIGH();                            // PTE16 = SW1 line : high = not pressed
    uart_puts("MEASURE mode : PTE16 -> PTC8 jumper required. ");
    uart_put_uint(N_RUNS); uart_puts(" runs\r\n");
    delay_ms(500);
    measuring = 1;
    loop_t0 = dwt_now();
    us_t0   = dwt_now();
    start_move(MEAS_STEPS, ST_OPENING, "OPENING (measure)");
#else
    set_state(ST_IDLE, "ready, PIN = 1234");
    us_t0 = dwt_now();
#endif

    for (;;) {
#if MEASURE
        // ---- 0. (measure) fire the emergency while closing, then keep UART busy for 20 chars ----
        if (state == ST_CLOSING && !fired && run < N_RUNS &&
            dwt_elapsed_us(close_t0) >= FIRE_BASE_US + (uint32_t)(run % 10) * 700u) {
            fired  = 1;
            t_fire = dwt_now();
            TOGGLE_LOW();                                       // "press" SW1
            uart_puts("STRESS LOG 0123456789\r\n");            // 20 chars + CRLF, about 23 ms at 9600
        }
        {   // R4 : loop period
            uint32_t lp = dwt_elapsed_us(loop_t0);
            loop_t0 = dwt_now();
            meas_elapsed_us += lp;
            if (measuring && lp < 1000000u) { if (lp > r4_max) r4_max = lp; if (lp < r4_min) r4_min = lp; }
        }
        if (state == ST_IDLE) {                                 // all runs done
            measuring = 0;
            report();
            for (;;) { Seg_out((int)r1_avg_ms); }
        }
#endif
        // ---- 1. FND (4 ms, blocking) ----
        fnd_update();

        // ---- 2. keypad every 3rd loop (3 ms, blocking) ----
        key_event = -1;
        if (++key_div >= KEY_EVERY) {
            key_div = 0;
            key = KeyScan();
            if (key >= 0 && prev_key == -1) key_event = key;
            prev_key = key;
        }

        // ---- 3. logic ----
        logic(key_event);

        // ---- 4. ultrasonic every 100 ms (up to 30 ms, blocking) ----
        if (dwt_elapsed_us(us_t0) >= US_PERIOD_US) {
            us_t0   = dwt_now();
            dist_cm = Ultrasonic_measure_cm();
            car     = (dist_cm > 0 && dist_cm <= DETECT_CM);
            if (car) LED0_ON(); else LED0_OFF();
            sensor_check();
#if MEASURE
            us_count++;
            if (dist_cm != US_NO_ECHO) us_valid++;
#endif
#if USE_OLED
            oled_dist();
#endif
        }

        // ---- 5. motor : one step if 2 ms passed ----
        motor_update();

        // ---- 6. emergency button, read once per loop ----
        if ((state == ST_OPENING || state == ST_CLOSING) && SW1_PRESSED()) {
            enter_emergency("EMERGENCY STOP : SW1");
        }
    }
}
