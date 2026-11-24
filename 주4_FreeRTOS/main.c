// main.c : week4 - smart parking barrier, structure (3) FreeRTOS
//
//  same scenario and parameters as week2 / week3. tasks (plan 3-3) :
//    T_Safety  prio 5  woken by the SW1 ISR (semaphore)      : Stepper_release(), buzzer, EMERGENCY event
//    T_FND     prio 4  every 1 ms  (vTaskDelayUntil)          : one digit            (FND_MODE 1 : LPIT ISR instead)
//    T_Motor   prio 3  every 2 ms                             : one step while steps remain
//    T_Sensor  prio 2  every 100 ms                           : TRIG, wait ECHO semaphore (<= 35 ms), DIST event
//    T_Keypad  prio 2  every 20 ms                            : KeyScan_fast, KEY event on press
//    T_Logic   prio 2  event queue                            : state machine, motor commands, log messages
//    T_Log     prio 1  log queue                              : UART, blocking only itself
//    T_Dummy   prio 1  x DUMMY_TASKS                          : compute loop, for M4
//  ISRs : PORTC (ECHO edges, SW1) only give semaphores. no delay_ms after the scheduler starts.
//
//  wiring   : same 29 wires as week2. MEASURE 1 : + PTE16 -> PTC8.  USE_OLED 1 : + J34 SCL -> PTA3, SDA -> PTA2
//  MEASURE 0 : system.  1 : stress run, R1..R5 + M2, M3 (needs M1_CYCLES from mode 2).  2 : M1 context switch benchmark
//
//  project : main.c, tp_board.c/.h, clocks_and_modes.c/.h, FreeRTOSConfig.h
//          + FreeRTOS kernel : tasks.c, queue.c, list.c, portable/GCC/ARM_CM4F/port.c, portable/MemMang/heap_4.c
//            include paths : FreeRTOS/include, FreeRTOS/portable/GCC/ARM_CM4F
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "tp_board.h"

#define USE_OLED        0
#define MEASURE         0
#define DUMMY_TASKS     0           // M4 : 0 / 3 / 6
#define FND_MODE        0           // 0 : T_FND task (tick must be 1 ms), 1 : FND in LPIT 1 ms ISR (use with 10 ms tick)
#define M1_CYCLES       0           // result of MEASURE 2, used for M3 in the MEASURE 1 report

#if USE_OLED
#include "oled.h"
#endif

#if (TP_TICK_HZ < 1000) && (FND_MODE == 0)
#error "tick slower than 1 ms : set FND_MODE 1 (FND in the LPIT interrupt)"
#endif

#define REG_PIN         1234
#define SLOTS_INIT      12
#define DETECT_CM       20
#define PASS_CLEAR_MS   2000
#define OPEN_STEPS      STEPS_90DEG

// task priorities : condition variable (plan 4-4). e.g. PRIO_FND 1 or PRIO_MOTOR 1 to see what breaks
#define PRIO_SAFETY     5
#define PRIO_FND        4
#define PRIO_MOTOR      3
#define PRIO_SENSOR     2
#define PRIO_KEYPAD     2
#define PRIO_LOGIC      2
#define PRIO_LOG        1
#define PRIO_DUMMY      1

#define PRIO_PORTC_IRQ  6           // >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5) : the ISR calls FromISR APIs
#define PRIO_LPIT_IRQ   6

#if MEASURE
#define N_RUNS          100
#define MEAS_STEPS      128
#define FIRE_BASE_MS    100
#define AUTO_RELEASE_MS 50
#define M1_ITER         10000
#endif

// ---------------- events / messages ----------------
enum { EV_KEY, EV_DIST, EV_MOTOR_DONE, EV_EMERGENCY };
typedef struct { uint8_t type; int32_t val; } event_t;

#define LOG_LEN         48
typedef struct { char s[LOG_LEN]; } logmsg_t;
#define NO_NUM          ((int32_t)0x80000000)

typedef enum { ST_IDLE, ST_INPUT, ST_OPENING, ST_OPEN, ST_CLOSING, ST_EMERGENCY } state_t;
static const char *STATE_NAME[] = { "IDLE", "INPUT", "OPENING", "OPEN", "CLOSING", "EMERGENCY" };

// ---------------- RTOS objects ----------------
static SemaphoreHandle_t sem_safety, sem_echo;
static QueueHandle_t     q_event, q_log;

// ---------------- shared variables ----------------
volatile uint32_t        ctx_switch_count = 0;        // traceTASK_SWITCHED_IN (FreeRTOSConfig.h)
static volatile uint32_t tick_cycles = 0, tick_count = 0;   // SysTick handler cost (M2)
static volatile int      motor_remaining = 0, motor_dir = 0, motor_pos = 0;
static volatile uint32_t echo_cycles = 0, t_rise = 0;
static volatile uint32_t t_stop = 0;
static volatile uint32_t dist_cm = 0;
static volatile uint32_t log_dropped = 0;

#if MEASURE
static volatile int      measuring = 0;
static volatile uint32_t r3_max = 0, r3_min = 0xFFFFFFFFu;
static volatile uint32_t r4_max = 0, r4_min = 0xFFFFFFFFu;
static volatile uint32_t us_count = 0;
static uint32_t r1_us[N_RUNS];
static int      run = 0, fired = 0;
static uint32_t t_fire = 0, r1_avg = 0;
static TickType_t close_tick = 0, emg_tick = 0, meas_start_tick = 0;
#endif

// ================= interrupt handlers =================
extern void xPortSysTickHandler(void);

void SysTick_Handler(void)                             // wraps the kernel tick to measure M2
{
    uint32_t t0 = DWT_CYCCNT;
    xPortSysTickHandler();
    tick_cycles += DWT_CYCCNT - t0;
    tick_count++;
}

void PORTC_IRQHandler(void)
{
    uint32_t now = dwt_now();
    BaseType_t woken = pdFALSE;

    if (PORTC_ISF(US_ECHO)) {
        PORTC_CLEAR_ISF(US_ECHO);
        if (PTC->PDIR & (1u << US_ECHO)) {
            t_rise = now;
        } else {
            echo_cycles = now - t_rise;
            xSemaphoreGiveFromISR(sem_echo, &woken);
        }
    }
    if (PORTC_ISF(SW1_PIN)) {
        PORTC_CLEAR_ISF(SW1_PIN);
        if (motor_remaining > 0) {
            xSemaphoreGiveFromISR(sem_safety, &woken);         // T_Safety stops the motor
        }
    }
    portYIELD_FROM_ISR(woken);
}

#if FND_MODE
void LPIT0_Ch0_IRQHandler(void)
{
    LPIT_CH0_CLEAR_FLAG();
    FND_tick();
}
#endif

// ================= log helpers (any task) =================
static void str_cat(char *dst, const char *src, int max)
{
    int i = 0;
    while (dst[i]) i++;
    while (*src && i < max - 1) dst[i++] = *src++;
    dst[i] = 0;
}

static void num_cat(char *dst, int32_t v, int max)
{
    char buf[12];
    int  i = 11;
    uint32_t u = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
    buf[i] = 0;
    do { buf[--i] = (char)('0' + u % 10); u /= 10; } while (u);
    if (v < 0) buf[--i] = '-';
    str_cat(dst, &buf[i], max);
}

static void logmsg(const char *a, int32_t num, const char *b)   // "a" [num] "b" CRLF -> log queue (never blocks)
{
    logmsg_t m;
    m.s[0] = 0;
    str_cat(m.s, a, LOG_LEN);
    if (num != NO_NUM) num_cat(m.s, num, LOG_LEN);
    if (b) str_cat(m.s, b, LOG_LEN);
    str_cat(m.s, "\r\n", LOG_LEN);
    if (xQueueSend(q_log, &m, 0) != pdPASS) log_dropped++;
}

static void post_event(uint8_t type, int32_t val)
{
    event_t ev = { type, val };
    xQueueSend(q_event, &ev, 0);
}

// ================= T_Safety =================
static void T_Safety(void *p)
{
    (void)p;
    for (;;) {
        xSemaphoreTake(sem_safety, portMAX_DELAY);
        taskENTER_CRITICAL();
        if (motor_remaining > 0) {
            Stepper_release();                                 // motor stops HERE
            t_stop = dwt_now();
            motor_remaining = 0;
            taskEXIT_CRITICAL();
            BUZZER_ON();
            LED1_ON();
            post_event(EV_EMERGENCY, 0);
        } else {
            taskEXIT_CRITICAL();
        }
    }
}

// ================= T_FND =================
#if !FND_MODE
static void T_FND(void *p)
{
    TickType_t last = xTaskGetTickCount();
#if MEASURE
    uint32_t prev = 0;
#endif
    (void)p;
    for (;;) {
        vTaskDelayUntil(&last, 1);
#if MEASURE
        {
            uint32_t now = dwt_now();
            if (measuring && prev) {
                uint32_t d = (now - prev) / CYCLES_PER_US;
                if (d > r4_max) r4_max = d;
                if (d < r4_min) r4_min = d;
            }
            prev = now;
        }
#endif
        FND_tick();
    }
}
#endif

// ================= T_Motor =================
static void T_Motor(void *p)
{
    TickType_t last = xTaskGetTickCount();
#if MEASURE
    uint32_t prev = 0;
#endif
    int done;
    (void)p;
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(2));
        done = 0;
        taskENTER_CRITICAL();
        if (motor_remaining > 0) {
#if MEASURE
            {
                uint32_t now = dwt_now();
                if (measuring && prev && motor_dir < 0) {
                    uint32_t d = (now - prev) / CYCLES_PER_US;
                    if (d > r3_max) r3_max = d;
                    if (d < r3_min) r3_min = d;
                }
                prev = now;
            }
#endif
            Stepper_step(motor_dir);
            motor_pos += motor_dir;
            if (--motor_remaining == 0) { Stepper_release(); done = 1; }
        }
#if MEASURE
        else prev = 0;
#endif
        taskEXIT_CRITICAL();
        if (done) post_event(EV_MOTOR_DONE, motor_pos);
    }
}

// ================= T_Sensor =================
static void T_Sensor(void *p)
{
    TickType_t last = xTaskGetTickCount();
    (void)p;
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
        xSemaphoreTake(sem_echo, 0);                           // drop a stale echo
        Ultrasonic_trigger();
        if (xSemaphoreTake(sem_echo, pdMS_TO_TICKS(35)) == pdTRUE) {
            dist_cm = Ultrasonic_cycles_to_cm(echo_cycles);
        } else {
            dist_cm = 0;
        }
#if MEASURE
        if (measuring) us_count++;
#endif
        post_event(EV_DIST, (int32_t)dist_cm);
    }
}

// ================= T_Keypad =================
static void T_Keypad(void *p)
{
    TickType_t last = xTaskGetTickCount();
    int key, prev = -1;
    (void)p;
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
        key = KeyScan_fast();
        if (key >= 0 && prev == -1) post_event(EV_KEY, key);
        prev = key;
    }
}

// ================= T_Log =================
static void T_Log(void *p)
{
    logmsg_t m;
    (void)p;
    for (;;) {
        xQueueReceive(q_log, &m, portMAX_DELAY);
        uart_puts(m.s);                                        // blocking, but only this lowest task waits
    }
}

// ================= T_Dummy (M4) =================
#if DUMMY_TASKS > 0
static volatile uint32_t dummy_sink = 0;
static void T_Dummy(void *p)
{
    uint32_t x = (uint32_t)(uintptr_t)p;
    for (;;) {
        x = x * 1664525u + 1013904223u;                        // keep the CPU busy
        dummy_sink = x;
    }
}
#endif

// ================= T_Logic : state machine =================
static state_t  state = ST_IDLE, resume_state = ST_CLOSING;
static int      slots = SLOTS_INIT, pin_val = 0, pin_len = 0, target = 0, car = 0;
static TickType_t car_gone_tick = 0; static int car_gone_timing = 0;
static TickType_t beep_tick = 0; static int beep_left = 0, beep_on = 0; static uint32_t beep_ms = 100;

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
    char head[16];
    state = s;
    head[0] = '['; head[1] = 0;
    str_cat(head, STATE_NAME[s], 16);
    str_cat(head, "] ", 16);
    {
        logmsg_t m;
        m.s[0] = 0;
        str_cat(m.s, head, LOG_LEN);
        str_cat(m.s, msg, LOG_LEN);
        str_cat(m.s, "\r\n", LOG_LEN);
        if (xQueueSend(q_log, &m, 0) != pdPASS) log_dropped++;
    }
    oled_status(msg);
}

static void fnd_update(void)
{
#if MEASURE
    FND_set(run);
    return;
#endif
    switch (state) {
    case ST_IDLE:      FND_set(slots);                          break;
    case ST_INPUT:     FND_set(pin_len == 0 ? -1 : pin_val);    break;
    case ST_EMERGENCY: FND_set(-1);                             break;
    default:           FND_set(motor_pos);                      break;
    }
}

static void beep_start(int times, uint32_t ms)
{
    beep_left = times; beep_ms = ms; beep_on = 1;
    BUZZER_ON(); LED1_ON();
    beep_tick = xTaskGetTickCount();
}

static void beep_update(void)
{
    if (beep_left == 0) return;
    if ((xTaskGetTickCount() - beep_tick) < pdMS_TO_TICKS(beep_ms)) return;
    beep_tick = xTaskGetTickCount();
    if (beep_on) { BUZZER_OFF(); beep_on = 0; if (--beep_left == 0) LED1_OFF(); }
    else         { BUZZER_ON();  beep_on = 1; }
}

static void motor_start(int to)
{
    target = to;
    if (to == motor_pos) { post_event(EV_MOTOR_DONE, motor_pos); return; }
    taskENTER_CRITICAL();
    motor_dir       = (to > motor_pos) ? +1 : -1;
    motor_remaining = (to > motor_pos) ? (to - motor_pos) : (motor_pos - to);
    taskEXIT_CRITICAL();
}

static void motor_stop(void)
{
    taskENTER_CRITICAL();
    motor_remaining = 0;
    Stepper_release();
    taskEXIT_CRITICAL();
}

static void enter_emergency(const char *why)
{
    BUZZER_ON(); LED1_ON();
    resume_state = state;
    set_state(ST_EMERGENCY, why);
    logmsg("  stopped at pos ", motor_pos, " - press SW2 to release");
}

static void leave_emergency(void)
{
    BUZZER_OFF(); LED1_OFF();
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

static int car_gone_for(uint32_t ms)              // 1 when the car has been absent for ms
{
    if (car) { car_gone_timing = 0; return 0; }
    if (!car_gone_timing) { car_gone_timing = 1; car_gone_tick = xTaskGetTickCount(); return 0; }
    if ((xTaskGetTickCount() - car_gone_tick) >= pdMS_TO_TICKS(ms)) { car_gone_timing = 0; return 1; }
    return 0;
}

static void motor_finished(void)
{
    if (state == ST_OPENING) {
#if MEASURE
        set_state(ST_CLOSING, "CLOSING (measure)");
        close_tick = xTaskGetTickCount();
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

static void handle_key(int key)
{
    if (state != ST_INPUT || beep_left) return;
    if (key >= 0 && key <= 9) {
        if (pin_len < 4) { pin_val = pin_val * 10 + key; pin_len++; }
        logmsg("  key -> ", pin_val, 0);
    } else if (key == 11) {
        pin_val = 0; pin_len = 0;
        logmsg("  key * -> clear", NO_NUM, 0);
    } else if (key == 12) {
        if (slots <= 0) {
            logmsg("  FULL, no free slot", NO_NUM, 0); beep_start(1, 500);
            pin_val = 0; pin_len = 0;
        } else if (pin_len == 4 && pin_val == REG_PIN) {
            logmsg("  PIN OK", NO_NUM, 0);
            set_state(ST_OPENING, "OPENING barrier");
            motor_start(OPEN_STEPS);
        } else {
            logmsg("  WRONG PIN, try again", NO_NUM, 0); beep_start(3, 100);
            pin_val = 0; pin_len = 0;
        }
    }
}

static void periodic_logic(void)                      // every 10 ms or on each event
{
    switch (state) {
    case ST_IDLE:
        if (car) { pin_val = 0; pin_len = 0; car_gone_timing = 0; set_state(ST_INPUT, "CAR DETECTED, enter PIN"); }
        break;
    case ST_INPUT:
        if (car_gone_for(PASS_CLEAR_MS)) set_state(ST_IDLE, "CAR LEFT, back to idle");
        break;
    case ST_OPEN:
        if (car_gone_for(PASS_CLEAR_MS)) {
            slots--;
            logmsg("  CAR PASSED, slots = ", slots, 0);
            set_state(ST_CLOSING, "CLOSING barrier");
            motor_start(0);
        }
        break;
    case ST_CLOSING:
#if !MEASURE
        if (car && motor_remaining > 0) { motor_stop(); enter_emergency("EMERGENCY STOP : car under barrier"); }
#endif
        break;
    case ST_EMERGENCY:
#if MEASURE
        if ((xTaskGetTickCount() - emg_tick) >= pdMS_TO_TICKS(AUTO_RELEASE_MS)) leave_emergency();
#else
        if (SW2_PRESSED() && !SW1_PRESSED()) leave_emergency();
#endif
        break;
    default:
        break;
    }
    beep_update();
    fnd_update();
}

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
    uint64_t sum = 0, sq = 0, elapsed_cycles, overhead_cycles;
    uint32_t mx = 0, mn = 0xFFFFFFFFu, sd, win_ms, per10, m2, m3_x100;

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
    win_ms = (uint32_t)((xTaskGetTickCount() - meas_start_tick) * 1000u / TP_TICK_HZ);
    per10  = (win_ms > 0) ? (uint32_t)((uint64_t)us_count * 10000u / win_ms) : 0;
    m2     = tick_count ? tick_cycles / tick_count : 0;
    elapsed_cycles  = (uint64_t)win_ms * (configCPU_CLOCK_HZ / 1000u);
    overhead_cycles = (uint64_t)tick_cycles + (uint64_t)ctx_switch_count * (uint64_t)M1_CYCLES;
    m3_x100 = elapsed_cycles ? (uint32_t)(overhead_cycles * 10000u / elapsed_cycles) : 0;

    while (uxQueueMessagesWaiting(q_log)) vTaskDelay(pdMS_TO_TICKS(10));   // let T_Log drain
    vTaskDelay(pdMS_TO_TICKS(100));

    uart_puts("\r\n=== structure (3) FreeRTOS : stress result, N = "); uart_put_uint(N_RUNS);
    uart_puts(", tick "); uart_put_uint(1000u / TP_TICK_HZ); uart_puts(" ms, preemption ");
    uart_put_uint(configUSE_PREEMPTION); uart_puts(", dummy tasks "); uart_put_uint(DUMMY_TASKS); uart_puts(" ===\r\n");
    uart_puts("R1 emergency stop latency : avg "); uart_put_uint(r1_avg);
    uart_puts(" us, max "); uart_put_uint(mx); uart_puts(" us, min "); uart_put_uint(mn);
    uart_puts(" us, std "); uart_put_uint(sd); uart_puts(" us\r\n");
    uart_puts("R2 jitter (max - min)     : "); uart_put_uint(mx - mn); uart_puts(" us\r\n");
    uart_puts("R3 motor step interval    : target 2000 us, max "); uart_put_uint(r3_max);
    uart_puts(" us, min "); uart_put_uint(r3_min); uart_puts(" us\r\n");
    uart_puts("R4 FND digit period       : target 1000 us, max "); uart_put_uint(r4_max);
    uart_puts(" us, min "); uart_put_uint(r4_min); uart_puts(" us");
#if FND_MODE
    uart_puts(" (FND in LPIT ISR)");
#endif
    uart_puts("\r\n");
    uart_puts("R5 ultrasonic             : "); uart_put_uint(us_count); uart_puts(" measurements in ");
    uart_put_uint(win_ms / 1000u); uart_puts(" s = "); uart_put_uint(per10); uart_puts(" per 10 s (target 100)\r\n");
    uart_puts("M2 tick handler           : avg "); uart_put_uint(m2); uart_puts(" cycles, ");
    uart_put_uint(tick_count); uart_puts(" ticks\r\n");
    uart_puts("context switches          : "); uart_put_uint(ctx_switch_count); uart_puts(" in ");
    uart_put_uint(win_ms); uart_puts(" ms\r\n");
    uart_puts("M3 overhead               : "); uart_put_uint(m3_x100 / 100u); uart_putc('.');
    uart_put_uint((m3_x100 / 10u) % 10u); uart_put_uint(m3_x100 % 10u); uart_puts(" % (tick cycles + switches x M1_CYCLES ");
    uart_put_uint(M1_CYCLES); uart_puts(")\r\n");
    uart_puts("log messages dropped      : "); uart_put_uint(log_dropped); uart_puts("\r\n");
    uart_puts("raw R1 (us) :");
    for (int i = 0; i < N_RUNS; i++) { uart_putc(' '); uart_put_uint(r1_us[i]); }
    uart_puts("\r\n=== end. FND shows R1 average in us ===\r\n");
}
#endif

static void T_Logic(void *p)
{
    event_t ev;
    (void)p;

#if MEASURE
    vTaskDelay(pdMS_TO_TICKS(500));
    measuring = 1;
    meas_start_tick = xTaskGetTickCount();
    set_state(ST_OPENING, "OPENING (measure)");
    motor_start(MEAS_STEPS);
#else
    set_state(ST_IDLE, "ready, PIN = 1234");
#endif

    for (;;) {
        if (xQueueReceive(q_event, &ev, pdMS_TO_TICKS(10)) == pdTRUE) {
            switch (ev.type) {
            case EV_KEY:
                handle_key((int)ev.val);
                break;
            case EV_DIST:
                car = (ev.val > 0 && ev.val <= DETECT_CM);
                if (car) LED0_ON(); else LED0_OFF();
#if USE_OLED
                if (ev.val == 0) OLED_puts(2, 6, "  ---");
                else             OLED_put_uint(2, 6, (uint32_t)ev.val, 5);
#endif
                break;
            case EV_MOTOR_DONE:
                motor_finished();
                break;
            case EV_EMERGENCY:
#if MEASURE
                if (fired) {
                    TOGGLE_HIGH();
                    if (run < N_RUNS) r1_us[run] = (t_stop - t_fire) / CYCLES_PER_US;
                    emg_tick = xTaskGetTickCount();
                }
#endif
                if (state == ST_OPENING || state == ST_CLOSING) enter_emergency("EMERGENCY STOP : SW1");
                break;
            default:
                break;
            }
        }

#if MEASURE
        if (state == ST_CLOSING && !fired && run < N_RUNS &&
            (xTaskGetTickCount() - close_tick) >= pdMS_TO_TICKS(FIRE_BASE_MS + (uint32_t)(run % 10))) {
            fired  = 1;
            t_fire = dwt_now();
            TOGGLE_LOW();                                               // "press" SW1 -> PORTC ISR -> T_Safety
            logmsg("STRESS LOG 0123456789", NO_NUM, 0);                 // 20 chars for T_Log
        }
        if (state == ST_IDLE) {
            measuring = 0;
            report();
            FND_set((int)(r1_avg > 9999u ? 9999u : r1_avg));
            for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
        }
#endif
        periodic_logic();
    }
}

// ================= MEASURE 2 : M1 context switch benchmark =================
#if MEASURE == 2
static SemaphoreHandle_t sem_a, sem_b;
static TaskHandle_t      h_b;

static void T_PingB(void *p)
{
    (void)p;
    for (;;) {
        xSemaphoreTake(sem_b, portMAX_DELAY);
        xSemaphoreGive(sem_a);
    }
}

static void T_PingA(void *p)
{
    uint32_t t0, cycles, tc0, tn0;
    (void)p;
    vTaskDelay(pdMS_TO_TICKS(200));
    tc0 = tick_cycles; tn0 = tick_count;
    t0  = dwt_now();
    for (int i = 0; i < M1_ITER; i++) {
        xSemaphoreGive(sem_b);                                  // wakes B (same priority) ...
        xSemaphoreTake(sem_a, portMAX_DELAY);                   // ... blocks here -> switch to B, B gives, blocks -> switch back
    }
    cycles = dwt_now() - t0;
    uart_puts("\r\n=== structure (3) FreeRTOS : M1 context switch, iterations = "); uart_put_uint(M1_ITER); uart_puts(" ===\r\n");
    uart_puts("total "); uart_put_uint(cycles); uart_puts(" cycles for "); uart_put_uint(2u * M1_ITER); uart_puts(" switches (incl. semaphore give/take)\r\n");
    uart_puts("M1 = "); uart_put_uint(cycles / (2u * M1_ITER)); uart_puts(" cycles = ");
    uart_put_uint(cycles / (2u * M1_ITER) / CYCLES_PER_US); uart_puts(".");
    uart_put_uint((cycles / (2u * M1_ITER) * 10u / CYCLES_PER_US) % 10u); uart_puts(" us  -> put this into M1_CYCLES\r\n");
    uart_puts("M2 = "); uart_put_uint((tick_count - tn0) ? (tick_cycles - tc0) / (tick_count - tn0) : 0);
    uart_puts(" cycles per tick ("); uart_put_uint(tick_count - tn0); uart_puts(" ticks during the test)\r\n");
    uart_puts("=== end ===\r\n");
    FND_set((int)(cycles / (2u * M1_ITER)));
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
#endif

// ================= hooks =================
void vApplicationStackOverflowHook(TaskHandle_t t, char *name)
{
    (void)t; (void)name;
    taskDISABLE_INTERRUPTS();
    LED1_ON(); BUZZER_ON();
    for (;;) {}
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    LED1_ON();
    for (;;) {}
}

// ================= main =================
int main(void)
{
    board_init();
    uart_puts("\r\n=== SMART PARKING BARRIER - structure (3) FreeRTOS ===\r\n");
#if USE_OLED
    if (OLED_init() < 0) uart_puts("OLED : no ACK, running without it\r\n");
#endif

    sem_safety = xSemaphoreCreateBinary();
    sem_echo   = xSemaphoreCreateBinary();
    q_event    = xQueueCreate(16, sizeof(event_t));
    q_log      = xQueueCreate(16, sizeof(logmsg_t));

    PORTC_irq_init();
    NVIC_enable_irq(PORTC_IRQ, PRIO_PORTC_IRQ);
#if FND_MODE
    LPIT_ch0_irq_init(1000);
    NVIC_enable_irq(LPIT0_CH0_IRQ, PRIO_LPIT_IRQ);
#endif
#if MEASURE
    TOGGLE_HIGH();
    uart_puts("MEASURE mode "); uart_put_uint(MEASURE); uart_puts("\r\n");
#endif

#if MEASURE == 2
    sem_a = xSemaphoreCreateBinary();
    sem_b = xSemaphoreCreateBinary();
    xTaskCreate(T_PingA, "pingA", 256, 0, 3, 0);
    xTaskCreate(T_PingB, "pingB", 256, 0, 3, &h_b);
#if !FND_MODE
    xTaskCreate(T_FND, "fnd", 128, 0, PRIO_FND, 0);
#endif
#else
    xTaskCreate(T_Safety, "safety", 192, 0, PRIO_SAFETY, 0);
#if !FND_MODE
    xTaskCreate(T_FND,    "fnd",    128, 0, PRIO_FND,    0);
#endif
    xTaskCreate(T_Motor,  "motor",  192, 0, PRIO_MOTOR,  0);
    xTaskCreate(T_Sensor, "sensor", 192, 0, PRIO_SENSOR, 0);
    xTaskCreate(T_Keypad, "keypad", 192, 0, PRIO_KEYPAD, 0);
    xTaskCreate(T_Logic,  "logic",  512, 0, PRIO_LOGIC,  0);
    xTaskCreate(T_Log,    "log",    256, 0, PRIO_LOG,    0);
#if DUMMY_TASKS > 0
    for (int i = 0; i < DUMMY_TASKS; i++) xTaskCreate(T_Dummy, "dummy", 128, (void *)(uintptr_t)(i + 1), PRIO_DUMMY, 0);
#endif
#endif

    vTaskStartScheduler();
    for (;;) {}                                                 // only if the heap was too small
}
