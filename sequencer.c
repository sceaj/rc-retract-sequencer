#ifndef F_CPU
#define F_CPU 16000000UL // Change to 8000000UL if using the 3.3V Pro Mini
#endif

#ifndef ENABLE_UART_LOGGING
#define ENABLE_UART_LOGGING 0
#endif

#ifndef ENABLE_DIAGNOSTICS
#define ENABLE_DIAGNOSTICS 0
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>

typedef enum {
    CMD_NONE = 0,
    CMD_LOWER,
    CMD_RAISE
} gear_command_t;

typedef enum {
    STATE_IDLE = 0,
    STATE_LOWERING,
    STATE_RAISING
} gear_state_t;

typedef enum {
    CH_DOOR = 0,
    CH_LEFT,
    CH_RIGHT,
    CH_NOSE,
    CH_SERVO_COUNT,
    CH_LANDING_LIGHTS,
    CH_COUNT
} servo_channel_t;

/* Tunable input command thresholds (microseconds). */
#define INPUT_LOWER_US 1250U
#define INPUT_UPPER_US 1750U

/* Accept only realistic RC servo input pulse widths. */
#define INPUT_VALID_MIN_US 500U
#define INPUT_VALID_MAX_US 2500U

/* Tunable loss-of-signal timeout (milliseconds). */
#define INPUT_TIMEOUT_MS 120U

/* LED status feedback timing (milliseconds). */
#define LED_BLINK_ON_MS 350U
#define LED_BLINK_OFF_MS 350U
#define LED_SEQUENCE_GAP_MS 1250U

/* LED status feedback codes. */
#define LED_CODE_LOWER_CMD 1U
#define LED_CODE_RAISE_CMD 2U
#define LED_CODE_STARTUP_MODE 3U

#define LED_EVENT_QUEUE_LEN 8U

/* Tunable servo pulse placeholders (microseconds). */
#define DOOR_OPEN_US 950U
#define DOOR_CLOSE_US 2050U
#define LEFT_UP_US 2000U
#define LEFT_DOWN_US 1000U
#define RIGHT_UP_US 2000U
#define RIGHT_DOWN_US 1000U
#define NOSE_UP_US 2000U
#define NOSE_DOWN_US 1000U

/* Tunable per-channel slew-rate limits (microseconds per millisecond). */
#define DOOR_SLEW_US_PER_MS 1U
#define LEFT_SLEW_US_PER_MS 20U
#define RIGHT_SLEW_US_PER_MS 20U
#define NOSE_SLEW_US_PER_MS 20U

/* Tunable startup-neutral outputs for invalid/none stored command state. */
#define STARTUP_DOOR_US DOOR_OPEN_US
#define STARTUP_LEFT_US LEFT_DOWN_US
#define STARTUP_RIGHT_US RIGHT_DOWN_US
#define STARTUP_NOSE_US NOSE_DOWN_US

#define SERVO_FRAME_TICKS_10US 2000U
#define SERVO_MIN_US 500U
#define SERVO_MAX_US 2500U

static const uint16_t k_servo_slew_us_per_ms[CH_SERVO_COUNT] = {
    DOOR_SLEW_US_PER_MS,
    LEFT_SLEW_US_PER_MS,
    RIGHT_SLEW_US_PER_MS,
    NOSE_SLEW_US_PER_MS
};

volatile uint32_t last_pulse_time_ms = 0;
volatile uint16_t latest_pulse_us = 0;
volatile uint8_t pulse_valid = 0;

static volatile uint32_t ms_ticks = 0;
static volatile uint8_t pulse_sample_ready = 0;
static uint8_t outputs_enabled = 0; /* Phase 5: Gate pulse generation until first command. */

static volatile uint16_t servo_target_us[CH_SERVO_COUNT] = {
    STARTUP_DOOR_US,
    STARTUP_LEFT_US,
    STARTUP_RIGHT_US,
    STARTUP_NOSE_US
};

static volatile uint16_t servo_pulse_ticks_10us[CH_SERVO_COUNT] = {
    STARTUP_DOOR_US / 10U,
    STARTUP_LEFT_US / 10U,
    STARTUP_RIGHT_US / 10U,
    STARTUP_NOSE_US / 10U
};

static uint16_t servo_current_us[CH_SERVO_COUNT] = {
    STARTUP_DOOR_US,
    STARTUP_LEFT_US,
    STARTUP_RIGHT_US,
    STARTUP_NOSE_US
};

typedef enum {
    LED_PHASE_IDLE = 0,
    LED_PHASE_ON,
    LED_PHASE_OFF,
    LED_PHASE_GAP
} led_phase_t;

static uint8_t led_event_queue[LED_EVENT_QUEUE_LEN] = {0};
static uint8_t led_queue_head = 0;
static uint8_t led_queue_tail = 0;
static led_phase_t led_phase = LED_PHASE_IDLE;
static uint16_t led_phase_remaining_ms = 0;
static uint8_t led_blinks_remaining = 0;

typedef struct {
    uint32_t offset_ms;
    servo_channel_t channel;
    uint16_t pulse_us;
} sequence_step_t;

typedef struct {
    const sequence_step_t *steps;
    uint8_t step_count;
    uint8_t next_step_idx;
    uint32_t start_ms;
    uint8_t active;
    gear_command_t command;
} sequence_runtime_t;

static const sequence_step_t k_lower_sequence[] = {
    {0U, CH_DOOR, DOOR_OPEN_US},
    {1200U, CH_LEFT, LEFT_DOWN_US},
    {2200U, CH_RIGHT, RIGHT_DOWN_US},
    {3200U, CH_NOSE, NOSE_DOWN_US},
    {6000U, CH_LANDING_LIGHTS, 1U} /* Turn on landing lights after 6 seconds. */
};

static const sequence_step_t k_raise_sequence[] = {
    {0U, CH_LEFT, LEFT_UP_US},
    {10U, CH_LANDING_LIGHTS, 0U}, /* Turn off landing lights early in raising sequence. */
    {1000U, CH_RIGHT, RIGHT_UP_US},
    {2000U, CH_NOSE, NOSE_UP_US},
    {7000U, CH_DOOR, DOOR_CLOSE_US}
};

static sequence_runtime_t sequence_rt = {
    0
};

#if ENABLE_UART_LOGGING
#include <stdio.h>

static int uart_putchar(char c, FILE *stream) {
    if (c == '\n') {
        uart_putchar('\r', stream);
    }

    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = c;
    return 0;
}

static FILE uart_stdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

static void uart_init(void) {
    uint16_t ubrr = (F_CPU / 16 / 57600UL) - 1;

    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0A = 0;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    stdout = &uart_stdout;
}
#define LOGF(...) printf(__VA_ARGS__)
#else
static void uart_init(void) {
}
#define LOGF(...) do { } while (0)
#endif

static void timer0_init_1khz(void) {
    TCCR0A = (1 << WGM01); // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
    OCR0A = 249; // 1 kHz at 16 MHz
    TIMSK0 = (1 << OCIE0A);
}

static void timer1_input_capture_init(void) {
    DDRB &= ~(1 << DDB0); // D8/ICP1 as input
    PORTB &= ~(1 << PORTB0); // No pull-up

    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11); // Noise cancel, rising edge, /8
    TIFR1 = (1 << ICF1) | (1 << TOV1); // Clear any stale capture/overflow flags
    TIMSK1 = (1 << ICIE1);
}

/* Ensure pulses are within valid range */
static uint16_t clamp_servo_us(uint16_t pulse_us) {
    if (pulse_us < SERVO_MIN_US) {
        return SERVO_MIN_US;
    }
    if (pulse_us > SERVO_MAX_US) {
        return SERVO_MAX_US;
    }
    return pulse_us;
}

/* Convert microseconds to 10us ticks and ensure within valid range */
static uint16_t us_to_ticks_10us(uint16_t pulse_us) {
    uint16_t ticks = (uint16_t)((pulse_us + 5U) / 10U);

    if (ticks == 0U) {
        ticks = 1U;
    }
    if (ticks >= SERVO_FRAME_TICKS_10US) {
        ticks = SERVO_FRAME_TICKS_10US - 1U;
    }
    return ticks;
}

static uint8_t servo_channel_mask_d(servo_channel_t channel) {
    if (channel == CH_DOOR) {
        return (1 << PORTD3); // D3
    }
    if (channel == CH_LEFT) {
        return (1 << PORTD5); // D5
    }
    return 0U;
}

static uint8_t servo_channel_mask_b(servo_channel_t channel) {
    if (channel == CH_RIGHT) {
        return (1 << PORTB1); // D9
    }
    if (channel == CH_NOSE) {
        return (1 << PORTB2); // D10
    }
    return 0U;
}

static void servo_io_init(void) {
    DDRD |= (1 << DDD3) | (1 << DDD5);
    DDRB |= (1 << DDB1) | (1 << DDB2);

    PORTD &= (uint8_t)~((1 << PORTD3) | (1 << PORTD5));
    PORTB &= (uint8_t)~((1 << PORTB1) | (1 << PORTB2));
}

static void timer2_init_10us(void) {
    TCCR2A = (1 << WGM21); // CTC mode
    TCCR2B = (1 << CS21); // Prescaler 8 => 0.5 us/tick
    OCR2A = 19; // 10 us period
    TIMSK2 = (1 << OCIE2A);
}

static void status_led_init(void) {
    DDRB |= (1 << DDB5); // D13 onboard LED
    PORTB &= (uint8_t)~(1 << PORTB5);
}

static void status_led_set(uint8_t on) {
    if (on) {
        PORTB |= (1 << PORTB5);
    } else {
        PORTB &= (uint8_t)~(1 << PORTB5);
    }
}

static void status_led_enqueue(uint8_t code) {
    uint8_t next_tail;

    if (code == 0U) {
        return;
    }

    next_tail = (uint8_t)((led_queue_tail + 1U) % LED_EVENT_QUEUE_LEN);
    if (next_tail == led_queue_head) {
        return; // Queue full; drop newest event.
    }

    led_event_queue[led_queue_tail] = code;
    led_queue_tail = next_tail;
}

static uint8_t status_led_dequeue(uint8_t *code) {
    if (led_queue_head == led_queue_tail) {
        return 0U;
    }

    *code = led_event_queue[led_queue_head];
    led_queue_head = (uint8_t)((led_queue_head + 1U) % LED_EVENT_QUEUE_LEN);
    return 1U;
}

static void status_led_update_1ms(void) {
    uint8_t next_code;

    if (led_phase_remaining_ms > 0U) {
        led_phase_remaining_ms--;
        return;
    }

    if (led_phase == LED_PHASE_IDLE) {
        if (!status_led_dequeue(&next_code)) {
            status_led_set(0);
            return;
        }

        led_blinks_remaining = next_code;
        led_phase = LED_PHASE_ON;
        led_phase_remaining_ms = LED_BLINK_ON_MS;
        status_led_set(1);
        return;
    }

    if (led_phase == LED_PHASE_ON) {
        status_led_set(0);
        if (led_blinks_remaining > 0U) {
            led_blinks_remaining--;
        }

        if (led_blinks_remaining == 0U) {
            led_phase = LED_PHASE_GAP;
            led_phase_remaining_ms = LED_SEQUENCE_GAP_MS;
        } else {
            led_phase = LED_PHASE_OFF;
            led_phase_remaining_ms = LED_BLINK_OFF_MS;
        }
        return;
    }

    if (led_phase == LED_PHASE_OFF) {
        led_phase = LED_PHASE_ON;
        led_phase_remaining_ms = LED_BLINK_ON_MS;
        status_led_set(1);
        return;
    }

    led_phase = LED_PHASE_IDLE;
}

static void landing_lights_init(void) {
    DDRD |= (1 << DDD4); // D4 as output for landing lights
    PORTD &= (uint8_t)~(1 << PORTD4); // Start with landing lights off
}

static void landing_lights_set(uint8_t on) {
    if (on) {
        PORTD |= (1 << PORTD4);
    } else {
        PORTD &= (uint8_t)~(1 << PORTD4);
    }
}

static void set_servo_target(servo_channel_t channel, uint16_t pulse_us) {
    uint16_t clamped = clamp_servo_us(pulse_us);

    if ((uint8_t)channel >= CH_SERVO_COUNT) {
        return;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        servo_target_us[channel] = clamped;
    }
}

static void servo_update_1ms(void) {
    uint16_t target_snapshot[CH_SERVO_COUNT];
    uint16_t ticks_snapshot[CH_SERVO_COUNT];
    uint8_t i;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (i = 0; i < CH_SERVO_COUNT; i++) {
            target_snapshot[i] = servo_target_us[i];
        }
    }

    for (i = 0; i < CH_SERVO_COUNT; i++) {
        uint16_t current = servo_current_us[i];
        uint16_t target = clamp_servo_us(target_snapshot[i]);
        uint16_t step = k_servo_slew_us_per_ms[i];

        if (step == 0U) {
            step = 1U;
        }

        if (current < target) {
            uint16_t delta = (uint16_t)(target - current);
            if (delta > step) {
                delta = step;
            }
            current = (uint16_t)(current + delta);
        } else if (current > target) {
            uint16_t delta = (uint16_t)(current - target);
            if (delta > step) {
                delta = step;
            }
            current = (uint16_t)(current - delta);
        }

        servo_current_us[i] = current;
        ticks_snapshot[i] = us_to_ticks_10us(current);
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (i = 0; i < CH_SERVO_COUNT; i++) {
            servo_pulse_ticks_10us[i] = ticks_snapshot[i];
        }
    }
}

static void start_sequence(gear_command_t command, uint32_t now_ms) {
    if (command == CMD_LOWER) {
        sequence_rt.steps = k_lower_sequence;
        sequence_rt.step_count = (uint8_t)(sizeof(k_lower_sequence) / sizeof(k_lower_sequence[0]));
    } else if (command == CMD_RAISE) {
        sequence_rt.steps = k_raise_sequence;
        sequence_rt.step_count = (uint8_t)(sizeof(k_raise_sequence) / sizeof(k_raise_sequence[0]));
    } else {
        sequence_rt.steps = 0;
        sequence_rt.step_count = 0;
        sequence_rt.active = 0;
        sequence_rt.command = CMD_NONE;
        return;
    }

    sequence_rt.command = command;
    sequence_rt.start_ms = now_ms;
    sequence_rt.next_step_idx = 0;
    sequence_rt.active = 1;
}

static void process_sequence(uint32_t now_ms) {
    if (!sequence_rt.active) {
        return;
    }

    while (sequence_rt.next_step_idx < sequence_rt.step_count) {
        const sequence_step_t *step = &sequence_rt.steps[sequence_rt.next_step_idx];
        uint32_t elapsed_ms = (uint32_t)(now_ms - sequence_rt.start_ms);

        if (elapsed_ms < step->offset_ms) {
            break;
        }

        if (step->channel < CH_SERVO_COUNT) {
            set_servo_target(step->channel, step->pulse_us);
        } else if (step->channel == CH_LANDING_LIGHTS) {
            landing_lights_set((uint8_t)step->pulse_us);
        }
        sequence_rt.next_step_idx++;
    }

    if (sequence_rt.next_step_idx >= sequence_rt.step_count) {
        sequence_rt.active = 0;
    }
}

static gear_command_t decode_command_from_pulse_us(uint16_t pulse_us) {
    if (pulse_us < INPUT_LOWER_US) {
        return CMD_LOWER;
    }
    if (pulse_us > INPUT_UPPER_US) {
        return CMD_RAISE;
    }
    return CMD_NONE;
}

#if ENABLE_UART_LOGGING && ENABLE_DIAGNOSTICS
static const char *gear_command_label(gear_command_t command) {
    if (command == CMD_LOWER) {
        return "LOWER";
    }
    if (command == CMD_RAISE) {
        return "RAISE";
    }
    return "NONE";
}

static const char *gear_state_label(gear_state_t state) {
    if (state == STATE_LOWERING) {
        return "LOWERING";
    }
    if (state == STATE_RAISING) {
        return "RAISING";
    }
    return "IDLE";
}

static gear_state_t current_gear_state(void) {
    if (sequence_rt.active) {
        if (sequence_rt.command == CMD_RAISE) {
            return STATE_RAISING;
        }
        if (sequence_rt.command == CMD_LOWER) {
            return STATE_LOWERING;
        }
    }
    return STATE_IDLE;
}

static void emit_diagnostics(uint32_t now_ms, uint16_t pulse_us, gear_command_t decoded_command) {
    static uint32_t last_diag_ms = 0;

    if ((uint32_t)(now_ms - last_diag_ms) < 1000U) {
        return;
    }

    last_diag_ms = now_ms;
    LOGF("diag t=%lu pulse=%u decoded=%s state=%s seq=%u/%u armed=%u\n",
         (unsigned long)now_ms,
         (unsigned)pulse_us,
         gear_command_label(decoded_command),
         gear_state_label(current_gear_state()),
         (unsigned)sequence_rt.next_step_idx,
         (unsigned)sequence_rt.step_count,
         (unsigned)outputs_enabled);
}
#endif

ISR(TIMER0_COMPA_vect) {
    ms_ticks++;
}

ISR(TIMER1_CAPT_vect) {
    static uint16_t rise_count = 0;
    uint16_t capture_count = ICR1;

    if (TCCR1B & (1 << ICES1)) {
        rise_count = capture_count;
        TCCR1B &= ~(1 << ICES1); // Next capture on falling edge
    } else {
        uint16_t width_counts = (uint16_t)(capture_count - rise_count);
        uint16_t pulse_us = (uint16_t)(width_counts / 2U); // Timer1 at /8 -> 0.5 us per count

        if ((pulse_us >= INPUT_VALID_MIN_US) && (pulse_us <= INPUT_VALID_MAX_US)) {
            latest_pulse_us = pulse_us;
            last_pulse_time_ms = ms_ticks;
            pulse_valid = 1;
            pulse_sample_ready = 1;
        }
        TCCR1B |= (1 << ICES1); // Next capture on rising edge
    }
}

ISR(TIMER2_COMPA_vect) {
    static uint16_t frame_tick_10us = 0;
    uint16_t ticks_door = servo_pulse_ticks_10us[CH_DOOR];
    uint16_t ticks_left = servo_pulse_ticks_10us[CH_LEFT];
    uint16_t ticks_right = servo_pulse_ticks_10us[CH_RIGHT];
    uint16_t ticks_nose = servo_pulse_ticks_10us[CH_NOSE];

    /* Phase 5: Gate pulse generation based on outputs_enabled. */
    if (!outputs_enabled) {
        return; /* No output until first command. */
    }

    if (frame_tick_10us == 0U) {
        PORTD |= (1 << PORTD3) | (1 << PORTD5);
        PORTB |= (1 << PORTB1) | (1 << PORTB2);
    }

    if (frame_tick_10us >= ticks_door) {
        PORTD &= (uint8_t)~servo_channel_mask_d(CH_DOOR);
    }
    if (frame_tick_10us >= ticks_left) {
        PORTD &= (uint8_t)~servo_channel_mask_d(CH_LEFT);
    }
    if (frame_tick_10us >= ticks_right) {
        PORTB &= (uint8_t)~servo_channel_mask_b(CH_RIGHT);
    }
    if (frame_tick_10us >= ticks_nose) {
        PORTB &= (uint8_t)~servo_channel_mask_b(CH_NOSE);
    }

    frame_tick_10us++;
    if (frame_tick_10us >= SERVO_FRAME_TICKS_10US) {
        frame_tick_10us = 0U;
    }
}

int main(void) {
    gear_command_t latched_cmd = CMD_NONE;
    uint32_t last_servo_update_ms = 0;
    uint8_t input_cmd_initialized = 0;
#if ENABLE_UART_LOGGING && ENABLE_DIAGNOSTICS
    gear_command_t last_decoded_cmd = CMD_NONE;
#endif

    uart_init();
    servo_io_init();
    landing_lights_init();
    status_led_init();
    timer0_init_1khz();
    timer1_input_capture_init();
    timer2_init_10us();

    /* Phase 5: Initialize all servo targets to 1500 us (neutral), but do NOT enable output generation yet. */
    set_servo_target(CH_DOOR, STARTUP_DOOR_US);
    set_servo_target(CH_LEFT, STARTUP_LEFT_US);
    set_servo_target(CH_RIGHT, STARTUP_RIGHT_US);
    set_servo_target(CH_NOSE, STARTUP_NOSE_US);
    outputs_enabled = 0; /* Gate is closed: no pulses until first command. */
    status_led_enqueue(LED_CODE_STARTUP_MODE);

    sei();
    LOGF("Startup: outputs disabled, awaiting first command.\n");

    while (1) {
        uint32_t now_ms;
        uint8_t sample_ready_snapshot;
        uint16_t pulse_us_snapshot = 0;

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            now_ms = ms_ticks;
        }

        while (last_servo_update_ms != now_ms) {
            servo_update_1ms();
            status_led_update_1ms();
            last_servo_update_ms++;
        }

        process_sequence(now_ms);

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            sample_ready_snapshot = pulse_sample_ready;
            if (sample_ready_snapshot) {
                pulse_sample_ready = 0;
                pulse_us_snapshot = latest_pulse_us;
            }
        }

        if (sample_ready_snapshot) {
            gear_command_t command = decode_command_from_pulse_us(pulse_us_snapshot);
#if ENABLE_UART_LOGGING && ENABLE_DIAGNOSTICS
            last_decoded_cmd = command;
#endif

            /* Startup behavior: baseline the first valid command without enabling outputs. */
            if (!input_cmd_initialized) {
                latched_cmd = command;
                input_cmd_initialized = 1;
                LOGF("input_baseline command=%u pulse_us=%u\n", (unsigned)command, (unsigned)pulse_us_snapshot);
                continue;
            }

            if (command == CMD_NONE) {
                latched_cmd = CMD_NONE;
            } else if (command != latched_cmd) {
                latched_cmd = command;
                outputs_enabled = 1; /* Enable pulse generation on first command. */
                start_sequence(command, now_ms);
                status_led_enqueue((command == CMD_LOWER) ? LED_CODE_LOWER_CMD : LED_CODE_RAISE_CMD);
                LOGF("command_transition=%u pulse_us=%u outputs_enabled\n", (unsigned)command, (unsigned)pulse_us_snapshot);
            }
        }

#if ENABLE_UART_LOGGING && ENABLE_DIAGNOSTICS
        emit_diagnostics(now_ms, pulse_us_snapshot, last_decoded_cmd);
#endif
    }
}
