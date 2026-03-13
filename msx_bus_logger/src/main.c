#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "msx_bus_logger.pio.h"

/* ピン配置 : https://github.com/piigaa-densetu-two-dai/MSXpi/blob/main/MSX%CF%80.pdf */

#define PIN_A0    0
#define PIN_A1    1
#define PIN_A2    2
#define PIN_A3    3
#define PIN_A4    4
#define PIN_A5    5
#define PIN_A6    6
#define PIN_A7    7
#define PIN_A8    8
#define PIN_A9    9
#define PIN_A10  10
#define PIN_A11  11
#define PIN_A12  12
#define PIN_A13  13
#define PIN_A14  14
#define PIN_A15  15

#define PIN_D0   16
#define PIN_D1   17
#define PIN_D2   18
#define PIN_D3   19
#define PIN_D4   20
#define PIN_D5   21
#define PIN_D6   22
#define PIN_D7   23

#define PIN_RD    24  // /RD
#define PIN_WR    25  // /WR
#define PIN_IORQ  26  // /IORQ
#define PIN_SLTSL 27  // /SLTSL
#define PIN_WAIT  28  // /WAIT
#define PIN_BUSDIR 29 // BUSDIR

/* ===== 設定 ===== */
#define DMA_BLOCK_WORDS 128
#define LOG_ENTRIES (8 * 1024)
#define LOG_BUF_MASK (LOG_ENTRIES - 1)

#define POWER_STABLE_MS      300
#define INIT_DISCARD_BLOCKS  2

/* ===== グローバル変数 ===== */
PIO pio = pio0;
uint sm = 0;
int dma_chan;
dma_channel_config dma_cfg;
volatile bool dma_irq_seen = false;
volatile bool drop_event_seen = false;
volatile bool severe_pending = false;

/* カウンタ */
volatile uint32_t dma_write_count = 0;
volatile uint32_t cpu_read_count = 0;

volatile bool dma_raw_drop = false;
volatile uint32_t dma_raw_drop_count = 0;

volatile bool cpu_log_drop = false;
volatile uint32_t cpu_log_drop_count = 0;

uint32_t dma_raw_buf[LOG_ENTRIES];
uint64_t log_buf64[LOG_ENTRIES];

volatile uint32_t dma_wr_idx = 0;
volatile uint32_t log_wr_idx = 0;

static bool drop_latched = false;
static uint8_t drop_seq = 0;
static bool fifo_overflow = false;
static bool dma_lag = false;
static bool dma_lag_prev = false;

volatile bool logger_armed = false;
volatile uint32_t dma_irq_time_us;

extern PIO pio;
extern uint sm;
extern int dma_chan;

/* ユーティリティ */
static inline uint32_t now_us(void) {
    return to_us_since_boot(get_absolute_time());
}

static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}


void enter_safe_state(void) {
    gpio_init(PIN_RD); gpio_set_dir(PIN_RD, GPIO_OUT); gpio_put(PIN_RD, 1);
    gpio_init(PIN_WR); gpio_set_dir(PIN_WR, GPIO_OUT); gpio_put(PIN_WR, 1);
    gpio_init(PIN_WAIT); gpio_set_dir(PIN_WAIT, GPIO_OUT); gpio_pull_up(PIN_WAIT); gpio_put(PIN_WAIT, 1);
}

void pio_init_logger(void) {
    uint offset = pio_add_program(pio, &msx_bus_logger_program);
    pio_sm_config c = msx_bus_logger_program_get_default_config(offset);

    sm_config_set_in_pins(&c, PIN_D0);
    sm_config_set_jmp_pin(&c, PIN_IORQ);
    sm_config_set_in_shift(&c, false, false, 32);

    int data_pins[] = {PIN_D0,PIN_D1,PIN_D2,PIN_D3,PIN_D4,PIN_D5,PIN_D6,PIN_D7};
    for (int i = 0; i < 8; ++i) {
        int p = data_pins[i];
        if (p == 24 || p == 25) continue;
        gpio_init(p); gpio_set_dir(p, GPIO_IN); gpio_pull_up(p);
        pio_gpio_init(pio, p);
        pio_sm_set_consecutive_pindirs(pio, sm, p, 1, false);
    }

    gpio_init(PIN_RD); gpio_set_dir(PIN_RD, GPIO_IN); gpio_pull_up(PIN_RD);
    pio_gpio_init(pio, PIN_RD); pio_sm_set_consecutive_pindirs(pio, sm, PIN_RD, 1, false);

    gpio_init(PIN_WR); gpio_set_dir(PIN_WR, GPIO_IN); gpio_pull_up(PIN_WR);
    pio_gpio_init(pio, PIN_WR); pio_sm_set_consecutive_pindirs(pio, sm, PIN_WR, 1, false);

    gpio_init(PIN_IORQ); gpio_set_dir(PIN_IORQ, GPIO_IN); gpio_pull_up(PIN_IORQ);
    pio_gpio_init(pio, PIN_IORQ);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void __isr dma_handler(void) {
    dma_hw->ints0 = 1u << dma_chan;
    dma_irq_seen = true;
    dma_irq_time_us = time_us_32();
    __atomic_thread_fence(__ATOMIC_ACQ_REL);
    dma_write_count += DMA_BLOCK_WORDS;
    __atomic_thread_fence(__ATOMIC_ACQ_REL);

    if (logger_armed) {
        uint32_t pending = dma_write_count - cpu_read_count;
        if (pending > LOG_ENTRIES) {
            dma_lag = true;
            drop_latched = true;
            drop_seq++;
            drop_event_seen = true;
            dma_raw_drop_count++;
        }
    }
}

void dma_init_logger_with_ring_enabled(PIO pio, uint sm) {
    dma_chan = dma_claim_unused_channel(true);
    dma_cfg = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(pio, sm, false));
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true);
    channel_config_set_chain_to(&dma_cfg, dma_chan);

    dma_channel_configure(dma_chan, &dma_cfg,
                          dma_raw_buf, &pio->rxf[sm],
                          DMA_BLOCK_WORDS, false);

    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_start_channel_mask(1u << dma_chan);
}

static inline uint8_t make_flags(uint32_t pio_v) {
    uint8_t f = 0;
    if (!(pio_v & (1u << 8)))  f |= 1 << 0;
    if (!(pio_v & (1u << 9)))  f |= 1 << 1;
    if (!gpio_get(PIN_IORQ))   f |= 1 << 2;
    if (gpio_get(PIN_SLTSL))   f |= 1 << 3;
    if (drop_latched)          f |= 1 << 4;
    if (fifo_overflow)         f |= 1 << 5;
    if (dma_lag)               f |= 1 << 6;
    return f;
}

static const int addr_pins[16] = {
    PIN_A0, PIN_A1, PIN_A2, PIN_A3,
    PIN_A4, PIN_A5, PIN_A6, PIN_A7,
    PIN_A8, PIN_A9, PIN_A10, PIN_A11,
    PIN_A12, PIN_A13, PIN_A14, PIN_A15
};

static inline uint16_t read_addr_pins(void) {
    uint16_t addr = 0;
    for (int i = 0; i < 16; i++) addr |= (gpio_get(addr_pins[i]) << i);
    return addr;
}

/* ===== メイン ===== */
int main(void) {
    stdio_init_all();
	sleep_ms(250); // wait for USB serial connection if needed
    printf("msx_bus_logger: startup (safe-mode)\n");
    enter_safe_state();

    //sleep_ms(10000);

    pio_init_logger();
    printf("\n[SETUP] pio_init_logger() done.\n");

    dma_init_logger_with_ring_enabled(pio, sm);
    printf("\n[SETUP] dma_init_logger_with_ring_enabled() done.\n");

    __atomic_store_n(&cpu_read_count, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&dma_write_count, 0u, __ATOMIC_RELEASE);
    drop_seq = 0;
    dma_lag = false;
    drop_latched = false;
    logger_armed = false;

    //const uint32_t expected_initial = DMA_BLOCK_WORDS * INIT_DISCARD_BLOCKS;
    //while (__atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE) < expected_initial) tight_loop_contents();

    __atomic_store_n(&cpu_read_count, __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE), __ATOMIC_RELEASE);
    log_wr_idx = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE) & LOG_BUF_MASK;

    logger_armed = true;
    printf("\n[START] MAIN LOOP START\n");

    const uint32_t status_interval_ms = 1000;
    uint32_t last_status_ms = now_ms();
    uint32_t last_write_count = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE);

    while (1) {
        uint32_t written = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE);
        static uint32_t led_until_us = 0;
        static bool led_on = false;

        if (dma_irq_seen) {
            dma_irq_seen = false;
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            led_on = true;
            led_until_us = now_us() + 10000;
        }

        if (drop_event_seen) {
            drop_event_seen = false;
            uint32_t w = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE);
            uint32_t r = __atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE);
            uint32_t pending = (w >= r) ? (w - r) : 0;
            if (pending > LOG_ENTRIES) {
                uint32_t overrun = pending - LOG_ENTRIES;
                dma_raw_drop = true;
                dma_raw_drop_count += overrun;
                uint32_t new_r = w - LOG_ENTRIES;
                __atomic_store_n(&cpu_read_count, new_r, __ATOMIC_RELEASE);
                drop_latched = true;
                dma_lag = true;
            }

            uint32_t r_after = __atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE);
            uint32_t pending_after = (__atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE) - r_after);
            printf("[EVENT] DROP detected: drop_seq=%u pending=%u dma_lag=%d",
                   (unsigned)drop_seq, (unsigned)pending_after, dma_lag);

            const int N = 8;
            uint32_t start = (r_after - N) & LOG_BUF_MASK;
            printf("Recent raw samples:");
            for (int i = 0; i < N; ++i) {
                uint32_t s = dma_raw_buf[(start + i) & LOG_BUF_MASK];
                printf(" %08x", s);
            }
            printf("\n");

            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            led_on = true;
            led_until_us = now_us() + 200;
        }

        if (drop_latched) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            led_on = true;
        } else {
            if (led_on && now_us() > led_until_us) {
                gpio_put(PICO_DEFAULT_LED_PIN, 0);
                led_on = false;
            }
        }

        uint32_t now = now_ms();
        if ((int32_t)(now - last_status_ms) >= 0 && now - last_status_ms >= status_interval_ms) {
            uint32_t w = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE);
            uint32_t r = __atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE);
            uint32_t pending = (w >= r) ? (w - r) : 0;
            uint32_t delta = w - last_write_count;
            float rate_wps = (float)delta / ((now - last_status_ms) / 1000.0f);
            last_write_count = w;
            last_status_ms = now;

            uint32_t write_idx = w & LOG_BUF_MASK;
            uint32_t read_idx = r & LOG_BUF_MASK;
            uint32_t used = (pending > LOG_ENTRIES) ? LOG_ENTRIES : pending;

            printf("[STATUS] w=%u r=%u pending=%u used=%u w_idx=0x%04x r_idx=0x%04x rate=%0.1f w/s drop_seq=%u drops=%u\n",
                   (unsigned)w, (unsigned)r, (unsigned)pending, (unsigned)used,
                   (unsigned)write_idx, (unsigned)read_idx, rate_wps,
                   (unsigned)drop_seq, (unsigned)dma_raw_drop_count);
        }

        while (__atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE) < written) {
            uint32_t idx = __atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE) & LOG_BUF_MASK;
            uint32_t raw = dma_raw_buf[idx];

            __atomic_store_n(&cpu_read_count, __atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE) + 1u, __ATOMIC_RELEASE);

            uint8_t data = raw & 0xFF;
            uint8_t flags = make_flags(raw);
            uint16_t addr = read_addr_pins();
            uint8_t timeL = dma_irq_time_us & 0xFF;

            uint8_t rsv = drop_seq & 0x0F;
            uint64_t pack = 0;
            pack |= ((uint64_t)flags << 56);
            pack |= ((uint64_t)rsv   << 48);
            pack |= ((uint64_t)addr  << 32);
            pack |= ((uint64_t)data  << 16);

            log_buf64[log_wr_idx & LOG_BUF_MASK] = pack;
            log_wr_idx++;
        }

        int ch = getchar_timeout_us(0);
        if (ch == 'c') {
            drop_latched = false;
            drop_seq = 0;
            fifo_overflow = false;
            dma_lag = false;
            printf("[CMD] Cleared drop_latched\n");
        }

        tight_loop_contents();
    }

    return 0;
}