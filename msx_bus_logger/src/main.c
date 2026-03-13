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
#define PIN_BUSDIR 29 // BUSDIR（未使用）

/* ===== 設定 ===== */
#define DMA_BLOCK_WORDS 128
#define LOG_ENTRIES (8 * 1024)
#define LOG_BUF_MASK (LOG_ENTRIES - 1)

/* ===== グローバル変数 ===== */
PIO pio = pio0;
uint sm = 0;
int dma_chan;
dma_channel_config dma_cfg;

/* カウンタ */
volatile uint32_t dma_write_count = 0;
volatile uint32_t cpu_read_count = 0;

uint32_t dma_raw_buf[LOG_ENTRIES];
uint64_t log_buf64[LOG_ENTRIES]; /* 将来のバイナリ転送に備えて保持 */

volatile uint32_t dma_wr_idx = 0;
volatile uint32_t log_wr_idx = 0;

static bool drop_latched = false;
static uint8_t drop_seq = 0;
static bool dma_lag = false;

volatile bool logger_armed = false;
volatile uint32_t dma_irq_time_us;

/* ユーティリティ */
static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

void enter_safe_state(void) {
    /* /RD と /WR は OUT+High（MSXロゴ表示のために必須） */
    gpio_init(PIN_RD);    gpio_set_dir(PIN_RD,    GPIO_OUT); gpio_put(PIN_RD,    1);
    gpio_init(PIN_WR);    gpio_set_dir(PIN_WR,    GPIO_OUT); gpio_put(PIN_WR,    1);
    /* /WAIT は OUT+High+プルアップ */
    gpio_init(PIN_WAIT);  gpio_set_dir(PIN_WAIT,  GPIO_OUT); gpio_pull_up(PIN_WAIT); gpio_put(PIN_WAIT, 1);
    /* その他の制御線は入力（Hi-Z） */
    gpio_init(PIN_IORQ);  gpio_set_dir(PIN_IORQ,  GPIO_IN);  gpio_disable_pulls(PIN_IORQ);
    gpio_init(PIN_SLTSL); gpio_set_dir(PIN_SLTSL, GPIO_IN);  gpio_disable_pulls(PIN_SLTSL);
}

void pio_init_logger(void) {
    uint offset = pio_add_program(pio, &msx_bus_logger_program);
    pio_sm_config c = msx_bus_logger_program_get_default_config(offset);

    sm_config_set_in_pins(&c, PIN_D0);
    sm_config_set_jmp_pin(&c, PIN_IORQ);
    sm_config_set_in_shift(&c,
        false,  /* 左シフト */
        false,  /* autopush 無効（.pio が手動 push するため） */
        32      /* threshold（autopush 無効時は参照されないが明示的に設定） */
    );

    /* GPIO16〜26（D0-D7 + /RD + /WR + /IORQ）を PIO 入力として初期化
     * プルアップは設定しない：/RD, /WR は MSX 側で駆動される出力線、
     * D0-D7 は MSX がドライブするデータバスであるため不要 */
    for (int i = PIN_D0; i <= PIN_IORQ; i++) {
        pio_gpio_init(pio, i);
        pio_sm_set_consecutive_pindirs(pio, sm, i, 1, false);
    }

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    /* オンボードLED初期化 */
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void __isr dma_handler(void) {
    dma_irq_time_us = time_us_32();
    dma_hw->ints0 = 1u << dma_chan;

    /* dma_wr_idx を進めてからリングバッファの次のブロック先頭を書き込み先にセット */
    dma_wr_idx += DMA_BLOCK_WORDS;
    dma_write_count = dma_wr_idx;

    dma_channel_set_read_addr(dma_chan, &pio->rxf[sm], false);
    dma_channel_set_write_addr(dma_chan, &dma_raw_buf[dma_wr_idx & LOG_BUF_MASK], true);

    if (!logger_armed) return;

    /* DROP 検出（リングバッファのオーバーラン） */
    uint32_t pending = dma_write_count - cpu_read_count;
    if (pending > LOG_ENTRIES) {
        dma_lag = true;
        drop_latched = true;
        drop_seq++;
    }
}

void dma_init_logger_with_ring_enabled(PIO pio, uint sm) {
    dma_chan = dma_claim_unused_channel(true);
    dma_cfg = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(pio, sm, false));
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true);
    channel_config_set_chain_to(&dma_cfg, dma_chan); /* 自身へのチェーン（チェーン無効） */

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
    if (!(pio_v & (1u << 8)))  f |= 1 << 0; /* /RD がアサート → RD フラグ */
    if (!(pio_v & (1u << 9)))  f |= 1 << 1; /* /WR がアサート → WR フラグ */
    if (!(pio_v & (1u << 10))) f |= 1 << 2; /* /IORQ がアサート → IO フラグ */
    if (drop_latched)          f |= 1 << 4; /* DROP ラッチフラグ */
    if (dma_lag)               f |= 1 << 6; /* DMA 遅延フラグ */
    return f;
}

static inline uint16_t read_addr_pins(void) {
    /* GPIO0-15 = A0-A15 なので下位16bitをそのまま使う */
    return (uint16_t)(gpio_get_all() & 0xFFFF);
}

/* ===== メイン ===== */
int main(void) {
    stdio_init_all();
    sleep_ms(250);
    printf("msx_bus_logger: 起動（セーフモード）\n");
    enter_safe_state();

    pio_init_logger();
    printf("[SETUP] pio_init_logger() 完了\n");

    dma_init_logger_with_ring_enabled(pio, sm);
    printf("[SETUP] dma_init_logger_with_ring_enabled() 完了\n");

    __atomic_store_n(&cpu_read_count, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&dma_write_count, 0u, __ATOMIC_RELEASE);
    drop_seq = 0;
    dma_lag = false;
    drop_latched = false;
    logger_armed = false;

    __atomic_store_n(&cpu_read_count, __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE), __ATOMIC_RELEASE);
    log_wr_idx = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE) & LOG_BUF_MASK;

    logger_armed = true;
    printf("[START] メインループ開始\n");

    const uint32_t status_interval_ms = 1000;
    uint32_t last_status_ms = now_ms();
    uint32_t last_write_count = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE);

    while (1) {
        uint32_t written = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE);

        /* DROP 中はLEDを点灯 */
        gpio_put(PICO_DEFAULT_LED_PIN, drop_latched ? 1 : 0);

        /* 1秒ごとのステータス出力 */
        uint32_t now = now_ms();
        if (now - last_status_ms >= status_interval_ms) {
            uint32_t w = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE);
            uint32_t r = __atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE);
            uint32_t pending = (w >= r) ? (w - r) : 0;
            uint32_t delta = w - last_write_count;
            float rate_wps = (float)delta / ((now - last_status_ms) / 1000.0f);
            last_write_count = w;
            last_status_ms = now;

            printf("[STATUS] w=%u r=%u pending=%u rate=%0.1f w/s drop_seq=%u\n",
                   (unsigned)w, (unsigned)r, (unsigned)pending, rate_wps, (unsigned)drop_seq);
        }

        /* DMA バッファから読み出してログ出力 */
        while (__atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE) < written) {
            uint32_t idx = __atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE) & LOG_BUF_MASK;
            uint32_t raw = dma_raw_buf[idx];

            __atomic_store_n(&cpu_read_count, __atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE) + 1u, __ATOMIC_RELEASE);

            uint8_t data  = (uint8_t)(raw & 0xFF);
            uint8_t flags = make_flags(raw);
            uint16_t addr = read_addr_pins();

            /* log_buf64 に格納（将来のバイナリ転送に備えて） */
            uint64_t pack = ((uint64_t)flags    << 56)
                          | ((uint64_t)drop_seq  << 48)
                          | ((uint64_t)addr      << 32)
                          | ((uint64_t)data      << 16);
            log_buf64[log_wr_idx & LOG_BUF_MASK] = pack;
            log_wr_idx++;

            /* USB Serial にテキスト出力
             * アクセス種別ラベル: IO（/IORQ）を最優先、次に RD、WR の順
             * /IORQ 監視構成では IO フラグは常に立つため、実質 IO ラベルが選ばれる */
            const char *label = (flags & (1 << 2)) ? "IO"
                               : (flags & (1 << 0)) ? "RD"
                               : (flags & (1 << 1)) ? "WR"
                               : "--";
            if (drop_latched) {
                printf("%s A=%04X D=%02X F=%02X [DROP seq=%u]\n",
                       label, addr, data, flags, (unsigned)drop_seq);
            } else {
                printf("%s A=%04X D=%02X F=%02X\n", label, addr, data, flags);
            }
        }

        /* 'c' キーで DROP フラグをリセット */
        int ch = getchar_timeout_us(0);
        if (ch == 'c') {
            drop_latched = false;
            drop_seq = 0;
            dma_lag = false;
            printf("[CMD] drop_latched をクリア\n");
        }

        tight_loop_contents();
    }

    return 0;
}