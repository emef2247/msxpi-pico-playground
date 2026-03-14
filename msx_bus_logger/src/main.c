#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/multicore.h"   /* multicore_launch_core1() に必要 */

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
/* 1: printfテキスト出力（デバッグ用）、0: バイナリ転送（本番用） */
#define TEXT_MODE 0

#define DMA_BLOCK_WORDS 512   /* 128 から増加: DMA IRQ 過多によるDROP防止 */
#define LOG_ENTRIES (8 * 1024)
#define LOG_BUF_MASK (LOG_ENTRIES - 1)

/* ===== LED 点滅モード ===== */
typedef enum {
    LED_OFF,        /* 消灯: DROP なし */
    LED_BLINK_SLOW, /* 0.5Hz（2000ms周期、1000msトグル間隔）: DROP あり・軽微（直近1秒の増分 1〜9） */
    LED_BLINK_FAST, /* 2Hz（500ms周期、250msトグル間隔）: DROP あり・重大（直近1秒の増分 10以上） */
} LedMode;

/* ===== グローバル変数 ===== */
PIO pio = pio0;
uint sm = 0;
int dma_chan;
dma_channel_config dma_cfg;

/* カウンタ */
volatile uint32_t dma_write_count = 0;
volatile uint32_t cpu_read_count = 0;

uint32_t dma_raw_buf[LOG_ENTRIES];

volatile uint32_t dma_wr_idx = 0;

static bool drop_latched = false;
static uint8_t drop_seq = 0;
static bool dma_lag = false;

volatile bool logger_armed = false;

/* TXリングバッファ: core0が書き込み、core1が読み出す */
#if TEXT_MODE == 0
#define TX_RING_PACKETS 1024                        /* 1024パケット × 12バイト = 12KB */
#define TX_RING_SIZE    (TX_RING_PACKETS * 12)

static uint8_t   tx_ring[TX_RING_SIZE];             /* パケットバッファ本体 */
static volatile uint32_t tx_wr = 0;                 /* core0が書き込むインデックス（パケット単位） */
static volatile uint32_t tx_rd = 0;                 /* core1が読み出すインデックス（パケット単位） */
static volatile uint32_t tx_drop_count = 0;         /* TXバッファ溢れカウント */
#endif

/* LED 状態 */
static LedMode  led_mode           = LED_OFF;
static uint32_t led_last_toggle_ms = 0;
static bool     led_state          = false;

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

    /* in_base = GPIO0 (A0): in pins, 32 でアドレス+データ+制御線を一括取り込む */
    sm_config_set_in_pins(&c, PIN_A0);
    sm_config_set_jmp_pin(&c, PIN_IORQ);

    /* 左シフト、autopush 無効（.pio が手動 push するため）、threshold=32 */
    sm_config_set_in_shift(&c, true, false, 32);

    /* GPIO0-23（A0-A15, D0-D7）を PIO 入力として初期化
     * GPIO24-26（/RD, /WR, /IORQ）は pio_gpio_init() しない
     * （enter_safe_state() の OUT+High 設定を維持するため） */
    for (int i = PIN_A0; i <= PIN_D7; i++) {
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

/* PIO raw 値からアドレスを取り出す
 * ISR[15:0] = GPIO0-15 = A0-A15 */
static inline uint16_t extract_addr(uint32_t raw) {
    return (uint16_t)(raw & 0xFFFF);
}

/* PIO raw 値からデータを取り出す
 * ISR[23:16] = GPIO16-23 = D0-D7 */
static inline uint8_t extract_data(uint32_t raw) {
    return (uint8_t)((raw >> 16) & 0xFF);
}

/* FLAGS バイトを生成する
 * ISR[24] = /RD  (GPIO24) : Low=アサート
 * ISR[25] = /WR  (GPIO25) : Low=アサート
 * ISR[26] = /IORQ(GPIO26) : Low=アサート */
static inline uint8_t make_flags(uint32_t raw) {
    uint8_t f = 0;
    if (!(raw & (1u << 24))) f |= 1 << 0; /* /RD がアサート → RD フラグ */
    if (!(raw & (1u << 25))) f |= 1 << 1; /* /WR がアサート → WR フラグ */
    if (!(raw & (1u << 26))) f |= 1 << 2; /* /IORQ がアサート → IO フラグ */
    if (drop_latched)        f |= 1 << 4; /* DROP ラッチフラグ */
    if (dma_lag)             f |= 1 << 6; /* DMA 遅延フラグ */
    return f;
}

/* LED 更新（メインループで毎回呼ぶ）
 * LED_OFF        : 消灯
 * LED_BLINK_SLOW : 1000ms ごとにトグル（0.5Hz、2000ms周期）
 * LED_BLINK_FAST : 250ms ごとにトグル（2Hz、500ms周期） */
static void update_led(uint32_t now_ms_val) {
    if (led_mode == LED_OFF) {
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        led_state = false;
        return;
    }
    /* 点滅周期の半分をトグル間隔とする */
    uint32_t half_period_ms = (led_mode == LED_BLINK_FAST) ? 250u : 1000u;
    if (now_ms_val - led_last_toggle_ms >= half_period_ms) {
        led_state = !led_state;
        gpio_put(PICO_DEFAULT_LED_PIN, led_state ? 1 : 0);
        led_last_toggle_ms = now_ms_val;
    }
}

/* ===== TXリングバッファへの書き込みとcore1 USB送信（TEXT_MODE=0 時のみ）===== */
#if TEXT_MODE == 0

/* core0: パケット組み立てとTXリングバッファへの書き込み
 *
 * パケットフォーマット:
 *   Byte 0   : マジックバイト1 (0xAA)
 *   Byte 1   : マジックバイト2 (0x55)
 *   Byte 2   : FLAGS
 *   Byte 3   : DROP_SEQ
 *   Byte 4-5 : ADDR (little-endian, A0-A15)
 *   Byte 6   : DATA (D0-D7)
 *   Byte 7   : 予約 (0x00)
 *   Byte 8-11: タイムスタンプ (uint32_t, little-endian, μs, time_us_32() の値)
 */
static inline void enqueue_packet(uint16_t addr, uint8_t data, uint8_t flags, uint8_t drop_seq_val) {
    uint32_t wr = __atomic_load_n(&tx_wr, __ATOMIC_ACQUIRE);
    uint32_t rd = __atomic_load_n(&tx_rd, __ATOMIC_ACQUIRE);

    if (wr - rd >= TX_RING_PACKETS) {
        /* TXバッファ溢れ: DROP としてカウントしてスキップ */
        __atomic_fetch_add(&tx_drop_count, 1u, __ATOMIC_RELAXED);
        return;
    }

    uint32_t ts = time_us_32();
    uint32_t idx = wr & (TX_RING_PACKETS - 1u);  /* TX_RING_PACKETS は2のべき乗 */
    uint8_t *pkt = &tx_ring[idx * 12];

    pkt[0]  = 0xAA;                    /* マジックバイト1 */
    pkt[1]  = 0x55;                    /* マジックバイト2 */
    pkt[2]  = flags;                   /* FLAGS */
    pkt[3]  = drop_seq_val;            /* DROP_SEQ */
    pkt[4]  = (uint8_t)(addr & 0xFF);  /* ADDR 下位バイト */
    pkt[5]  = (uint8_t)(addr >> 8);    /* ADDR 上位バイト */
    pkt[6]  = data;                    /* DATA */
    pkt[7]  = 0x00;                    /* 予約 */
    pkt[8]  = (uint8_t)(ts        );   /* タイムスタンプ byte0 */
    pkt[9]  = (uint8_t)(ts >>  8  );   /* タイムスタンプ byte1 */
    pkt[10] = (uint8_t)(ts >> 16  );   /* タイムスタンプ byte2 */
    pkt[11] = (uint8_t)(ts >> 24  );   /* タイムスタンプ byte3 */

    __atomic_store_n(&tx_wr, wr + 1u, __ATOMIC_RELEASE);
}

/* core1メイン: TXリングバッファからパケットをUSB送信する */
void core1_usb_sender(void) {
    while (1) {
        uint32_t rd = __atomic_load_n(&tx_rd, __ATOMIC_ACQUIRE);
        uint32_t wr = __atomic_load_n(&tx_wr, __ATOMIC_ACQUIRE);

        if (rd == wr) {
            /* 送信待ちなし: tight_loop_contents() でビジーウェイト
             * （USB送信は割り込みではなくポーリングのため WFI は使用しない） */
            tight_loop_contents();
            continue;
        }

        /* TXリングバッファからパケットを取り出す */
        uint32_t idx = rd & (TX_RING_PACKETS - 1u);  /* TX_RING_PACKETS は2のべき乗 */
        uint8_t *pkt = &tx_ring[idx * 12];

        if (stdio_usb_connected()) {
            /* USB接続中: 12バイト一括送信 */
            fwrite(pkt, 1, 12, stdout);
            fflush(stdout);
        }
        /* 未接続時もインデックスを進めてバッファを解放する（読み捨て） */

        __atomic_store_n(&tx_rd, rd + 1u, __ATOMIC_RELEASE);
    }
}

#endif

/* ===== メイン ===== */
int main(void) {
    stdio_init_all();
    sleep_ms(250);
#if TEXT_MODE == 1
    printf("msx_bus_logger: 起動（セーフモード）\n");
#endif
    enter_safe_state();

    pio_init_logger();
#if TEXT_MODE == 1
    printf("[SETUP] pio_init_logger() 完了\n");
#endif

    dma_init_logger_with_ring_enabled(pio, sm);
#if TEXT_MODE == 1
    printf("[SETUP] dma_init_logger_with_ring_enabled() 完了\n");
#endif

    __atomic_store_n(&cpu_read_count, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&dma_write_count, 0u, __ATOMIC_RELEASE);
    drop_seq = 0;
    dma_lag = false;
    drop_latched = false;
    logger_armed = false;

    __atomic_store_n(&cpu_read_count, __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE), __ATOMIC_RELEASE);

#if TEXT_MODE == 0
    /* core1 を USB送信ループとして起動 */
    multicore_launch_core1(core1_usb_sender);
#endif

    logger_armed = true;
#if TEXT_MODE == 1
    printf("[START] メインループ開始\n");
#endif

    const uint32_t status_interval_ms = 1000;
    uint32_t last_status_ms = now_ms();
    uint32_t last_write_count = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE);
    uint8_t  last_drop_seq   = drop_seq;

    while (1) {
        uint32_t now = now_ms();

        /* LED 更新（毎ループ） */
        update_led(now);

        /* 1秒ごとのステータス出力 */
        if (now - last_status_ms >= status_interval_ms) {
            uint32_t w = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE);
            uint32_t r = __atomic_load_n(&cpu_read_count,  __ATOMIC_ACQUIRE);
            uint32_t pending = (w >= r) ? (w - r) : 0;
            uint32_t delta = w - last_write_count;
            float rate_wps = (float)delta / ((now - last_status_ms) / 1000.0f);

            /* drop_seq の直近1秒間の増分で LED モードを決定 */
            uint8_t delta_drop = (uint8_t)(drop_seq - last_drop_seq);
            last_drop_seq = drop_seq;

            if (delta_drop == 0) {
                led_mode = LED_OFF;        /* 直近1秒でDROPなし → 完全消灯 */
            } else if (delta_drop >= 10) {
                led_mode = LED_BLINK_FAST;
            } else {
                led_mode = LED_BLINK_SLOW;
            }

#if TEXT_MODE == 1
            printf("[STATUS] w=%u r=%u pending=%u rate=%.1f w/s drop_seq=%u led=%s\n",
                   (unsigned)w, (unsigned)r, (unsigned)pending, rate_wps, (unsigned)drop_seq,
                   led_mode == LED_OFF ? "OFF" : led_mode == LED_BLINK_SLOW ? "SLOW" : "FAST");
#endif
            last_write_count = w;
            last_status_ms   = now;
        }

        /* DMA バッファから読み出してログ出力 */
        uint32_t written = __atomic_load_n(&dma_write_count, __ATOMIC_ACQUIRE);
        while (__atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE) < written) {
            uint32_t idx = __atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE) & LOG_BUF_MASK;
            uint32_t raw = dma_raw_buf[idx];

            __atomic_store_n(&cpu_read_count,
                             __atomic_load_n(&cpu_read_count, __ATOMIC_ACQUIRE) + 1u,
                             __ATOMIC_RELEASE);

            uint16_t addr  = extract_addr(raw);   /* PIO サンプル値からアドレスを正確に取得 */
            uint8_t  data  = extract_data(raw);
            uint8_t  flags = make_flags(raw);

#if TEXT_MODE == 1
            /* テキストモード: printf でイベントを出力
             * アクセス種別ラベル: IO（/IORQ）を最優先、次に RD、WR の順 */
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
#else
            /* バイナリモード: TXリングバッファにパケットをエンキュー */
            enqueue_packet(addr, data, flags, drop_seq);
#endif
        }

        /* 'c' キーで DROP フラグをリセット（テキストモード時のみ） */
#if TEXT_MODE == 1
        int ch = getchar_timeout_us(0);
        if (ch == 'c') {
            drop_latched = false;
            drop_seq     = 0;
            dma_lag      = false;
            last_drop_seq = 0;
            led_mode     = LED_OFF;
            printf("[CMD] drop_latched をクリア\n");
        }
#endif

        tight_loop_contents();
    }

    return 0;
}