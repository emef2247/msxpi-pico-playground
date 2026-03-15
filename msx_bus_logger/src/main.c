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

/* パケット種別 */
#define PKT_TYPE_STATUS  0x01
#define PKT_TYPE_EVENT   0x02

/* SM_ID */
#define SM_ID_WR    0x00  /* SM0: /WR トリガー */
#define SM_ID_IORQ  0x01  /* SM1: /IORQ トリガー */

/* パケットサイズ（STATUS/EVENT 共通、12バイト固定） */
#define PKT_SIZE        12
#define TX_RING_PACKETS 1024
#define TX_RING_SIZE    (TX_RING_PACKETS * PKT_SIZE)

/* ===== LED 点滅モード ===== */
typedef enum {
    LED_OFF,        /* 消灯: DROP なし */
    LED_BLINK_SLOW, /* 0.5Hz（2000ms周期、1000msトグル間隔）: DROP あり・軽微（直近1秒の増分 1〜9） */
    LED_BLINK_FAST, /* 2Hz（500ms周期、250msトグル間隔）: DROP あり・重大（直近1秒の増分 10以上） */
} LedMode;

/* ===== グローバル変数 ===== */
PIO pio = pio0;
uint sm0 = 0;   /* /WR トリガー */
uint sm1 = 1;   /* /IORQ トリガー */

/* DMA チャンネル */
int dma_chan0;   /* SM0用 */
int dma_chan1;   /* SM1用 */

/* リングバッファ */
uint32_t dma_raw_buf0[LOG_ENTRIES];  /* SM0用 */
uint32_t dma_raw_buf1[LOG_ENTRIES];  /* SM1用 */

/* 書き込みインデックス（DMA ISR で更新） */
volatile uint32_t dma_wr_idx0 = 0;
volatile uint32_t dma_wr_idx1 = 0;

/* 書き込み・読み出しカウンタ（SM別） */
volatile uint32_t dma_write_count0 = 0;
volatile uint32_t dma_write_count1 = 0;
volatile uint32_t cpu_read_count0 = 0;
volatile uint32_t cpu_read_count1 = 0;

/* DROP 状態（SM別） */
static bool     drop_latched0 = false;
static uint8_t  drop_seq0     = 0;
static bool     drop_latched1 = false;
static uint8_t  drop_seq1     = 0;

volatile bool logger_armed = false;

/* TXリングバッファ: core0が書き込み、core1が読み出す（SM0/SM1共有） */
#if TEXT_MODE == 0
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
 * ISR[24] = /RD    (GPIO24) : Low=アサート
 * ISR[25] = /WR    (GPIO25) : Low=アサート
 * ISR[26] = /IORQ  (GPIO26) : Low=アサート
 * ISR[27] = /SLTSL (GPIO27) : Low=アサート
 * ISR[28] = /WAIT  (GPIO28) : Low=アサート */
static inline uint8_t make_flags(uint32_t raw) {
    uint8_t f = 0;
    if (!(raw & (1u << 24))) f |= 1 << 0; /* /RD がアサート → RD フラグ */
    if (!(raw & (1u << 25))) f |= 1 << 1; /* /WR がアサート → WR フラグ */
    if (!(raw & (1u << 26))) f |= 1 << 2; /* /IORQ がアサート → IO フラグ */
    if (!(raw & (1u << 27))) f |= 1 << 3; /* /SLTSL がアサート → SLTSL フラグ */
    if (!(raw & (1u << 28))) f |= 1 << 4; /* /WAIT がアサート → WAIT フラグ */
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

/* STATUSパケットをエンキュー（ISRから呼ぶ）
 *
 * パケットフォーマット（12バイト固定）:
 *   Byte 0   : 0xAA
 *   Byte 1   : 0x55
 *   Byte 2   : 0x01 (TYPE=STATUS)
 *   Byte 3   : SM_ID  (0x00=SM0/WR, 0x01=SM1/IORQ)
 *   Byte 4   : DROP_LATCH
 *   Byte 5   : DROP_SEQ
 *   Byte 6-9 : TIMESTAMP (uint32_t, μs, little-endian)
 *   Byte 10-11: PENDING (uint16_t, little-endian)
 */
static inline void enqueue_status_packet(uint8_t sm_id,
                                          uint8_t drop_latch,
                                          uint8_t drop_seq_val,
                                          uint32_t ts,
                                          uint16_t pending) {
    uint32_t wr = __atomic_load_n(&tx_wr, __ATOMIC_ACQUIRE);
    uint32_t rd = __atomic_load_n(&tx_rd, __ATOMIC_ACQUIRE);

    if (wr - rd >= TX_RING_PACKETS) {
        __atomic_fetch_add(&tx_drop_count, 1u, __ATOMIC_RELAXED);
        return;
    }

    uint32_t idx = wr & (TX_RING_PACKETS - 1u);
    uint8_t *pkt = &tx_ring[idx * PKT_SIZE];

    pkt[0]  = 0xAA;
    pkt[1]  = 0x55;
    pkt[2]  = PKT_TYPE_STATUS;
    pkt[3]  = sm_id;
    pkt[4]  = drop_latch;
    pkt[5]  = drop_seq_val;
    pkt[6]  = (uint8_t)(ts       );
    pkt[7]  = (uint8_t)(ts >>  8 );
    pkt[8]  = (uint8_t)(ts >> 16 );
    pkt[9]  = (uint8_t)(ts >> 24 );
    pkt[10] = (uint8_t)(pending      );
    pkt[11] = (uint8_t)(pending >> 8 );

    __atomic_store_n(&tx_wr, wr + 1u, __ATOMIC_RELEASE);
}

/* EVENTパケットをエンキュー（core0メインループから呼ぶ）
 *
 * パケットフォーマット（12バイト固定）:
 *   Byte 0   : 0xAA
 *   Byte 1   : 0x55
 *   Byte 2   : 0x02 (TYPE=EVENT)
 *   Byte 3   : SM_ID  (0x00=SM0/WR, 0x01=SM1/IORQ)
 *   Byte 4   : FLAGS
 *   Byte 5-6 : ADDR (uint16_t, little-endian)
 *   Byte 7   : DATA
 *   Byte 8-11: 予約 (0x00 × 4バイト)
 */
static inline void enqueue_event_packet(uint8_t sm_id,
                                         uint16_t addr,
                                         uint8_t data,
                                         uint8_t flags) {
    uint32_t wr = __atomic_load_n(&tx_wr, __ATOMIC_ACQUIRE);
    uint32_t rd = __atomic_load_n(&tx_rd, __ATOMIC_ACQUIRE);

    if (wr - rd >= TX_RING_PACKETS) {
        __atomic_fetch_add(&tx_drop_count, 1u, __ATOMIC_RELAXED);
        return;
    }

    uint32_t idx = wr & (TX_RING_PACKETS - 1u);
    uint8_t *pkt = &tx_ring[idx * PKT_SIZE];

    pkt[0]  = 0xAA;
    pkt[1]  = 0x55;
    pkt[2]  = PKT_TYPE_EVENT;
    pkt[3]  = sm_id;
    pkt[4]  = flags;
    pkt[5]  = (uint8_t)(addr & 0xFF);
    pkt[6]  = (uint8_t)(addr >> 8);
    pkt[7]  = data;
    pkt[8]  = 0x00;
    pkt[9]  = 0x00;
    pkt[10] = 0x00;
    pkt[11] = 0x00;

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
        uint32_t idx = rd & (TX_RING_PACKETS - 1u);
        uint8_t *pkt = &tx_ring[idx * PKT_SIZE];

        if (stdio_usb_connected()) {
            /* USB接続中: PKT_SIZE バイト一括送信 */
            fwrite(pkt, 1, PKT_SIZE, stdout);
            fflush(stdout);
        }
        /* 未接続時もインデックスを進めてバッファを解放する（読み捨て） */

        __atomic_store_n(&tx_rd, rd + 1u, __ATOMIC_RELEASE);
    }
}

#endif

/* ===== PIO 初期化 ===== */
void pio_init_logger(void) {
    /* SM0: /WR トリガー */
    uint offset0 = pio_add_program(pio, &msx_bus_logger_wr_program);
    pio_sm_config c0 = msx_bus_logger_wr_program_get_default_config(offset0);
    sm_config_set_in_pins(&c0, PIN_A0);
    sm_config_set_jmp_pin(&c0, PIN_WR);
    /* 右シフト、autopush 無効（.pio が手動 push するため）、threshold=32 */
    sm_config_set_in_shift(&c0, true, false, 32);
    /* GPIO0-23（A0-A15, D0-D7）を PIO 入力として初期化
     * GPIO24-28（/RD, /WR, /IORQ, /SLTSL, /WAIT）は pio_gpio_init() しない
     * （enter_safe_state() の OUT+High 設定を維持するため） */
    for (int i = PIN_A0; i <= PIN_D7; i++) {
        pio_gpio_init(pio, i);
        pio_sm_set_consecutive_pindirs(pio, sm0, i, 1, false);
    }
    pio_sm_init(pio, sm0, offset0, &c0);
    pio_sm_set_enabled(pio, sm0, true);

    /* SM1: /IORQ トリガー */
    uint offset1 = pio_add_program(pio, &msx_bus_logger_iorq_program);
    pio_sm_config c1 = msx_bus_logger_iorq_program_get_default_config(offset1);
    sm_config_set_in_pins(&c1, PIN_A0);
    sm_config_set_jmp_pin(&c1, PIN_IORQ);
    sm_config_set_in_shift(&c1, true, false, 32);
    /* GPIO0-23 は SM0 で初期化済みのため再初期化不要 */
    pio_sm_set_consecutive_pindirs(pio, sm1, PIN_A0, 24, false);
    pio_sm_init(pio, sm1, offset1, &c1);
    pio_sm_set_enabled(pio, sm1, true);

    /* オンボードLED初期化 */
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

/* ===== DMA 割り込みハンドラ ===== */

/* SM0用 DMA割り込み (DMA_IRQ_0) */
void __isr dma_handler0(void) {
    dma_hw->ints0 = 1u << dma_chan0;

    dma_wr_idx0 += DMA_BLOCK_WORDS;
    dma_write_count0 = dma_wr_idx0;

    dma_channel_set_read_addr(dma_chan0, &pio->rxf[sm0], false);
    dma_channel_set_write_addr(dma_chan0, &dma_raw_buf0[dma_wr_idx0 & LOG_BUF_MASK], true);

    if (!logger_armed) return;

    uint32_t pending = dma_write_count0 - cpu_read_count0;
    if (pending > LOG_ENTRIES) {
        drop_latched0 = true;
        drop_seq0++;
    }

#if TEXT_MODE == 0
    uint32_t ts = time_us_32();
    uint16_t pend16 = (pending > 0xFFFF) ? 0xFFFF : (uint16_t)pending;
    enqueue_status_packet(SM_ID_WR, drop_latched0 ? 1 : 0, drop_seq0, ts, pend16);
#endif
}

/* SM1用 DMA割り込み (DMA_IRQ_1) */
void __isr dma_handler1(void) {
    dma_hw->ints1 = 1u << dma_chan1;

    dma_wr_idx1 += DMA_BLOCK_WORDS;
    dma_write_count1 = dma_wr_idx1;

    dma_channel_set_read_addr(dma_chan1, &pio->rxf[sm1], false);
    dma_channel_set_write_addr(dma_chan1, &dma_raw_buf1[dma_wr_idx1 & LOG_BUF_MASK], true);

    if (!logger_armed) return;

    uint32_t pending = dma_write_count1 - cpu_read_count1;
    if (pending > LOG_ENTRIES) {
        drop_latched1 = true;
        drop_seq1++;
    }

#if TEXT_MODE == 0
    uint32_t ts = time_us_32();
    uint16_t pend16 = (pending > 0xFFFF) ? 0xFFFF : (uint16_t)pending;
    enqueue_status_packet(SM_ID_IORQ, drop_latched1 ? 1 : 0, drop_seq1, ts, pend16);
#endif
}

/* ===== DMA 初期化 ===== */
void dma_init_logger(void) {
    /* SM0: DMA_IRQ_0 を使用 */
    dma_chan0 = dma_claim_unused_channel(true);
    dma_channel_config cfg0 = dma_channel_get_default_config(dma_chan0);
    channel_config_set_transfer_data_size(&cfg0, DMA_SIZE_32);
    channel_config_set_dreq(&cfg0, pio_get_dreq(pio, sm0, false));
    channel_config_set_read_increment(&cfg0, false);
    channel_config_set_write_increment(&cfg0, true);
    channel_config_set_chain_to(&cfg0, dma_chan0); /* 自身へのチェーン（チェーン無効） */
    dma_channel_configure(dma_chan0, &cfg0,
                          dma_raw_buf0, &pio->rxf[sm0],
                          DMA_BLOCK_WORDS, false);
    dma_channel_set_irq0_enabled(dma_chan0, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler0);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_start_channel_mask(1u << dma_chan0);

    /* SM1: DMA_IRQ_1 を使用 */
    dma_chan1 = dma_claim_unused_channel(true);
    dma_channel_config cfg1 = dma_channel_get_default_config(dma_chan1);
    channel_config_set_transfer_data_size(&cfg1, DMA_SIZE_32);
    channel_config_set_dreq(&cfg1, pio_get_dreq(pio, sm1, false));
    channel_config_set_read_increment(&cfg1, false);
    channel_config_set_write_increment(&cfg1, true);
    channel_config_set_chain_to(&cfg1, dma_chan1);
    dma_channel_configure(dma_chan1, &cfg1,
                          dma_raw_buf1, &pio->rxf[sm1],
                          DMA_BLOCK_WORDS, false);
    dma_channel_set_irq1_enabled(dma_chan1, true);
    irq_set_exclusive_handler(DMA_IRQ_1, dma_handler1);
    irq_set_enabled(DMA_IRQ_1, true);
    dma_start_channel_mask(1u << dma_chan1);
}

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

    dma_init_logger();
#if TEXT_MODE == 1
    printf("[SETUP] dma_init_logger() 完了\n");
#endif

    __atomic_store_n(&cpu_read_count0, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&cpu_read_count1, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&dma_write_count0, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&dma_write_count1, 0u, __ATOMIC_RELEASE);
    drop_seq0 = 0;
    drop_seq1 = 0;
    drop_latched0 = false;
    drop_latched1 = false;
    logger_armed = false;

    /* DMA が既に書き込んだ分をスキップしてから開始 */
    __atomic_store_n(&cpu_read_count0,
                     __atomic_load_n(&dma_write_count0, __ATOMIC_ACQUIRE),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&cpu_read_count1,
                     __atomic_load_n(&dma_write_count1, __ATOMIC_ACQUIRE),
                     __ATOMIC_RELEASE);

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
    uint8_t  last_drop_seq0 = drop_seq0;
    uint8_t  last_drop_seq1 = drop_seq1;

    while (1) {
        uint32_t now = now_ms();

        /* LED 更新（毎ループ） */
        update_led(now);

        /* 1秒ごとのステータス出力 */
        if (now - last_status_ms >= status_interval_ms) {
            /* drop_seq の直近1秒間の増分で LED モードを決定 */
            uint8_t delta_drop0 = (uint8_t)(drop_seq0 - last_drop_seq0);
            uint8_t delta_drop1 = (uint8_t)(drop_seq1 - last_drop_seq1);
            uint16_t delta_drop = (uint16_t)delta_drop0 + (uint16_t)delta_drop1;
            last_drop_seq0 = drop_seq0;
            last_drop_seq1 = drop_seq1;

            if (delta_drop == 0) {
                led_mode = LED_OFF;
            } else if (delta_drop >= 10) {
                led_mode = LED_BLINK_FAST;
            } else {
                led_mode = LED_BLINK_SLOW;
            }

#if TEXT_MODE == 1
            {
                uint32_t w0 = __atomic_load_n(&dma_write_count0, __ATOMIC_ACQUIRE);
                uint32_t r0 = __atomic_load_n(&cpu_read_count0,  __ATOMIC_ACQUIRE);
                uint32_t w1 = __atomic_load_n(&dma_write_count1, __ATOMIC_ACQUIRE);
                uint32_t r1 = __atomic_load_n(&cpu_read_count1,  __ATOMIC_ACQUIRE);
                uint32_t pending0 = (w0 >= r0) ? (w0 - r0) : 0;
                uint32_t pending1 = (w1 >= r1) ? (w1 - r1) : 0;
                printf("[STATUS SM0] w=%u r=%u pending=%u drop_seq=%u\n",
                       (unsigned)w0, (unsigned)r0, (unsigned)pending0, (unsigned)drop_seq0);
                printf("[STATUS SM1] w=%u r=%u pending=%u drop_seq=%u\n",
                       (unsigned)w1, (unsigned)r1, (unsigned)pending1, (unsigned)drop_seq1);
            }
#endif
            last_status_ms = now;
        }

        /* SM0バッファ読み出し */
        uint32_t written0 = __atomic_load_n(&dma_write_count0, __ATOMIC_ACQUIRE);
        while (__atomic_load_n(&cpu_read_count0, __ATOMIC_ACQUIRE) < written0) {
            uint32_t idx = __atomic_load_n(&cpu_read_count0, __ATOMIC_ACQUIRE) & LOG_BUF_MASK;
            uint32_t raw = dma_raw_buf0[idx];
            __atomic_fetch_add(&cpu_read_count0, 1u, __ATOMIC_RELEASE);
#if TEXT_MODE == 0
            enqueue_event_packet(SM_ID_WR,
                                 extract_addr(raw),
                                 extract_data(raw),
                                 make_flags(raw));
#endif
        }

        /* SM1バッファ読み出し */
        uint32_t written1 = __atomic_load_n(&dma_write_count1, __ATOMIC_ACQUIRE);
        while (__atomic_load_n(&cpu_read_count1, __ATOMIC_ACQUIRE) < written1) {
            uint32_t idx = __atomic_load_n(&cpu_read_count1, __ATOMIC_ACQUIRE) & LOG_BUF_MASK;
            uint32_t raw = dma_raw_buf1[idx];
            __atomic_fetch_add(&cpu_read_count1, 1u, __ATOMIC_RELEASE);
#if TEXT_MODE == 0
            enqueue_event_packet(SM_ID_IORQ,
                                 extract_addr(raw),
                                 extract_data(raw),
                                 make_flags(raw));
#endif
        }

        /* 'c' キーで DROP フラグをリセット（テキストモード時のみ） */
#if TEXT_MODE == 1
        int ch = getchar_timeout_us(0);
        if (ch == 'c') {
            drop_latched0 = false;
            drop_seq0     = 0;
            drop_latched1 = false;
            drop_seq1     = 0;
            last_drop_seq0 = 0;
            last_drop_seq1 = 0;
            led_mode      = LED_OFF;
            printf("[CMD] drop_latched をクリア\n");
        }
#endif

        tight_loop_contents();
    }

    return 0;
}
