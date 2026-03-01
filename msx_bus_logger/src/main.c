#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "msx_bus_logger.pio.h"

/* ===== ピン定義（配線に合わせて変更） ===== */
#define PIN_A0 1 //A0_3V3 
#define PIN_A1 2 
#define PIN_A2 3 
#define PIN_A3 4 
#define PIN_A4 5 
#define PIN_A5 7 
#define PIN_A6 8 
#define PIN_A7 9 
#define PIN_A8 10 //A8_3V3 
#define PIN_A9 11 
#define PIN_A10 12 
#define PIN_A11 13 
#define PIN_A12 14 
#define PIN_A13 16 
#define PIN_A14 17 
#define PIN_A15 18 
#define PIN_D0 19 
#define PIN_D1 20 
#define PIN_D2 21 
#define PIN_D3 22 
#define PIN_D4 23 
#define PIN_D5 24 
#define PIN_D6 25 
#define PIN_D7 26 
#define PIN_RD 27 
#define PIN_WR 28 
#define PIN_IORQ 30
#define PIN_SLTSL 31
#define PIN_WAIT 32 
#define PIN_BUSDIR 33
#define PIN_RESET 34

#define PIN_GPIO_6 6
#define PIN_GPIO_15 15
#define PIN_GPIO_29 29

/* リングバッファ */
#define DMA_BLOCK_WORDS 512 // DMA がIRQを出すまでに行うPIO->DMA間の転送回数
#define LOG_ENTRIES (8 * 1024)   // 8K entries
#define LOG_BUF_MASK (LOG_ENTRIES - 1)

/* --- 起動安定化 / 初期破棄の設定 --- */
#define POWER_STABLE_MS      300   // /RESET が連続して High であることを確認する時間（ms）
#define INIT_DISCARD_BLOCKS  2     // 起動後に破棄する DMA ブロック数（調整可）

/* ===== グローバル変数 ===== */
PIO pio = pio0;
uint sm = 0;
int dma_chan;
dma_channel_config dma_cfg;
volatile bool dma_irq_seen = false;
// 追加フラグ（ISR -> main 通信用)
volatile bool drop_event_seen = false;   // ISR が DROP を検出した瞬間のイベント（クリアは main）
volatile bool severe_pending = false;    // 必要なら ISR で立てる

/* ===== DROP 及び カウンタ ===== */
volatile uint32_t dma_write_count = 0;      // DMA 論理総パケット数
volatile uint32_t cpu_read_count = 0;      // CPU 読み込みカウンタ

volatile bool dma_raw_drop = false;
volatile uint32_t dma_raw_drop_count = 0;

volatile bool cpu_log_drop = false;
volatile uint32_t cpu_log_drop_count = 0;


/* DMA 生ログ（PIO→DMA） */
uint32_t dma_raw_buf[LOG_ENTRIES];

/* 64bit 完成ログ */
uint64_t log_buf64[LOG_ENTRIES];

volatile uint32_t dma_wr_idx = 0;
volatile uint32_t log_wr_idx = 0;

/* DROP検出 */
static bool drop_latched = false;
static uint8_t drop_seq = 0;
static bool fifo_overflow = false;
static bool dma_lag = false;
static bool dma_lag_prev = false;

volatile bool logger_armed = false;

// Timestamp captured at DMA IRQ.
// Used by main loop without calling time functions (performance critical).
volatile uint32_t dma_irq_time_us;


/* ===== ユーティリティ ===== */
static inline uint32_t now_us(void) {
    return to_us_since_boot(get_absolute_time());
}

/* MSX 電源検出：/RESET が High なら ON とみなす */
static inline bool msx_power_on(void) {
    return gpio_get(PIN_RESET);
}

/* ===== 安全状態 =====
 * MSX が OFF / 不明なときに絶対にバスを触らない
 */
void enter_safe_state(void) {
	gpio_init(PIN_RD);
    gpio_set_dir(PIN_RD, GPIO_OUT);
    gpio_put(PIN_RD, 1);

	gpio_init(PIN_WR);
    gpio_set_dir(PIN_WR, GPIO_OUT);
    gpio_put(PIN_WR, 1);
	
	gpio_init(PIN_WAIT);
	gpio_set_dir(PIN_WAIT, GPIO_OUT);
	gpio_put(PIN_WAIT, 1);
}

/* /RESET が連続して stable_ms ミリ秒間 High であることを確認する */
static void wait_msx_power_stable(uint32_t stable_ms) {
    const uint32_t check_interval_ms = 10;
    uint32_t required_counts = (stable_ms + check_interval_ms - 1) / check_interval_ms;
    uint32_t good_counts = 0;

    while (good_counts < required_counts) {
        if (gpio_get(PIN_RESET)) {
            good_counts++;
        } else {
            good_counts = 0;
        }
        sleep_ms(check_interval_ms);
    }
}

// pio_init_logger の安全実装（訂正版ピン配に合わせる）
void pio_init_logger(void) {
    uint offset = pio_add_program(pio, &msx_bus_logger_program);
    pio_sm_config c = msx_bus_logger_program_get_default_config(offset);

    // base = PIN_D0 (GPIO19), jmp pin = PIN_IORQ (GPIO30)
    sm_config_set_in_pins(&c, PIN_D0);
    sm_config_set_jmp_pin(&c, PIN_IORQ);

    // in_shift autopush off (we use PIO push block)
    sm_config_set_in_shift(&c, false, false, 32);

    // Initialize only the pins we actually need (explicit list)
    int data_pins[] = {PIN_D0,PIN_D1,PIN_D2,PIN_D3,PIN_D4,PIN_D5,PIN_D6,PIN_D7};
    for (int i = 0; i < 8; ++i) {
        int p = data_pins[i];
		
		// USB D+/D- を触らない
		if (p != 24 && p != 25) {
			// make sure the SIO/PIO does not drive the pin: set GPIO to input and disable pulls
			gpio_init(p);
			gpio_set_dir(p, GPIO_IN);
			gpio_pull_up(p);
			
			// register pin with PIO and set SM pin direction to input (so PIO reads it)
			pio_gpio_init(pio, p);
			pio_sm_set_consecutive_pindirs(pio, sm, p, 1, false);
		}
    }

    // RD and WR
    gpio_init(PIN_RD); gpio_set_dir(PIN_RD, GPIO_IN); gpio_pull_up(PIN_RD);
    pio_gpio_init(pio, PIN_RD); pio_sm_set_consecutive_pindirs(pio, sm, PIN_RD, 1, false);
    gpio_init(PIN_WR); gpio_set_dir(PIN_WR, GPIO_IN); gpio_pull_up(PIN_WR);
    pio_gpio_init(pio, PIN_WR); pio_sm_set_consecutive_pindirs(pio, sm, PIN_WR, 1, false);

    // IORQ: jmp_pin; register for PIO but keep pulls controlled
    gpio_init(PIN_IORQ); gpio_set_dir(PIN_IORQ, GPIO_IN); gpio_pull_up(PIN_IORQ);
    pio_gpio_init(pio, PIN_IORQ);

    // start the state machine
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    // LED init
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void dma_init_logger(PIO pio, uint sm) {
    dma_chan = dma_claim_unused_channel(true);
    dma_cfg = dma_channel_get_default_config(dma_chan);

    // 転送サイズ：32bit
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);

    // 書き込み側はインクリメント（RAM）
    channel_config_set_write_increment(&dma_cfg, true);

    // 読み込み側は固定（PIO RX FIFO）
    channel_config_set_read_increment(&dma_cfg, false);

    // DREQ: PIO RX FIFO
    channel_config_set_dreq(&dma_cfg,
        pio_get_dreq(pio, sm, false)); // false = RX

    dma_channel_configure(
        dma_chan,
        &dma_cfg,
        dma_raw_buf,                // write addr
        &pio->rxf[sm],          // read addr
        DMA_BLOCK_WORDS,           // 転送数
        true                     // start immediately
    );
}

/* DMA ISR */
void __isr dma_handler(void) {
    // ISR の先頭でシグナル（短いパルス検出を main に伝える）
    dma_irq_seen = true;

    // Latch timestamp once per DMA block
    dma_irq_time_us = time_us_32();
    dma_hw->ints0 = 1u << dma_chan;

    // DMA カウントは常に更新して main が追従できるようにする
    dma_write_count += DMA_BLOCK_WORDS;

    // reconfigure DMA addresses
    dma_channel_set_read_addr(dma_chan, &pio->rxf[sm], false);
    dma_channel_set_write_addr(
        dma_chan,
        &dma_raw_buf[dma_wr_idx & LOG_BUF_MASK],
        true
    );

    // DROP 判定は logger_armed 時だけ（既存）
    if (!logger_armed) return;

    // DMA 遅延検出（リングバッファ基準）
    uint32_t pending = dma_write_count - cpu_read_count;
    if (pending > LOG_ENTRIES) {
        dma_lag = true;
        drop_latched = true;
        drop_seq++;

        // ISR では軽いフラグを立てるだけ（main が処理/ログ/LED を担当）
        drop_event_seen = true;
        // severe_pending は閾値により main で判定するのでも良いが、
        // ISR 側で立てたいならここで条件を追加して set してください
        // if (pending > LOG_ENTRIES * 4) {
        //     severe_pending = true;
        // }
    }
}
void dma_init_logger_with_ring_enabled(PIO pio, uint sm) {
    dma_chan = dma_claim_unused_channel(true);
    dma_cfg = dma_channel_get_default_config(dma_chan);

	// 転送サイズ：32bit
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
	
	// DREQ: PIO RX FIFO
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(pio, sm, false));// false = RX
	
	// 読み込み側は固定（PIO RX FIFO）
    channel_config_set_read_increment(&dma_cfg, false);
	
	// 書き込み側はインクリメント（RAM）
    channel_config_set_write_increment(&dma_cfg, true);
	
	// DMA リングバッファ化 (DMAは 一定サイズを永遠に周回)
    channel_config_set_chain_to(&dma_cfg, dma_chan);

    dma_channel_configure(
        dma_chan,
        &dma_cfg,
        dma_raw_buf,
        &pio->rxf[sm],
        DMA_BLOCK_WORDS,
        false
    );

    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_start_channel_mask(1u << dma_chan);
}

/* 64bit Pack Format */
// 63      56 55      48 47      32 31      16 15       8 7        0   
// +----------+----------+----------+----------+----------+----------+ 
// |  FLAGS   |  RSV     |   ADDR   |   ADDR   |   DATA   |  TIME_L  | 
// +----------+----------+----------+----------+----------+----------+ 
// DATA : D0–D7				
// ADDR : A0–A15（CPU GPIO）	
// TIME_L : 下位 8bit		
// FLAGS					
// 	bit0 : RD					
// 	bit1 : WR					
// 	bit2 : IO					
//  bit3 : EXT_SLOT         
//  bit4 : DROP_LATCHED   ;一度示したら 以後ずっと 1。ログが「完全ではない」ことを示す 大局フラグ。
//  bit5 : FIFO_OVERFLOW  ;PIO RX FIFO が詰まった兆候。瞬間的な過負荷。				
//  bit6 : DMA_LAG        ;DMA が CPU/DMA バッファに追いついていない。継続的な帯域不足。
//  bit7 : RESERVED		  ;これらbit 4,5,6は同時に立つこともある
// RSV 						
//  bit0–3 : DROP_SEQ (0–15)  DROP発生回数の下位4bit 
//  bit4–7 : RESERVED       

// Z80 bus
//   ↓
// PIO IN + push
//   ↓
// PIO RX FIFO (深さ 4)
//   ↓  (DREQ)
// DMA
//   ↓
// dma_raw_buf[] (RAM ring buffer)
//   ↓
// CPU (main loop)

/* ADDR（A0–A15）取得関数 */
static const int addr_pins[16] = {
    PIN_A0, PIN_A1, PIN_A2, PIN_A3,
    PIN_A4, PIN_A5, PIN_A6, PIN_A7,
    PIN_A8, PIN_A9, PIN_A10, PIN_A11,
    PIN_A12, PIN_A13, PIN_A14, PIN_A15
};

static inline uint16_t read_addr_pins(void) {
    uint16_t addr = 0;
    for (int i = 0; i < 16; i++) {
        addr |= (gpio_get(addr_pins[i]) << i);
    }
    return addr;
}

/* FLAGS 合成関数 */
static inline uint8_t make_flags(uint32_t pio_v) {
    uint8_t f = 0;

    /* PIO サンプル由来: bit0..7 = D0..D7, bit8 = RD, bit9 = WR */
    if (!(pio_v & (1u << 8)))  f |= 1 << 0; // RD (active-low)
    if (!(pio_v & (1u << 9)))  f |= 1 << 1; // WR (active-low)

    /* IORQ は PIO サンプルに入っていないので GPIO 直読み */
    if (!gpio_get(PIN_IORQ))  f |= 1 << 2; // IO (/IORQ low -> asserted)

    /* GPIO 直読み */
    if (gpio_get(PIN_SLTSL))   f |= 1 << 3; // EXT SLOT

    /* DROP 系（状態フラグ） */
    if (drop_latched)          f |= 1 << 4;
    if (fifo_overflow)         f |= 1 << 5;
    if (dma_lag)               f |= 1 << 6;

    return f;
}

/* ===== メイン ===== */

int main(void) {
    stdio_init_all();
	
	printf("stdio_init_all() pass !\n");

    /* 1) 起動直後にRDとWRをOUTPUTにし、1出力*/
    enter_safe_state();

	/* IOを切り替えるのはMSXの起動ロゴし、スロットのスキャン処理完了まで十分待つ（例 100000ms） */
	sleep_ms(10000);

	/* 4) MSX が起きて安定したと判定したら PIO と DMA を起動 */
	pio_init_logger();
	printf("\n[SETUP] pio_init_logger() done. \n");

	/* 5) DMA 起動（PIO -> DMA の順） */
	dma_init_logger_with_ring_enabled(pio, sm);
	printf("\n[SETUP] dma_init_logger_with_ring_enabled() done. \n");


	/* 初期不安定ブロック破棄: logger_armed は false のまま */
	cpu_read_count = 0;
	dma_write_count = 0;
	drop_seq = 0;
	dma_lag = false;
	drop_latched = false;
	logger_armed = false;

	/* DISCARD: DMA が書き込む数が期待分を超えるまで待つ */
	const uint32_t expected_initial = DMA_BLOCK_WORDS * INIT_DISCARD_BLOCKS;
	while (dma_write_count < expected_initial) {
		tight_loop_contents();
	}
	/* 破棄したことに合わせて read ポインタを追従 */
	cpu_read_count = dma_write_count;
	log_wr_idx = dma_write_count & LOG_BUF_MASK;

	uint32_t last_status_time = to_ms_since_boot(get_absolute_time());
	uint32_t status_interval_ms = 1000; // 1s

	/* 本稼働開始 */
	logger_armed = true;

	printf("\n[START] MAIN LOOP START\n");
	/* 7) メインループ*/
	while (1) {
		uint32_t written = dma_write_count;
		// LED 管理用 state（static でループ内に置いておく）
		static uint32_t led_until_us = 0;
		static bool led_on = false;

		// 1) DMA IRQ 短パルス表示（dma_irq_seen は ISR が true にする）
		if (dma_irq_seen) {
			dma_irq_seen = false;
			gpio_put(PICO_DEFAULT_LED_PIN, 1);
			led_on = true;
			led_until_us = now_us() + 10000; // 10 ms
		}

		// 2) drop_event_seen が立っていたら短い強調表示 & CDC 出力
		if (drop_event_seen) {
			drop_event_seen = false;
			// 強調短点滅（200ms 視認可能）
			gpio_put(PICO_DEFAULT_LED_PIN, 1);
			led_on = true;
			led_until_us = now_us() + 200;

			// CDC にイベント詳細ダンプ（軽量）
			uint32_t pending = dma_write_count - cpu_read_count;
			printf("[EVENT] DROP detected: drop_seq=%u pending=%u dma_lag=%d",
				   (unsigned)drop_seq, (unsigned)pending, dma_lag);

			// 直近 N サンプルのダンプ（解析用）
			const int N = 8;
			int start = (cpu_read_count - N) & LOG_BUF_MASK;
			printf("Recent raw samples:");
			for (int i = 0; i < N; ++i) {
				uint32_t s = dma_raw_buf[(start + i) & LOG_BUF_MASK];
				printf(" %08x", s);
			}
			printf("\n");
		}

		// 3) drop_latched が true なら常時点灯（ラッチ表示）
		if (drop_latched) {
			gpio_put(PICO_DEFAULT_LED_PIN, 1);
			led_on = true;
			// ラッチはユーザ操作でクリアする（下で USB 'c' をチェック）
		} else {
			// 通常時は短点滅管理（非ブロッキング）
			if (led_on && now_us() > led_until_us) {
				gpio_put(PICO_DEFAULT_LED_PIN, 0);
				led_on = false;
			}
		}

		// USB キー入力で drop_latched をクリア（デバッグ用）
		int ch = getchar_timeout_us(0);
		if (ch == 'c') {
			drop_latched = false;
			drop_seq = 0;
			fifo_overflow = false;
			dma_lag = false;
			printf("[CMD] Cleared drop_latched\n");
		}

		while (cpu_read_count < written) {
			/* CPUのraw読み込み */
			uint32_t raw = dma_raw_buf[cpu_read_count & LOG_BUF_MASK];

			/* CPU 上の生ポインタ更新 */
			cpu_read_count++;
			
			uint8_t  data  = raw & 0xFF;
			uint8_t  flags = make_flags(raw);
			uint16_t addr  = read_addr_pins();
			// Use timestamp latched in DMA IRQ.
			// Avoid calling time functions here to prevent DMA/PIO stalls.
			uint8_t timeL = dma_irq_time_us & 0xFF;
			
			uint8_t rsv = drop_seq & 0x0F;
			uint64_t pack = 0;
			pack |= ((uint64_t)flags << 56);
			pack |= ((uint64_t)rsv   << 48);
			pack |= ((uint64_t)addr  << 32);
			pack |= ((uint64_t)data  << 16);
			//pack |= ((uint64_t)timeL);
			
			log_buf64[log_wr_idx & LOG_BUF_MASK] = pack;
			log_wr_idx++;
		}

		tight_loop_contents();
	}
}
