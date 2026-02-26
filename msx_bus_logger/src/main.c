#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "msx_rd_logger.pio.h"

/* ===== ピン定義（配線に合わせて変更） ===== */
#define PIN_A0 0 //A0_3V3 
#define PIN_A1 1 
#define PIN_A2 2 
#define PIN_A3 3 
#define PIN_A4 4 
#define PIN_A5 5 
#define PIN_A6 6 
#define PIN_A7 7 
#define PIN_A8 8 //A8_3V3 
#define PIN_A9 9 
#define PIN_A10 10 
#define PIN_A11 11 
#define PIN_A12 12 
#define PIN_A13 13 
#define PIN_A14 14 
#define PIN_A15 15 
#define PIN_D0 16 
#define PIN_D1 17 
#define PIN_D2 18 
#define PIN_D3 19 
#define PIN_D4 20 
#define PIN_D5 21 
#define PIN_D6 22 
#define PIN_D7 23 
#define PIN_RD 24 
#define PIN_WR 25 
#define PIN_IORQ 26 // MSX /RESET（読むだけ） 
#define PIN_SLTSL 27 // 
#define PIN_WAIT 28 
#define PIN_RESET 29 // MSX /RESET（読むだけ）

/* リングバッファ */
#define DMA_BLOCK_WORDS 32
#define LOG_ENTRIES (8 * 1024)   // 8K entries
#define LOG_BUF_MASK (LOG_ENTRIES - 1)

/* ===== グローバル変数 ===== */
PIO pio = pio0;
uint sm = 0;
int dma_chan;
dma_channel_config dma_cfg;

volatile uint32_t dma_write_count = 0;
uint32_t cpu_read_count = 0;

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
    const uint pins[] = {
        PIN_IORQ,
        PIN_RD,
        PIN_RESET,
        // 実際に配線しているピンだけ列挙すること
    };

    for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_disable_pulls(pins[i]);
    }
	
	// WAIT は必ず非アサート
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 1);
}

/* ===== バス監視ピン初期化 ===== */
void init_bus_pins(void) {
    gpio_init(PIN_IORQ);
    gpio_set_dir(PIN_IORQ, GPIO_IN);
    gpio_disable_pulls(PIN_IORQ);

    gpio_init(PIN_RD);
    gpio_set_dir(PIN_RD, GPIO_IN);
    gpio_disable_pulls(PIN_RD);
}

void pio_init_logger	(void) {
    uint offset = pio_add_program(pio, &msx_rd_logger_program);
    pio_sm_config c = msx_rd_logger_program_get_default_config(offset);

    // データピン D0-D7
    sm_config_set_in_pins(&c, PIN_D0);
    sm_config_set_jmp_pin(&c, PIN_IORQ); // /IORQ

    // シフト設定
    sm_config_set_in_shift(&c,
        false,   // shift left
        true,    // autopush
        8        // 8bit で push
    );

    // GPIO 初期化
    for (int i = PIN_D0; i < PIN_IORQ; i++) {
        pio_gpio_init(pio, i);
        pio_sm_set_consecutive_pindirs(pio, sm, i, 1, false);
    }
    pio_gpio_init(pio, PIN_IORQ);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_IORQ, 1, false);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
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
    dma_hw->ints0 = 1u << dma_chan;
    dma_write_count += DMA_BLOCK_WORDS;

    dma_channel_set_read_addr(dma_chan, &pio->rxf[sm], false);
	dma_channel_set_write_addr(
		dma_chan,
		&dma_raw_buf[dma_wr_idx & LOG_BUF_MASK],
		true
	);

	// DMA 遅延検出（リングバッファ基準）
	uint32_t pending = dma_write_count - cpu_read_count;
	bool lag_now = (pending > (LOG_ENTRIES - DMA_BLOCK_WORDS));
	if (lag_now && !dma_lag_prev) {
		dma_lag = true;
		drop_latched = true;
		drop_seq++;
	}
	dma_lag_prev = lag_now;
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

/* ADDR（A0–A15）取得関数 */
static inline uint16_t read_addr_pins(void) {
    uint16_t addr = 0;
    for (int i = 0; i < 16; i++) {
        addr |= (gpio_get(PIN_A0 + i) << i);
    }
    return addr;
}

/* FLAGS 合成関数 */
static inline uint8_t make_flags(uint32_t pio_v) {
    uint8_t f = 0;
	
    /* PIO サンプル由来 */
    if (!(pio_v & (1u << 8)))  f |= 1 << 0; // RD
    if (!(pio_v & (1u << 9)))  f |= 1 << 1; // WR
    if (!(pio_v & (1u << 10))) f |= 1 << 2; // IO (/IORQ low)

    /* GPIO 直読み */
    if (gpio_get(PIN_SLTSL))   f |= 1 << 3; // EXT SLOT

    /* DROP 系（状態フラグ） */
    if (drop_latched)          f |= 1 << 4; // DROP_LATCHED
    if (fifo_overflow)         f |= 1 << 5; // FIFO_OVERFLOW
    if (dma_lag)               f |= 1 << 6; // DMA_LAG

    return f;
}

/* ===== メイン ===== */

int main(void) {
    stdio_init_all();

    /* 1) 起動直後に必ず安全状態へ */
    enter_safe_state();

    /* 2) /RESET は読むだけ（pull しない） */
    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_IN);

    /* 3) MSX 電源 ON 待ち */
    while (!msx_power_on()) {
        tight_loop_contents();   // 低消費＆NOP
    }

	/* 4) MSX が起きたので PIO ロガー開始 */
	pio_init_logger();
	

	/* 5) DMA 起動　(必ず PIO → DMA の順: FIFO が存在してから DMA を張る) */
	dma_init_logger_with_ring_enabled(pio, sm);


    /* 6) USB があれば起動メッセージ */
    if (stdio_usb_connected()) {
        printf("\nMSX bus logger armed\n");
        printf("Logger start @ %u us\n", now_us());
    }

	/* 7) メインループ*/
	while (1) {
		uint32_t written = dma_write_count;

		while (log_wr_idx < written) {
			uint32_t raw = dma_raw_buf[log_wr_idx & LOG_BUF_MASK];
			cpu_read_count++;
			if (dma_lag) fifo_overflow = true;
			
			uint8_t  data  = raw & 0xFF;
			uint8_t  flags = make_flags(raw);
			uint16_t addr  = read_addr_pins();
			uint8_t  timeL = now_us() & 0xFF;

			uint8_t rsv = drop_seq & 0x0F;
			uint64_t pack = 0;
			pack |= ((uint64_t)flags << 56);
			pack |= ((uint64_t)rsv   << 48);
			pack |= ((uint64_t)addr  << 32);
			pack |= ((uint64_t)data  << 16);
			pack |= ((uint64_t)timeL);

			log_buf64[log_wr_idx & LOG_BUF_MASK] = pack;

			/* テキスト確認用（CDC） */
			if (stdio_usb_connected()) {
				printf("%02X,%02X,%04X,%02X,%02X\n",
					   flags,rsv, addr, data, timeL);
			}

			log_wr_idx++;
		}

		tight_loop_contents();
	}
}