#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"

/* ===== ピン定義（配線に合わせて変更） ===== */
#define PIN_A0  	0	//A0_3V3
#define PIN_A1  	1
#define PIN_A2  	2
#define PIN_A3  	3
#define PIN_A4  	4
#define PIN_A5  	5
#define PIN_A6  	6
#define PIN_A7  	7
#define PIN_A8  	8	//A8_3V3
#define PIN_A9  	9
#define PIN_A10  	10
#define PIN_A11  	11
#define PIN_A12  	12
#define PIN_A13  	13
#define PIN_A14  	14
#define PIN_A15  	15
#define PIN_D0  	16
#define PIN_D1  	17
#define PIN_D2  	18
#define PIN_D3  	19
#define PIN_D4  	20
#define PIN_D5  	21
#define PIN_D6  	22
#define PIN_D7  	23
#define PIN_RD		24
#define PIN_WR		25
#define PIN_IORQ	26   // MSX /RESET（読むだけ）
#define PIN_SLTSL	27   // 
#define PIN_WAIT	28
#define PIN_RESET	29   // MSX /RESET（読むだけ）

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

    /* 4) MSX が起きたのでバス監視開始 */
    init_bus_pins();

    /* 5) USB があれば起動メッセージ */
    if (stdio_usb_connected()) {
        printf("\nMSX bus logger armed\n");
        printf("Logger start @ %u us\n", now_us());
    }

    bool last_active = false;

    /* 6) メインループ */
    while (1) {
        bool iorq = !gpio_get(PIN_IORQ);
        bool rd   = !gpio_get(PIN_RD);
        bool active = iorq && rd;

        /* IORQ+RD の立ち上がりを検出 */
        if (active && !last_active) {
            if (stdio_usb_connected()) {
                printf("IORQ+RD @ %u us\n", now_us());
            }
        }
        last_active = active;

        /* MSX 電源断検出 → 即 Hi-Z */
        if (!msx_power_on()) {
            if (stdio_usb_connected()) {
                printf("MSX power lost, entering safe state\n");
            }
            enter_safe_state();
            while (1) {
                sleep_ms(100);
            }
        }

        tight_loop_contents();
    }
}