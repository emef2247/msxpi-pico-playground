/*
 * io_capture/src/main.c
 *
 * RP2040 I/O bus capture pipeline
 *
 * Uses PIO to trigger on /IOQR (active low) and capture one 32-bit packet per
 * I/O bus access, DMA to stream captured words into a SRAM ring buffer without
 * CPU involvement, and Core1 to filter write cycles targeting OPLL / PSG ports.
 *
 * Default GPIO pin mapping (change the #defines below to adapt):
 *   GPIO0-7   = A0-A7   (address bus, low byte)
 *   GPIO8-15  = D0-D7   (data bus)
 *   GPIO16    = /WR     (active low; 0 = write cycle)
 *   GPIO17    = /IOQR   (active low; used as capture trigger)
 *
 * Captured 32-bit word layout (bit positions match GPIO index):
 *   addr8  = word & 0xFF           (A0-A7 from GPIO0-7)
 *   data8  = (word >> 8)  & 0xFF   (D0-D7 from GPIO8-15)
 *   nwr    = (word >> 16) & 1      (0 = write, 1 = read)
 *   nioqr  = (word >> 17) & 1      (0 = asserted — always 0 at capture time)
 *
 * Filtered ports (write-only):
 *   OPLL : 0x7C, 0x7D
 *   PSG  : 0xA0, 0xA1
 */

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

#include "io_capture.pio.h"

/* ------------------------------------------------------------------ */
/* GPIO pin mapping — edit here to change the hardware wiring          */
/* ------------------------------------------------------------------ */
#define PIN_A0      0   /* A0..A7  on GPIO0..7   */
#define PIN_D0      8   /* D0..D7  on GPIO8..15  */
#define PIN_WR      16  /* /WR     on GPIO16     */
#define PIN_IOQR    17  /* /IOQR   on GPIO17 (capture trigger) */

/* ------------------------------------------------------------------ */
/* Ring buffer — must be a power of 2; increase if overflow is seen    */
/* ------------------------------------------------------------------ */
#define DMA_BLOCK_WORDS  256u
#define RING_WORDS       (8u * 1024u)   /* 8 K words = 32 KB */
#define RING_MASK        (RING_WORDS - 1u)

static uint32_t ring_buf[RING_WORDS];

/* ------------------------------------------------------------------ */
/* Matched-packet secondary buffer (Core1 stores here after filtering) */
/* ------------------------------------------------------------------ */
#define MATCH_BUF_WORDS  256u
#define MATCH_BUF_MASK   (MATCH_BUF_WORDS - 1u)

/* Each entry: bits[7:0] = addr8, bits[15:8] = data8 */
static volatile uint32_t match_buf[MATCH_BUF_WORDS];
/* match_wr is written and read exclusively by Core1; no cross-core access */
static uint32_t match_wr = 0;

/* ------------------------------------------------------------------ */
/* Shared counters                                                      */
/*   captured_count  — words written to ring_buf by DMA (Core0 ISR)   */
/*   processed_count — words consumed by Core1 filter loop             */
/*   matched_count   — write cycles that matched a filter port         */
/*   overflow_count  — ring buffer overflows detected in DMA ISR       */
/* ------------------------------------------------------------------ */
volatile uint32_t captured_count  = 0;
volatile uint32_t processed_count = 0;
volatile uint32_t matched_count   = 0;
volatile uint32_t overflow_count  = 0;

/* ------------------------------------------------------------------ */
/* Internal DMA bookkeeping (Core0 ISR only — no cross-core access)    */
/* ------------------------------------------------------------------ */
/* dma_wr_idx is accessed exclusively inside dma_irq_handler (Core0).
 * atomic operations are used for store/load to avoid compiler reordering
 * and to serve as a memory barrier when updating the public captured_count. */
static volatile uint32_t dma_wr_idx = 0;  /* monotonically increasing word index */

/* ------------------------------------------------------------------ */
/* PIO / DMA handles                                                   */
/* ------------------------------------------------------------------ */
static PIO  pio_inst;
static uint sm_id;
static int  dma_chan;
static dma_channel_config dma_cfg;

/* ------------------------------------------------------------------ */
/* Filter port list                                                    */
/* ------------------------------------------------------------------ */
static const uint8_t FILTER_PORTS[] = { 0x7C, 0x7D, 0xA0, 0xA1 };
#define NUM_FILTER_PORTS  (sizeof(FILTER_PORTS) / sizeof(FILTER_PORTS[0]))

/* ================================================================== */
/* DMA interrupt handler — runs on Core0                              */
/* ================================================================== */
static void __isr dma_irq_handler(void) {
    /* Acknowledge interrupt */
    dma_hw->ints0 = 1u << dma_chan;

    /* Advance write index; captured_count is the public view of it.
     * Use atomic add/store so the compiler cannot reorder these updates. */
    uint32_t new_wr_idx = __atomic_add_fetch(&dma_wr_idx, DMA_BLOCK_WORDS, __ATOMIC_RELAXED);
    __atomic_store_n(&captured_count, new_wr_idx, __ATOMIC_RELEASE);

    /* Overflow detection: ring has been lapped by DMA */
    uint32_t proc    = __atomic_load_n(&processed_count, __ATOMIC_ACQUIRE);
    uint32_t pending = new_wr_idx - proc;
    if (pending > RING_WORDS) {
        __atomic_fetch_add(&overflow_count, 1u, __ATOMIC_RELAXED);
    }

    /* Restart DMA: same read source (PIO FIFO), next ring-buffer slot */
    dma_channel_set_read_addr(dma_chan, &pio_inst->rxf[sm_id], false);
    dma_channel_set_write_addr(dma_chan, &ring_buf[new_wr_idx & RING_MASK], true);
}

/* ================================================================== */
/* PIO initialisation                                                  */
/* ================================================================== */
static void pio_capture_init(void) {
    pio_inst = pio0;
    sm_id    = 0;

    uint offset = pio_add_program(pio_inst, &io_capture_program);
    pio_sm_config c = io_capture_program_get_default_config(offset);

    /* in_base = GPIO0; `in pins, 32` samples GPIO0-31 in one instruction */
    sm_config_set_in_pins(&c, PIN_A0);

    /* /IOQR is also configured as jmp_pin (useful for future jmp-based variants) */
    sm_config_set_jmp_pin(&c, PIN_IOQR);

    /* Shift right (shift_direction=true), autopush disabled, threshold=32.
     * With shift-right: ISR bit N = GPIO(in_base + N), so GPIO0 → bit0, GPIO17 → bit17. */
    sm_config_set_in_shift(&c, true, false, 32);

    /* Configure GPIO0-17 as PIO inputs (high-Z, no drive) */
    for (int i = PIN_A0; i <= PIN_IOQR; i++) {
        pio_gpio_init(pio_inst, i);
        pio_sm_set_consecutive_pindirs(pio_inst, sm_id, i, 1, false);
    }

    pio_sm_init(pio_inst, sm_id, offset, &c);
    pio_sm_set_enabled(pio_inst, sm_id, true);
}

/* ================================================================== */
/* DMA initialisation                                                  */
/* ================================================================== */
static void dma_capture_init(void) {
    dma_chan = dma_claim_unused_channel(true);
    dma_cfg  = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(pio_inst, sm_id, false));
    channel_config_set_read_increment(&dma_cfg, false);  /* always read PIO FIFO */
    channel_config_set_write_increment(&dma_cfg, true);  /* advance through ring */

    /* chain_to = self disables automatic chaining; IRQ handler restarts manually */
    channel_config_set_chain_to(&dma_cfg, (uint)dma_chan);

    dma_channel_configure(dma_chan, &dma_cfg,
                          ring_buf,              /* initial write destination */
                          &pio_inst->rxf[sm_id], /* read from PIO RX FIFO    */
                          DMA_BLOCK_WORDS,        /* words per block          */
                          false);                 /* don't start yet          */

    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_start_channel_mask(1u << dma_chan);
}

/* ================================================================== */
/* Core1 entry — filter loop                                           */
/* ================================================================== */
static void core1_entry(void) {
    while (1) {
        uint32_t cap  = __atomic_load_n(&captured_count,  __ATOMIC_ACQUIRE);
        uint32_t proc = __atomic_load_n(&processed_count, __ATOMIC_ACQUIRE);

        if (proc >= cap) {
            /* Nothing pending; spin without I/O to keep hot path fast */
            tight_loop_contents();
            continue;
        }

        uint32_t word = ring_buf[proc & RING_MASK];

        /* Commit consumption before any further work */
        __atomic_store_n(&processed_count, proc + 1u, __ATOMIC_RELEASE);

        /* Extract fields from the captured 32-bit word */
        uint8_t addr8 = (uint8_t)(word        & 0xFFu);  /* GPIO0-7  = A0-A7 */
        uint8_t data8 = (uint8_t)((word >> 8)  & 0xFFu); /* GPIO8-15 = D0-D7 */
        uint8_t nwr   = (uint8_t)((word >> 16) & 1u);    /* GPIO16   = /WR   */

        /* Only process write cycles (/WR asserted = 0) */
        if (nwr != 0u) {
            continue;
        }

        /* Check whether the address is in the filter list */
        bool match = false;
        for (uint i = 0; i < NUM_FILTER_PORTS; i++) {
            if (addr8 == FILTER_PORTS[i]) {
                match = true;
                break;
            }
        }

        if (!match) {
            continue;
        }

        /* Matched: increment counter and store to secondary ring buffer */
        __atomic_fetch_add(&matched_count, 1u, __ATOMIC_RELAXED);

        uint32_t mw = match_wr & MATCH_BUF_MASK;
        match_buf[mw] = ((uint32_t)addr8) | ((uint32_t)data8 << 8);
        match_wr++;
    }
}

/* ================================================================== */
/* Helpers                                                             */
/* ================================================================== */
static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

/* ================================================================== */
/* main — Core0                                                        */
/* ================================================================== */
int main(void) {
    stdio_init_all();
    sleep_ms(500);  /* wait for USB CDC enumeration or UART to settle */

    printf("io_capture: starting\n");
    printf("  GPIO0-7  = A0-A7   (address bits)\n");
    printf("  GPIO8-15 = D0-D7   (data bits)\n");
    printf("  GPIO16   = /WR     (0 = write cycle)\n");
    printf("  GPIO17   = /IOQR   (trigger, active low)\n");
    printf("  Filter   = write to 0x7C, 0x7D (OPLL), 0xA0, 0xA1 (PSG)\n\n");

    pio_capture_init();
    dma_capture_init();

    /* Align the consumer read pointer to the current DMA write position so
     * stale pre-start data in the ring is not processed. */
    __atomic_store_n(&processed_count,
                     __atomic_load_n(&captured_count, __ATOMIC_ACQUIRE),
                     __ATOMIC_RELEASE);

    /* LED for overflow indication */
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    /* Launch Core1 filter loop */
    multicore_launch_core1(core1_entry);

    const uint32_t STATUS_INTERVAL_MS = 1000u;
    uint32_t last_status_ms  = now_ms();
    uint32_t last_captured   = 0;
    uint32_t last_processed  = 0;
    uint32_t last_matched    = 0;
    uint32_t last_overflow   = 0;

    while (1) {
        uint32_t now = now_ms();

        if (now - last_status_ms >= STATUS_INTERVAL_MS) {
            uint32_t cap  = __atomic_load_n(&captured_count,  __ATOMIC_ACQUIRE);
            uint32_t proc = __atomic_load_n(&processed_count, __ATOMIC_ACQUIRE);
            uint32_t mat  = __atomic_load_n(&matched_count,   __ATOMIC_ACQUIRE);
            uint32_t ovf  = __atomic_load_n(&overflow_count,  __ATOMIC_ACQUIRE);

            uint32_t delta_cap  = cap  - last_captured;
            uint32_t delta_proc = proc - last_processed;
            uint32_t delta_mat  = mat  - last_matched;
            uint32_t delta_ovf  = ovf  - last_overflow;

            printf("[STATUS] captured=%u processed=%u matched=%u overflow=%u"
                   " | +cap=%u +proc=%u +match=%u +ovf=%u\n",
                   (unsigned)cap,  (unsigned)proc,
                   (unsigned)mat,  (unsigned)ovf,
                   (unsigned)delta_cap,  (unsigned)delta_proc,
                   (unsigned)delta_mat,  (unsigned)delta_ovf);

            /* LED on while overflows are occurring */
            gpio_put(PICO_DEFAULT_LED_PIN, (delta_ovf > 0u) ? 1 : 0);

            last_captured  = cap;
            last_processed = proc;
            last_matched   = mat;
            last_overflow  = ovf;
            last_status_ms = now;
        }

        tight_loop_contents();
    }

    return 0;
}
