# io_capture — RP2040 I/O Bus Capture Pipeline

Minimal, drop-resistant I/O bus capture for Raspberry Pi Pico (RP2040).

Triggers on **/IOQR (active low)**, captures one 32-bit packet per I/O access,
streams packets into SRAM via DMA, and uses Core1 to filter write cycles
addressed to OPLL or PSG ports — all without CPU polling in the hot path.

---

## Hardware Wiring

| GPIO   | Signal  | Description                        |
|--------|---------|------------------------------------|
| 0 – 7  | A0–A7   | Address bus (low byte)             |
| 8 – 15 | D0–D7   | Data bus                           |
| 16     | /WR     | Write strobe, active low           |
| 17     | /IOQR   | I/O request, active low (trigger)  |

> **Important**: The MSX bus is 5 V TTL. The Pico GPIO is 3.3 V.  
> Always use a level-shifter or high-impedance 3.3 V compatible buffer  
> (e.g. 74LVC244) between the MSX bus and the Pico GPIO pins.  
> Never connect 5 V signals directly to the Pico.

All GPIO pins are configured as inputs (high-Z). The Pico does not drive the bus.

---

## What is Captured

Each I/O access produces exactly **one 32-bit word** pushed to the PIO RX FIFO.
The word is a raw snapshot of GPIO 0–31 at the moment /IOQR is asserted:

```
bit 31              17 16  15              8  7              0
 ┌─────────────────┬──┬──┬────────────────┬────────────────┐
 │  unused (18-31) │/IOQR│/WR│   D0-D7 (8-15) │  A0-A7 (0-7)  │
 └─────────────────┴──┴──┴────────────────┴────────────────┘
```

Field extraction in C:

```c
uint8_t addr8  = (uint8_t)(word        & 0xFF);  // A0-A7
uint8_t data8  = (uint8_t)((word >> 8)  & 0xFF); // D0-D7
uint8_t nwr    = (uint8_t)((word >> 16) & 1);    // 0 = write
uint8_t nioqr  = (uint8_t)((word >> 17) & 1);    // always 0 at capture time
```

---

## Architecture

```
                   PIO (Core0)
  /IOQR assert ──► wait 0 gpio 17
                   in   pins, 32   ← samples GPIO0-31
                   push block      → RX FIFO
  /IOQR deassert ► wait 1 gpio 17  (1 access = 1 packet)
                        │
                        ▼
                   DMA channel 0
                   (no CPU involvement)
                        │
                        ▼
                   ring_buf[8192]   (SRAM ring buffer)
                   ▲        │
              DMA IRQ        ▼
           (Core0 ISR)   Core1 filter loop
           advances       reads from ring_buf
           write ptr      filters: /WR==0 AND
           restarts DMA   addr ∈ {0x7C,0x7D,0xA0,0xA1}
                               │
                               ▼
                          match_buf[] + matched_count
```

- **Core0**: runs `main()` (status output) and the DMA IRQ handler.
- **Core1**: tight filter loop — reads ring_buf, no I/O in the hot path.
- **DMA**: transfers `DMA_BLOCK_WORDS` (256) words per block, fires IRQ on completion.
  The IRQ handler advances the write index and immediately restarts DMA for the next block.

---

## Counters and Status Output

Every second, a status line is printed over USB CDC (or UART if reconfigured):

```
[STATUS] captured=12800 processed=12800 matched=42 overflow=0 | +cap=256 +proc=256 +match=3 +ovf=0
```

| Counter       | Meaning                                                                  |
|---------------|--------------------------------------------------------------------------|
| `captured`    | Total 32-bit words written to the ring buffer by DMA                    |
| `processed`   | Total words consumed by the Core1 filter loop                           |
| `matched`     | Write cycles to OPLL (0x7C/0x7D) or PSG (0xA0/0xA1) that were matched  |
| `overflow`    | Number of times DMA lapped Core1 (ring buffer overran)                  |
| `+cap`        | Words captured in the last 1-second interval                            |
| `+proc`       | Words processed in the last 1-second interval                           |
| `+match`      | Matches in the last 1-second interval                                   |
| `+ovf`        | Overflows in the last 1-second interval                                 |

**Overflow = 0** in normal operation. If `overflow` grows, Core1 cannot keep up with
the incoming packet rate. Increase `RING_WORDS` or reduce Core1 work to fix this.

The onboard LED is lit during any 1-second status interval in which new overflows
occurred. If the overflow rate drops to zero, the LED turns off in the next interval.

---

## Filtered Ports

Only **write cycles** (`/WR` asserted, i.e. `nwr == 0`) to the following I/O ports
are counted as matches:

| Device | Port(s)    |
|--------|------------|
| OPLL   | 0x7C, 0x7D |
| PSG    | 0xA0, 0xA1 |

All other packets (reads, writes to other ports) are silently discarded by Core1.

---

## Build

Prerequisites: [pico-sdk](https://github.com/raspberrypi/pico-sdk) installed and
`PICO_SDK_PATH` set.

```sh
cd io_capture
mkdir build && cd build
cmake ..
make
# → io_capture.uf2
```

Flash to Pico in BOOTSEL mode by copying `io_capture.uf2` to the USB drive.

Connect a serial terminal (115200 baud) to the Pico USB CDC port to view status output.

---

## Configuration

All pin assignments are `#define`s at the top of `src/main.c`:

```c
#define PIN_A0      0   /* A0..A7  on GPIO0..7   */
#define PIN_D0      8   /* D0..D7  on GPIO8..15  */
#define PIN_WR      16  /* /WR     on GPIO16     */
#define PIN_IOQR    17  /* /IOQR   on GPIO17     */
```

Buffer sizes:

```c
#define DMA_BLOCK_WORDS  256u          /* words per DMA block / IRQ */
#define RING_WORDS       (8u * 1024u)  /* ring buffer depth (power of 2) */
```

Increase `RING_WORDS` if you see `overflow` counter incrementing under heavy load.

---

## File Structure

```
io_capture/
├── CMakeLists.txt
├── README.md
└── src/
    ├── io_capture.pio   PIO program (/IOQR trigger, 1 sample per access)
    └── main.c           Firmware: DMA ring, Core1 filter, status output
```
