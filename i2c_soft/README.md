# i2c_soft — Bit-Banged I2C Master

A software (bit-banged) I2C master in portable C99. The library owns the wire
protocol — START, repeated START, STOP, byte shifting, acknowledge slots and
bus recovery — and nothing else: your board owns the two pins, the delay and
the interrupt mask, and injects them through ports.

- **Pure C99 core**: `i2c_soft.h` includes only `<stddef.h>`/`<stdint.h>` —
  no MCU header, no vendor SDK, no RTOS — so it compiles anywhere and is
  testable on a host with a fake port.
- **All hardware is a port**: five GPIO/timing operations
  (`i2c_soft_port_t`) plus a nesting interrupt mask
  (`i2c_soft_port_critical_t`). No registers, no pin numbers, no cycle-counted
  busy loops live in this library.
- **Static and multi-instance**: no dynamic memory, no mutable global state.
  The caller owns each `i2c_soft_t`, so any number of buses coexist.
- **Standard mode only**: a half period of `I2C_SOFT_HALF_PERIOD_US` (5 µs,
  nominally 100 kHz). Fast-mode 400 kHz, 10-bit addressing and multi-master
  arbitration are out of scope.

## Layout

```
i2c_soft/                      this library's root
├── README.md                  this file
├── LICENSE-MIT                license
├── i2c_soft.h                 public API: ports, config, instance, functions
├── i2c_soft.c                 the protocol: waveform, recovery, transactions
└── test/                      host test: fake port + fake slave
    └── test_i2c_soft.c
```

The layout is flat: a build only needs this directory on the include path and
`i2c_soft.c` in the source list. The repository root above this directory
holds the shared `.clang-format`.

## Layering

```
device driver (tmp75, ...)      register model, device semantics
        ▲  write_read() / probe()
i2c_soft_t                      pure C: conditions, bytes, ACK slots, recovery
        ▲  port (SDA · SCL · delay) · critical (nesting mask)
board binding                   open-drain GPIO, delay_us, interrupt mask
        ▼
hardware
```

## The GPIO port

All five operations are required; `i2c_soft_ctor()` rejects a configuration
that leaves any of them NULL.

| Operation | Context | Contract |
|---|---|---|
| `sda_write(level)` | task | `0` pulls the line low, `1` releases it. Open-drain only: the library never drives a line high, and a push-pull implementation would fight the slave. |
| `scl_write(level)` | task | Same, for SCL. |
| `sda_read()` | task | The **resolved** line level: `1` = high. Read the pin, not the output latch, or a released line never reads back. |
| `scl_read()` | task | Same, for SCL. |
| `delay_us(us)` | task, interrupts masked | Delay **at least** `us`. Must work with interrupts disabled: every half period is spent inside a critical section, so a delay that waits for a tick interrupt would hang there. |

`ctx` is passed to every operation and belongs to the board — one GPIO port
per bus instance.

## The critical port

| Operation | Context | Contract |
|---|---|---|
| `enter()` | task | Mask interrupts and increment the nesting count. |
| `exit()` | task | Decrement; unmask only when the count reaches zero. |

The count belongs to the board; the library only guarantees balanced pairs.
Implement it as a counter, not a save/restore token, so a caller can hold its
own critical section around a call into the library — the QP pattern
`QF_CRIT_ENTRY()` / `QF_CRIT_EXIT()` shares one counter with it.

The library enters a critical section **per edge group** — START, each data
bit, each ACK slot, the repeated START, STOP, one recovery pulse — never for a
whole transaction. One section spans about 10–15 µs (three half periods plus
the port calls). That keeps ISR latency bounded, but it does not give bus
ownership: an ISR that touches the same pins *between* two bits is still the
caller's problem. When the whole bus must be exclusive, wrap the transaction
in an outer critical section or a mutex.

## Lifecycle

| Call | What it does |
|---|---|
| `i2c_soft_ctor(me, config)` | Pure RAM construction: validates and copies the two ports. No port operation runs, no GPIO is touched. |
| `i2c_soft_init(me)` | The one bus entry point: releases SDA and SCL, waits a half period, then — SCL low ⇒ `ERR_BUS` without a single pulse; SDA low ⇒ one `i2c_soft_bus_recover()`; finally both lines must read high. Repeatable. |

The four transaction APIs **never** initialize implicitly and keep no
"initialized" flag: call `i2c_soft_init()` explicitly before the first
transaction (and whenever you suspect the bus), then use them directly.

## Transaction matrix

| API | Bus activity | Argument requirements |
|---|---|---|
| `i2c_soft_write(me, addr7, data, len)` | START → address+W → data → STOP | `len > 0`, `data != NULL` |
| `i2c_soft_read(me, addr7, data, len)` | START → address+R → data → STOP | `len > 0`, `data != NULL` |
| `i2c_soft_write_read(me, addr7, wr, wr_len, rd, rd_len)` | START → address+W → write → repeated START → address+R → read → STOP | both lengths `> 0`, both pointers non-NULL |
| `i2c_soft_probe(me, addr7)` | START → address+W → sample ACK → STOP | address only, no buffer |
| `i2c_soft_bus_recover(me)` | release, up to 9 SCL pulses, STOP | none |

Rules that hold for all of them:

- The address byte is `(addr7 << 1) | R/W`; every byte goes out MSB-first.
- After each written byte the master samples the slave's ACK on the ninth
  clock. On a NACK — address or write data — it sends STOP immediately, skips
  the remaining bytes, never reaches the read phase, and returns
  `I2C_SOFT_ERR_NACK`. Recovery is **not** automatic: call
  `i2c_soft_bus_recover()` yourself if you want it.
- A read acknowledges the first `len - 1` bytes and leaves the last one
  NACKed before the STOP. The read phase has no slave NACK to report: the
  master controls those clocks, so `i2c_soft_read()` can only fail on the
  address.
- `probe()` sends STOP on both outcomes: `I2C_SOFT_OK` when the address is
  acknowledged, `I2C_SOFT_ERR_NACK` when it is not.
- `write`/`read` with `len == 0`, `write_read` with either length `0`, a NULL
  buffer for a non-zero length, a NULL instance or configuration, and any
  address outside `0x01..0x77` all return `I2C_SOFT_ERR_ARG` **without any
  bus action and without entering a critical section**.

### Status codes

| Code | Value | Meaning |
|---|---|---|
| `I2C_SOFT_OK` | 0 | The transfer (or the bus check) succeeded. |
| `I2C_SOFT_ERR_ARG` | -1 | NULL argument, zero length, or a reserved address. |
| `I2C_SOFT_ERR_NACK` | -2 | The address or a written byte was NACKed; a STOP was sent. |
| `I2C_SOFT_ERR_BUS` | -3 | Bus unavailable: SCL held low, or SDA still low after recovery. |

## Addresses and parameters

- `addr7` accepts `0x01..0x77` only. `0x00` (general call) and `0x78..0x7F`
  (10-bit prefix and other reservations) are rejected with `ERR_ARG`; 10-bit
  addressing is not supported.
- There is no internal length limit. The caller owns the buffers and keeps
  them valid, non-overlapping and unchanging for the duration of the call.
- `i2c_soft_t` is public so it can be statically allocated, but its members
  are private: build it with `i2c_soft_ctor()` and never write them directly.

## Bit timing

Every half period is `I2C_SOFT_HALF_PERIOD_US` (default `5U`) spent in
`delay_us()`. Define the macro to another value before including the header
to change the rate. The port's call overhead is part of each half period, so
the real SCL is slightly *slower* than the nominal figure — the library does
not promise an exact 100 kHz, and standard-mode slaves accept slower clocks.

Each waveform below is one critical section; the delay runs inside it so an
interrupt cannot tear a bit apart. Data and addresses are MSB-first.

| Waveform | Sequence |
|---|---|
| START | SDA falls while SCL is high → half period → SCL low → half period |
| Write bit | SCL low → set SDA → half → SCL high → half → SCL low |
| Read bit / ACK slot | SCL low → release SDA → half → SCL high → half → sample SDA → SCL low |
| Repeated START | SCL low → release SDA → half → SCL high → half → SDA low → half → SCL low → half |
| STOP | SCL low → SDA low → half → SCL high → half → release SDA → half (bus-free time) |

A transaction therefore starts from the idle-high bus that `i2c_soft_init()`
or a previous STOP left behind; STOP pulls SCL low first so it is a STOP
whatever state it finds — bus recovery ends with both lines released.

## Bus recovery

`i2c_soft_bus_recover()` promises one thing: an SDA stuck low.

1. Release both lines. SCL low ⇒ `ERR_BUS` — reported only. Software cannot
   produce a rising edge while a slave holds the clock down, and driving
   against it would make two outputs fight.
2. SDA already high ⇒ send STOP, return `I2C_SOFT_OK`.
3. Otherwise clock at most nine pulses, stopping as soon as SDA reads high.
4. Send STOP; SDA still low ⇒ `ERR_BUS`, otherwise `OK`.

`i2c_soft_init()` runs exactly this once when it sees SDA low, then re-checks
both lines. A SCL held low (clock stretching, a wedged slave, a missing
pull-up) is never repaired in software: reset or power-cycle the slave, or
check the hardware.

## Limits

- **Blocking**: every call runs the whole transfer with no yield. Task
  context only — never an ISR, never a hard-real-time path.
- **Not thread-safe**: the library takes no locks. Serialize calls per
  instance, and serialize access to the bus itself if two masters share it.
- **No clock stretching**: there is no wait loop; every step is bounded.
  `init()`/`bus_recover()` can only *detect* a held SCL.
- **No multi-master arbitration**: two masters on one bus must be excluded by
  the caller.
- **No 10-bit addressing, no SMBus timeouts/PEC, no 400 kHz fast mode.**
- **`ERR_NACK` does not self-heal**: the caller decides whether to recover,
  retry or report.

## A minimal port

```c
#include "i2c_soft.h"

static void board_sda_write(void* ctx, uint8_t level)
{
    (void)ctx;
    gpio_write(GPIOA, PIN_SDA, level);   /* open-drain: 1 = release */
}

static void board_scl_write(void* ctx, uint8_t level)
{
    (void)ctx;
    gpio_write(GPIOA, PIN_SCL, level);
}

static uint8_t board_sda_read(void* ctx)
{
    (void)ctx;
    return (uint8_t)gpio_read(GPIOA, PIN_SDA);
}

static uint8_t board_scl_read(void* ctx)
{
    (void)ctx;
    return (uint8_t)gpio_read(GPIOA, PIN_SCL);
}

static void board_delay_us(void* ctx, uint32_t us)
{
    (void)ctx;
    wk_delay_us(us);                     /* polling: works with IRQs masked */
}

static void board_critical_enter(void* ctx) { (void)ctx; board_crit_inc(); }
static void board_critical_exit(void* ctx)  { (void)ctx; board_crit_dec(); }

static i2c_soft_t i2c1;

void app_i2c_init(void)
{
    i2c_soft_config_t cfg;

    cfg.port.ctx          = NULL;
    cfg.port.sda_write    = board_sda_write;
    cfg.port.scl_write    = board_scl_write;
    cfg.port.sda_read     = board_sda_read;
    cfg.port.scl_read     = board_scl_read;
    cfg.port.delay_us     = board_delay_us;
    cfg.critical.ctx      = NULL;
    cfg.critical.enter    = board_critical_enter;
    cfg.critical.exit     = board_critical_exit;

    (void)i2c_soft_ctor(&i2c1, &cfg);
    (void)i2c_soft_init(&i2c1);
}
```

The board port must configure both pins open-drain with a pull-up before
`i2c_soft_init()` runs.

## Testing

`test/test_i2c_soft.c` drives the library through its public interface with
no framework and nothing compiled into the library itself. The fake port
records every `sda_write`/`scl_write`/`sda_read`/`scl_read`/`delay_us` call in
order, resolves the line levels, and runs a fake slave that shifts bytes
MSB-first, detects START/repeated START/STOP, drives ACK/NACK and read data,
and can be told to NACK an address, NACK the N-th written byte, hold SDA low
for N pulses or hold SCL low. The tests walk the recorded waveform edge by
edge — bit order, ACK slots, STOP — and check the critical-section depth on
every path, including a caller that wraps the transaction in its own critical
section.

```sh
gcc -std=c99 -Wall -Wextra -Werror -pedantic -I library/protocol/i2c_soft \
    library/protocol/i2c_soft/i2c_soft.c \
    library/protocol/i2c_soft/test/test_i2c_soft.c -o test_i2c_soft
./test_i2c_soft
```

On Windows (the MinGW GCC this repository was developed with):

```powershell
C:\Users\HP\mingw64\bin\gcc.exe -std=c99 -Wall -Wextra -Werror -pedantic `
  -I library/protocol/i2c_soft `
  library/protocol/i2c_soft/i2c_soft.c `
  library/protocol/i2c_soft/test/test_i2c_soft.c `
  -o test_i2c_soft.exe
.\test_i2c_soft.exe
```

It prints `N checks, M failures` and exits non-zero on failure. The fake
slave is an ideal model: it verifies protocol and bit order, not pull-ups,
capacitance or glitches — a logic analyser still owns the analog side.

## Licensing

MIT — see `LICENSE-MIT`. The library contains no third-party code.
