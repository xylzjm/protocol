# modbus_slave — Modbus RTU Slave Protocol Stack

A Modbus RTU slave stack in portable C99. The core owns the transaction; your
application owns the scheduling.

- **Pure C core**: framing (session, RX/TX machines, T3.5 inter-frame timing,
  TX watchdog, CRC16), the nine function codes, and reply building. The core
  includes only `<stdint.h>`/`<stddef.h>` — no QP, no RTOS, no vendor headers —
  so it compiles anywhere.
- **One operation per frame**: the core reports a completed frame; you call
  `modbus_slave_advance()` from your own context — an active object, a task, a
  superloop — and it runs the whole transaction. The reception window never
  leaves the core, and there is no backend to write.
- **The address is a question**: the callbacks answer the current slave
  address before every frame, so a register write or a DIP switch applies to
  the next one, with no reconfiguration step.
- **Board-owned hardware**: the board module implements three capabilities —
  the RS485 line, the one-shot timer and a nesting interrupt mask, each with
  its own context — and owns the core instances, typically as a static array,
  the same pattern as `sig24131_board.c`.

Supported function codes: `01`, `02`, `03`, `04`, `05`, `06`, `0F`, `10`,
`11`. Everything else (including `08` Diagnostics, `17`, `2B/0E`) answers
exception `0x01` (illegal function) to unicast requests and is ignored when
broadcast. ASCII, TCP and listen-only mode are out of scope; only the T3.5
inter-frame gap is implemented (no strict T1.5).

## Layout

```
modbus_slave/                      this stack's root
├── README.md                      this file
├── GLOSSARY.md                    domain terms (transaction, reception window, ...)
├── LICENSE-MIT                    license
├── docs/adr/                      architecture decisions (the core owns the
│                                  transaction; the address is a callback)
├── test/                          host test with a fake board
│   └── test_modbus_slave.c
├── inc/                           core headers
│   ├── modbus_slave.h             public core API
│   └── modbus_slave_internal.h    internal constants/state enums (core only)
├── src/                           pure C core sources
│   ├── modbus_slave.c                 ctor/start/stop, advance(), ISRs, event port
│   ├── modbus_slave_rtu.c             RX/TX frame machines, T3.5, TX watchdog
│   ├── modbus_slave_adu.c             frame codec: ADU layout, request/response views
│   ├── modbus_slave_function_code.c   the 9 function codes
│   └── modbus_slave_crc.c             CRC16/Modbus (256-entry table)
```

The repository root above this directory holds only the shared
`.clang-format`; each protocol stack keeps its own README and license, so
further stacks can be added next to `modbus_slave/`.

## Layering

```
application AO (event queue, its own states)
        ▲  event port (frame received / frame sent / TX timeout)
modbus_slave_t core                 pure C: session, RX/TX FSM, T3.5, CRC, PDU
        ▲  line · timer · critical (three capability structs, one ctx each)
board binding                       pure C: USART, DE, T3.5 timer, buffers
        ▲
hardware
```

Five seams, each a constructor or runtime dependency:

- `modbus_slave_line_t` — the **RS485 line**: direction, one byte out, end of
  the stream. A constructor dependency in `modbus_slave_config_t`.
- `modbus_slave_timer_t` — the **one-shot timer** behind T3.5 and the TX
  watchdog. Constructor dependency.
- `modbus_slave_critical_t` — the **nesting interrupt mask**. Constructor
  dependency. The three hardware structs carry their own `ctx`, so a board's
  UART and timer stay separate objects.
- `modbus_slave_event_port_t` — the **event port**, a runtime dependency
  attached with `modbus_slave_set_event_port()` **before**
  `modbus_slave_start()` (after start it returns `MODBUS_SLAVE_ERR_STATE`).
  `NULL` detaches the port: the core keeps working but drops events.
- `modbus_slave_callbacks_t` — the **data space and the address**: five
  callbacks asked synchronously inside `advance()`, in the caller's context.

## Hardware capabilities

All seven operations are required; the constructor rejects a NULL.

### Line — the RS485 transceiver

| Operation | Context | Contract |
|---|---|---|
| `direction(rx, tx)` | task / ISR | `rx=1`: enable RX, release DE, disable the TXE/TC interrupts and clear a pending TC. `tx=1`: assert DE, clear TC, enable TXE+TC. `(0,0)`: disable both, release DE. Each call must leave the direction it does not select switched off. |
| `put_byte(byte)` | TXE ISR | Write the UART data register, non-blocking. |
| `tx_done()` | TXE ISR | The core has loaded the last byte (or has nothing left to send). Stop requesting TXE; keep TCIE enabled so the TC path closes the frame and releases DE. |

### Timer — one-shot

| Operation | Context | Contract |
|---|---|---|
| `arm(us)` | task / ISR | One-shot; on expiry call `modbus_slave_timer_expired()`. Must support long timeouts (seconds at low baud). Must also be callable from the timer's own expiry ISR — the core re-arms it there to resynchronize reception after an aborted transmission — so clear the pending match before arming again. |
| `cancel()` | task / ISR | Stop and clear. |

### Critical — a nesting interrupt mask

| Operation | Context | Contract |
|---|---|---|
| `enter()` | task / ISR | Disable interrupts and increment the nesting count. |
| `exit()` | task / ISR | Decrement; re-enable interrupts only when the count reaches zero. |

The count belongs to the board; the core only guarantees balanced pairs.
Implement it as a counter rather than a save/restore token — the QP
`QF_CRIT_ENTRY()/QF_CRIT_EXIT()` pattern — so an application can hold its own
critical section across a call into the core. On a QP port, wire these two
functions to the same counter as the QF critical section.

Notes the board implementation must respect:

- **TXE vs TC**: TXE only means the data register is empty; the last byte is
  done when TC sets. Clear TC and enable the TC interrupt before starting a
  transmission. When the core has loaded the last byte it calls `tx_done()`:
  the line disables TXEIE there and leaves TCIE enabled, so the TC path closes
  the frame. There is no trailing empty TXE any more.
- **Mute the transmitter interrupts**: `direction(1, 0)` must also disable
  TXEIE/TCIE (end of frame). `tx_done()` is the mid-frame mute that keeps
  TCIE on.
- `modbus_slave_start()` and `modbus_slave_stop()` return
  `MODBUS_SLAVE_ERR_IO` when a capability fails; a failed first start is
  rolled back, so the caller can retry.
- `modbus_slave_tx_complete()` (TC interrupt) closes TX, reopens RX and
  notifies `on_frame_sent`.

## Callback contract

Five callbacks — coils, discrete inputs, holding registers, input registers and
the slave address — plus a shared `ctx`:

- `address()` returns the current slave address (1..247) and is asked before
  every received frame. A product that combines a register base with a DIP
  offset computes it here. A value outside 1..247 makes the core ignore every
  frame, so a miscomputed address leaves the slave silent instead of answering
  as if it were the broadcast address.
- Addresses are 0-based wire addresses; no `+1` adjustment anywhere.
- Registers are big-endian, 2 bytes each (`modbus_slave_get_u16()` /
  `modbus_slave_put_u16()` helpers). Reads: up to 125 registers; writes: up
  to 123.
- Coils/discrete inputs are packed LSB-first, unused high bits of the last
  byte cleared. Reads: up to 2000 bits; writes: up to 1968.
- Read mode: the callback fills `buffer`. Write mode: the callback reads
  `buffer` and must not modify it.
- Out-of-range addresses are the callback's job
  (`MODBUS_SLAVE_EX_ILLEGAL_DATA_ADDRESS`); quantities and byte counts are
  the stack's job (`ILLEGAL_DATA_VALUE`).
- Callbacks run synchronously in the caller's context — the one that calls
  `advance()`: keep them short, never block. Data not ready ⇒ return
  `SLAVE_BUSY` / `SLAVE_DEVICE_FAILURE`; the stack never defers a response.

## Wire behavior

- Broadcast (address 0): `05`/`06`/`0F`/`10` execute without a response;
  `01`/`02`/`03`/`04`/`11` are ignored without invoking callbacks.
- A broadcast is never answered: `advance()` executes the write and drops
  the reply it built, so the guarantee comes from the core itself.
- Report Slave ID (0x11) answers with `slave_id`, the run indicator
  (`0xFF`/`0x00`) and up to 249 additional bytes; the constructor rejects
  `identity.additional_length > 249`. The additional bytes are caller-owned
  — only the pointer is copied, so they must outlive the instance and must
  not change afterwards. `modbus_slave_set_running()` sets the indicator.
- CRC error, frame shorter than 4 bytes, foreign address: silently dropped.
- Exception response: `[addr][func|0x80][exception][crc]`, unicast only.
- T3.5 = 3.5 character times: `(38500000 + baud − 1) / baud` µs up to
  19200 baud, fixed 1750 µs above.
- TX watchdog: `tx_len × 11 × 10⁶ / baud + 10000` µs. On expiry the core
  aborts the transmission in the timer interrupt — TXE/TC disabled, DE
  released, reception resynchronized on a fresh T3.5 — and then reports
  `on_tx_timeout`, which is informational: the line is already free.

## Core session

The core is in exactly one of six states. The state implies what reception is
doing and what the one-shot timer means, so the two cannot disagree:

| State | Reception | Timer | Leaves to |
|---|---|---|---|
| `STOPPED` | off | — | `start()` → `DISCARDING` |
| `DISCARDING` | on, bytes dropped | T3.5 gap | T3.5 → `IDLE` |
| `IDLE` | on | — | first byte → `RECEIVING` |
| `RECEIVING` | on | T3.5 gap | T3.5 → `CAPTURED`; overlong frame → `DISCARDING` |
| `CAPTURED` | off (frozen) | — | `advance()` → `IDLE` or `SENDING` |
| `SENDING` | off | TX watchdog | TC → `IDLE`; watchdog → `DISCARDING` |

`DISCARDING` is the resynchronization state: the core enters it at start,
after an overlong frame and after an aborted transmission. Reception stays on
while the bytes are dropped, and the gap after the last one is what makes the
next frame's edge trustworthy.

RX is switched off in `CAPTURED` so the next request cannot overwrite a frame
the application is still processing. The **reception window belongs to the
core**, and `modbus_slave_advance()` is the only operation that closes a
transaction: it re-opens reception for an ignored frame, a broadcast or a
unicast without a reply, and hands the window to the TX path — which releases
it at TC, or when the watchdog aborts the transmission — for a unicast that is
answered. Every transaction outcome releases the window and the application
never touches reception switching — so a rejected request or a stuck
transmitter can never leave the slave deaf.

## Driving it from an application AO

There is no backend to write: the transaction is one call, and the AO is
already the thing that serializes events. Wire the event port to post your own
signals, and call the core from the AO:

```c
enum AppSignals {
    MB_FRAME_RCVED_SIG = Q_USER_SIG,
    MB_TX_TIMEOUT_SIG,
    /* the application's own signals follow */
};

static void on_frame_received(void* ctx)   /* T3.5 interrupt */
{
    QACTIVE_POST(ao_of(ctx), &frame_evt, 0);
}

static void on_tx_timeout(void* ctx)       /* timer interrupt */
{
    QACTIVE_POST(ao_of(ctx), &timeout_evt, 0);
}

static QState App_serving(App* me, QEvt const* e)
{
    switch (e->sig) {
        case MB_FRAME_RCVED_SIG:
            (void)modbus_slave_advance(&me->slave);
            return Q_HANDLED();
        case MB_TX_TIMEOUT_SIG:
            if (me->faults < 0xFFU) { me->faults++; }
            return Q_HANDLED();
        /* ... */
    }
    return Q_SUPER(&QHsm_top);
}
```

Only `on_frame_received` changes what the master sees: it is the call that
runs the transaction. `on_frame_sent` and `on_tx_timeout` are optional
diagnostics — the core has already closed the transaction (line released,
reception reopened or resynchronizing) before either fires. A `NULL` callback
is simply not called, so leaving them unset costs nothing.

Rules that live on the application side:

- **Post, never process in the ISR.** The event callbacks run in interrupt
  context; `advance()` runs CRC, the register callbacks and the reply.
- **One context.** `advance()` is not re-entrant: call it from the AO only.
- **Queue.** Two signals per transaction; the AO's event queue must not
  overflow (with QP, at least 4 events).
- **Lifecycle.** Call `start()` when the channel enters service and `stop()`
  when it leaves. Both are idempotent; `start()` refuses while a reply is on
  the wire.

The same recipe in a superloop: set a flag in `on_frame_received`, poll it,
call `advance()`. In a FreeRTOS task: `xTaskNotifyFromISR()` /
`ulTaskNotifyTake()`.

## The slave address

The core stores no address. Before every frame it asks `callbacks.address()`,
which lets one value serve both of the product's sources:

```c
static uint8_t app_address(void* ctx)
{
    App* me = ctx;
    const uint8_t base = (uint8_t)me->holding[ADDR_REG];  /* written by 0x06/0x10 */
    return (uint8_t)(base + me->dip_offset);              /* DIP switch decoded at boot */
}
```

- A unicast write to the address register is answered with the address the
  master used: the reply carries `req->address`, captured when the frame was
  taken. The next frame is compared against the new value.
- A broadcast write to the same register is never answered and needs no
  follow-up: the next frame already uses the new address.
- Validate in the register callback and return
  `MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE` for a value that would put
  base + offset outside 1..247; the register keeps its old value and the
  master sees an exception.
- `modbus_slave_get_address()` asks the same callback, so it is useful for
  logging without duplicating the formula.

## Multi-instance

Each channel is an independent `modbus_slave_t` with its own UART, DE pin,
T3.5 timer, buffers, callbacks and address. The core keeps no mutable global
state, so any number of instances coexist; the only shared file-scope data is
the board module's instance arrays. A typical board module:

```c
static modbus_slave_t           slaves[2];
static modbus_slave_board_ctx_t contexts[2];  /* uart, de, timer, 256+256 B */

modbus_slave_board_ctor(channel, baudrate, callbacks, identity);
modbus_slave_board_get(channel);  /* -> modbus_slave_t* for the application */
```

Per channel: ~120 B core + 512 B buffers, plus the AO's queue and stack.

## Building

The core is five `.c` files under `src/` plus the two headers under `inc/`.
Add `src/*.c` and put `inc/` on the include path — nothing else is required.

Reference check (Arm GCC, C99, no extra headers):

```sh
arm-none-eabi-gcc -std=c99 -Wall -Wextra -Werror -pedantic \
    -I modbus_slave/inc -c modbus_slave/src/*.c
```

Formatting: `clang-format` with the repository-root `.clang-format`; the
sources are kept format-clean.

## Testing

`test/test_modbus_slave.c` drives the core through its public interface: a
fake board supplies the three hardware capabilities and the events, the test
feeds ADU bytes, and it checks the bytes that would reach the wire. A second
part checks the frame codec's decode table directly — a request PDU in, a
request view or an exception out, with no port and no interrupt involved.
There is no framework and nothing is compiled into the library:

```sh
gcc -std=c99 -Wall -Wextra -Werror -pedantic -I modbus_slave/inc \
    modbus_slave/src/*.c modbus_slave/test/test_modbus_slave.c -o test_modbus_slave
./test_modbus_slave
```

It prints `N checks, M failures` and exits non-zero on failure. Expected
frames are rebuilt with an independent bit-wise CRC16, so the table-driven CRC
never verifies itself.

## Licensing

MIT — see `LICENSE-MIT`. The library contains no third-party code; a firmware
that links QP for its own active objects is a combined work under QP's license,
which is between the firmware and Quantum Leaps.
