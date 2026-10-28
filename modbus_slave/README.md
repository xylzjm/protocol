# modbus_slave — Modbus RTU Slave Protocol Stack

A Modbus RTU slave stack in portable C99 with a pluggable server backend.

- **Pure C core**: framing (RX/TX state machines, T3.5 inter-frame timing,
  TX watchdog, CRC16), PDU execution for 9 function codes, and response
  building. The core includes only `<stdint.h>`/`<stddef.h>` — no QP, no
  RTOS, no vendor headers — so it compiles anywhere and can be reused under
  a different backend.
- **Replaceable server backend** under `server/`: a backend receives frame
  events from the core and drives the receive/execute/send primitives in its
  own context (QP state machine, bare-metal superloop, RTOS task). The QP
  backend ships in this repository; `server/bare/` and `server/rtos/` are
  extension points.
- **Board-owned hardware**: the board module implements the hardware port
  (UART direction control, byte output, one-shot timer, critical sections)
  and owns the core instances, typically as a static array — the same
  pattern as `sig24131_board.c`.

Supported function codes: `01`, `02`, `03`, `04`, `05`, `06`, `0F`, `10`,
`11`. Everything else (including `08` Diagnostics, `17`, `2B/0E`) answers
exception `0x01` (illegal function) to unicast requests and is ignored when
broadcast. ASCII, TCP and listen-only mode are out of scope; only the T3.5
inter-frame gap is implemented (no strict T1.5).

## Layout

```
modbus_slave/                      this stack's root
├── README.md                      this file
├── LICENSE-MIT                    license of the pure C core
├── LICENSE-GPL-3.0                license of server/qp/
├── modbus_slave.h                 public core API
├── modbus_slave_internal.h        internal constants/state enums (core only)
├── modbus_slave.c                 ctor/start/stop, receive/execute/send, ISRs, event port
├── modbus_slave_rtu.c             RX/TX frame machines, T3.5, TX watchdog, ADU
├── modbus_slave_pdu.c             the 9 function codes
├── modbus_slave_crc.c             CRC16/Modbus (256-entry table)
└── server/                        backends
    ├── qp/                        QP/C (QHsm) backend — GPL-3.0-or-later
    │   ├── modbus_slave_server_qp.h
    │   └── modbus_slave_server_qp.c
    ├── bare/                      (future) bare-metal superloop backend
    └── rtos/                      (future) RTOS-task backend
```

The repository root above this directory holds only the shared
`.clang-format`; each protocol stack keeps its own README and licenses, so
further stacks can be added next to `modbus_slave/`.

## Layering

```
application callbacks (coils/registers)
        ▲
server backend (qp | bare | rtos)   QHsm / superloop / task
        ▲  event port (frame received / frame sent / TX timeout)
modbus_slave_t core                 pure C: port, RX/TX FSM, T3.5, CRC, PDU
        ▲  hardware port (UART, DE, timer, critical section)
board binding                       pure C: USART, DE, T3.5 timer, buffers
        ▲
hardware
```

Two capability interfaces cross the boundaries:

- `modbus_slave_port_t` — the **hardware port**, a constructor dependency
  passed in `modbus_slave_config_t`. There is no setter: swapping the UART
  or timer at runtime would corrupt the frame machines.
- `modbus_slave_event_port_t` — the **event port**, a runtime dependency
  attached with `modbus_slave_set_event_port()` **before**
  `modbus_slave_start()` (after start it returns `MODBUS_SLAVE_ERR_STATE`).
  `NULL` detaches the port: the core keeps working but drops events.

## Hardware port contract

| Operation | Context | Contract |
|---|---|---|
| `serial_enable(rx, tx)` | task / ISR | `rx=1`: enable RX, release DE, disable the TXE/TC interrupts and clear a pending TC. `tx=1`: assert DE, clear TC, enable TXE+TC. `(0,0)`: disable both, release DE. |
| `put_byte(byte)` | TXE ISR | Write the UART data register, non-blocking. |
| `timer_enable(us)` | task / ISR | One-shot; on expiry call `modbus_slave_timer_expired()`. Must support long timeouts (seconds at low baud). |
| `timer_disable()` | task / ISR | Stop and clear. |
| `critical_enter()` / `critical_exit(state)` | ISR / task | Save+disable interrupts / restore. PRIMASK or BASEPRI on QV/QK; `portSET/CLEAR_INTERRUPT_MASK_FROM_ISR` under FreeRTOS+QP. Not `QF_CRIT_ENTRY/EXIT` (default non-saving, non-nesting). |

Notes the board implementation must respect:

- **TXE vs TC**: TXE only means the data register is empty; the last byte is
  done when TC sets. Clear TC and enable the TC interrupt before starting a
  transmission. TXE fires once more after the last byte is loaded — the core
  tolerates it and simply emits nothing.
- **Mute the transmitter interrupts**: `serial_enable(1, 0)` must also
  disable TXEIE/TCIE. One extra TXE is expected and ignored, but a port that
  leaves TXEIE enabled would keep interrupting on the empty data register.
- `modbus_slave_start()` and `modbus_slave_stop()` return
  `MODBUS_SLAVE_ERR_IO` when a port operation fails; a failed first start is
  rolled back, so the caller can retry.
- `modbus_slave_tx_complete()` (TC interrupt) closes TX, reopens RX and
  notifies `on_frame_sent`.

## Callback contract

Four callbacks (coils, discrete inputs, holding registers, input registers)
plus a shared `ctx`:

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
- Callbacks run synchronously in the backend's context: keep them short,
  never block. Data not ready ⇒ return `SLAVE_BUSY` /
  `SLAVE_DEVICE_FAILURE`; the stack never defers a response.

## Wire behavior

- Broadcast (address 0): `05`/`06`/`0F`/`10` execute without a response;
  `01`/`02`/`03`/`04`/`11` are ignored without invoking callbacks.
- A broadcast is never answered: `modbus_slave_send()` refuses a broadcast
  response even when `execute()` has built one, so every backend gets that
  guarantee from the core itself.
- Report Slave ID (0x11) answers with `slave_id`, the run indicator
  (`0xFF`/`0x00`) and up to 249 additional bytes; the constructor rejects
  `identity.additional_length > 249`. The additional bytes are caller-owned
  — only the pointer is copied, so they must outlive the instance and must
  not change afterwards. `modbus_slave_set_running()` sets the indicator.
- CRC error, frame shorter than 4 bytes, foreign address: silently dropped.
- Exception response: `[addr][func|0x80][exception][crc]`, unicast only.
- T3.5 = 3.5 character times: `(38500000 + baud − 1) / baud` µs up to
  19200 baud, fixed 1750 µs above.
- TX watchdog: `tx_len × 11 × 10⁶ / baud + 10000` µs; on expiry the core
  stops transmitting and reports `on_tx_timeout`.

## Core RX frame machine

```
INIT ──T3.5──► IDLE ──first byte──► RCV ──each byte──► RCV (reload T3.5)
                  ▲                  │
                  │        T3.5 ─────┴──► WAIT_PROCESS (RX off, on_frame_received)
                  │      >256 bytes ──► ERROR ──T3.5──► IDLE (RX on)
                  └──── receive() ◄── WAIT_PROCESS
```

RX is switched off in `WAIT_PROCESS` so the next request cannot overwrite a
frame the backend is still processing. The backend re-opens reception with
`modbus_slave_enable_rx()` after `IGNORE`/`BROADCAST` results (and after
`SENDING` completes or aborts) — every path re-enables RX, so a stuck
transmitter can never leave the slave deaf.

## Server backends

| Backend | Event context | Status |
|---|---|---|
| `server/qp` | directed `QACTIVE_POST()` to this channel's AO | provided, GPL-3.0-or-later |
| `server/bare` | flag set, polled by a superloop | planned |
| `server/rtos` | task notify / queue | planned |

### QP backend

`modbus_slave_server_qp_t` is a **QHsm component**, not an active object:
each channel's AO embeds it and forwards the protocol signals with
`QASM_DISPATCH(&me->server.super, e, 0u)`.

```
STOPPED ──START──► IDLE ──FRAME_RCVED (unicast)──► SENDING
   ▲                ▲  └─────────── FRAME_SENT / TX_TIMEOUT ───┘
   └──── STOP ──────┴───────────────────────────────────────────
```

| State | Event | Action | Next |
|---|---|---|---|
| STOPPED | START | `start()` + apply pending config (via idle entry); ignored if construction faulted | IDLE |
| STOPPED | CONFIG | keep pending | STOPPED |
| IDLE | FRAME_RCVED | `receive()`; IGNORE ⇒ `enable_rx`; BROADCAST ⇒ `execute()`+`enable_rx`; UNICAST ⇒ `execute()`+`send()` | IDLE / SENDING |
| IDLE | CONFIG | RX off, apply config, RX on | IDLE |
| IDLE | STOP | `stop()` | STOPPED |
| IDLE | FRAME_SENT / TX_TIMEOUT | ignore (stale) | IDLE |
| SENDING | FRAME_SENT | — | IDLE |
| SENDING | TX_TIMEOUT | `abort_tx()`, increment `last_error` (readable with `modbus_slave_server_qp_tx_timeout_count()`) | IDLE |
| SENDING | STOP | `stop()` | STOPPED |
| SENDING | CONFIG | keep pending | SENDING |
| SENDING | FRAME_RCVED | ignore | SENDING |

The initial transition lands in `IDLE`, whose entry applies the pending
configuration and starts the core.

Call `modbus_slave_server_qp_init()` once, **after** `QACTIVE_START()` (or
from the AO's own initial transition). `QACTIVE_START()` is what gives the
AO its priority and event queue, so initializing earlier would open the bus
before an ISR-posted protocol event has a valid target. If the backend was
constructed with a NULL core/AO, or against a core that is already started,
it records a fault, stays in `STOPPED`, and ignores `MB_START_SIG`.

The AO must forward the six signals in **every** state it can be in while the
slave is in service. An `MB_FRAME_RCVED_SIG` left in the queue keeps the core
with RX off, and the slave stays deaf until that event is dispatched.

Signals (from `modbus_slave_server_qp.h`): `MB_FRAME_RCVED_SIG`,
`MB_FRAME_SENT_SIG`, `MB_TX_TIMEOUT_SIG` are posted by the backend's event
port from interrupt context; `MB_START_SIG`, `MB_STOP_SIG`, `MB_CONFIG_SIG`
are posted by the application. Application-defined signals start at
`MB_APP_SIG_BASE`.

Requirements:

- **Directed posting only**: the event port posts straight to this channel's
  AO (`QACTIVE_POST`), never publishes — otherwise two RS485 channels would
  cross-talk. Under QK wrap the posts in `QK_ISR_ENTRY`/`QK_ISR_EXIT`;
  under FreeRTOS use `QACTIVE_POST_FROM_ISR`.
- **Queue depth ≥ 8** events per AO, so the three ISR-posted events cannot
  overflow the queue (`QF_NO_MARGIN` assertion).

Minimal application shape:

```c
typedef struct {
    QActive super;                    /* offset 0 */
    modbus_slave_server_qp_t server;  /* QHsm, points at the core */
    /* register image, ... */
} ModbusApp;

static QState ModbusApp_ready(ModbusApp* me, QEvt const* e)
{
    switch (e->sig) {
        case MB_FRAME_RCVED_SIG:
        case MB_FRAME_SENT_SIG:
        case MB_TX_TIMEOUT_SIG:
        case MB_START_SIG:
        case MB_STOP_SIG:
        case MB_CONFIG_SIG:
            QASM_DISPATCH(&me->server.super, e, 0u);
            return Q_HANDLED();
        /* ... application signals ... */
    }
    return Q_SUPER(&QHsm_top);
}
```

To change the slave address at runtime: call
`modbus_slave_set_address_pending()` (with an address in 1..247), then post
`MB_CONFIG_SIG`; the backend applies it in `IDLE` (immediately) or at the
next start (from `STOPPED`/`SENDING`).

## Multi-instance

Each channel is an independent `modbus_slave_t` with its own UART, DE pin,
T3.5 timer, buffers and address. The core keeps no mutable global state, so
any number of instances coexist; the only shared file-scope data is the
board module's instance arrays. A typical board module:

```c
static modbus_slave_t           slaves[2];
static modbus_slave_board_ctx_t contexts[2];  /* uart, de, timer, 256+256 B */

modbus_slave_board_ctor(channel, address, baudrate, callbacks, identity);
modbus_slave_board_get(channel);  /* -> modbus_slave_t* for the backend */
```

Per channel: ~130 B core + 512 B buffers, plus the backend object and the
AO's queue/stack.

## Building

The core is four `.c` files plus the two headers; add
`modbus_slave/*.c` (excluding `server/`) to any C99 project — no QP
include paths required. The QP backend additionally needs the QP/C 8.x
include path and a port (e.g. `ports/arm-cm/qv/gnu` plus
`ports/arm-cm/config`).

Reference checks (Arm GCC):

```sh
# core only, C99, no QP headers visible
arm-none-eabi-gcc -std=c99 -Wall -Wextra -Werror -pedantic -I modbus_slave -c modbus_slave*.c

# QP backend, QP 8.1.3
arm-none-eabi-gcc -std=c11 -Wall -Wextra -Werror -mcpu=cortex-m4 -mthumb \
    -I modbus_slave -I modbus_slave/server/qp \
    -I <qpc>/include -I <qpc>/ports/arm-cm/qv/gnu -I <qpc>/ports/arm-cm/config \
    -c modbus_slave/server/qp/modbus_slave_server_qp.c
```

Formatting: `clang-format` with the repository-root `.clang-format`;
the sources are kept format-clean.

## Licensing

- **Core** (`modbus_slave.h`, `modbus_slave_internal.h`, the four `.c`
  files): MIT — see `LICENSE-MIT`.
- **`server/qp/`**: GPL-3.0-or-later — see `LICENSE-GPL-3.0`. QP/C itself is
  dual-licensed (GPL or commercial from Quantum Leaps).

Because QP/C is GPL-3.0-or-later (absent a commercial license), a firmware
that links the QP backend is a combined work and its overall distribution
is subject to the GPL. The MIT core alone does not impose that; a
permissively licensed `server/bare/` backend keeps a proprietary firmware
possible.
