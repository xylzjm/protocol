# ADR-0003 — The hardware port is three capabilities

- **Status:** accepted (2026-10-31)

## Context

The board interface was one struct, `modbus_slave_port_t`: seven operations
and one `ctx` covering the RS485 direction, the byte stream, a one-shot timer
and an interrupt mask. The operations are called from three different
contexts — task, TXE interrupt, timer interrupt — and the ordering between
them is part of the contract, which no type could express.

Two things kept pushing on it: `tx_done()` was added as a seventh operation
just to mute a level-triggered TXE, and the core now re-arms the timer from
inside its own expiry interrupt, a rule that lives only in prose.

## Decision

1. **Three structs, one context each.**
   - `modbus_slave_line_t`: `direction(rx, tx)`, `put_byte`, `tx_done`.
   - `modbus_slave_timer_t`: `arm(us)`, `cancel`.
   - `modbus_slave_critical_t`: `enter`, `exit`.
   A board's UART and timer are different objects; the split stops forcing
   them into one merged context.
2. **`direction(rx, tx)` stays one operation.** It is one piece of hardware;
   `(1,0)`, `(0,1)` and `(0,0)` are its contract, not three functions.
3. **The critical section nests with a counter, QP-style.** `enter()` disables
   interrupts and increments a per-instance count; `exit()` decrements and
   enables only at zero. No saved-state token. This is the
   `QF_CRIT_ENTRY()/QF_CRIT_EXIT()` pattern: on a QP port, the board wires
   these two functions to the same counter as the QF critical section, so an
   application's own critical section survives a call into the core.

## Consequences

- A board that only changes its timer implements two operations, with its own
  context.
- The TXE/TC dance is confined to the line adapter; the timer knows nothing
  about the bus.
- The core's `start()`/`stop()` no longer carry a token, and an application
  critical section around a core call now works.
- Every board file changes once, and so does the fake board in the test —
  which is also the first object that proves the new seams.
- The config grows from one nested struct to three.

## Alternatives rejected

- **Keep one struct.** It works, but forces the merged context and spreads the
  transceiver's rules across every board file.
- **A save/restore token for the critical section** (`uint32_t enter()`,
  `exit(state)`). It nests only if every caller uses the same token discipline
  and exits in LIFO order; it cannot nest inside a QP critical section whose
  exit owns the enable. The counter composes with the rest of the firmware.
- **Two structs (line + "clock" with the timer and the critical section).**
  The critical section is not a clock; grouping them was a shape of
  convenience.
