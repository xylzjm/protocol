# ADR-0001 — The core owns the transaction and its reception window

- **Status:** accepted (2026-10-30)
- **Supersedes:** §4.6 of the implementation plan ("reception window
  ownership"), which described the same rule in prose only.

## Context

Reception is frozen while a frame is processed: the T3.5 interrupt switches
it off so the next request cannot overwrite the ADU the backend is about to
read. Deciding when it opens again was spread over three primitives
(`receive()`, `execute()`, `send()`) and their callers: nine places in the
core switched reception back on, and the backend had to know which branch
released the window, that a broadcast must still be executed, and that a TX
watchdog expiry had to be followed by `abort_tx()`.

Two failure modes came from that split:

- a backend that skipped a call left the slave deaf, silently;
- after a TX timeout the core had already declared the transaction over
  (`tx_state = IDLE`) while the hardware still drove DE, so the shared RS485
  segment stayed blocked until the backend's active object ran — unbounded
  under AO starvation or a lost event.

## Decision

1. **One operation per frame.** `modbus_slave_advance()` validates the
   captured ADU, executes the function code, builds the reply and starts the
   transmission. It returns `NONE` (nothing was waiting), `HANDLED` (consumed,
   reception open again) or `SENDING` (a reply is on the wire).
2. **The reception window never leaves the core.** Every outcome of
   `advance()` leaves reception open or hands it to the TX path, which
   releases it at TC or on the watchdog. No backend call switches reception.
3. **A timed-out transmission is aborted by the core**, in the timer
   interrupt: disable TXE/TC, release DE, then resynchronize reception on a
   fresh T3.5. `on_tx_timeout` remains, but it is a notification, not the
   recovery path.

## Consequences

- A backend is: one call per frame, one branch on the result, one counter.
  See `GLOSSARY.md` for *transaction*, *reception window*, *resynchronization*.
- Safety no longer depends on event delivery: a dropped `on_tx_timeout`
  costs a diagnostic count, not the bus.
- The bus is released within one interrupt latency instead of one AO
  dispatch. This changes behavior after a timeout: reception resumes
  earlier, after a full T3.5 gap.
- `apply_config()` is only legal while no reply is on the wire; the core
  returns `MODBUS_SLAVE_ERR_STATE` otherwise, and it resynchronizes reception
  around the address swap.
- The core now holds one more policy: it recovers unconditionally. A backend
  can observe and log the timeout, but cannot delay the recovery.

## Alternatives rejected

- **Keep `abort_tx()` in the backend.** Preserves application control and
  keeps policy out of the core, but leaves "the transaction ends" split
  between the core and the AO, and leaves the bus held for an unbounded time
  when the AO does not run. Rejected: the whole point of `advance()` is that
  the window closes inside the module.
- **Abort without resynchronizing.** One line less, but the tail of the
  aborted response can be mistaken for the start of the next frame on a
  transceiver that echoes while DE is asserted.
