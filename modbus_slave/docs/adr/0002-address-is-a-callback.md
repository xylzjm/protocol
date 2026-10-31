# ADR-0002 — The slave address is a callback, not state

- **Status:** accepted (2026-10-31)
- **Supersedes in part:** ADR-0001 — the cached address, the pending address
  and `apply_config()` are gone.

## Context

The product derives the slave address from two inputs: a holding register the
master can write, and a DIP switch decoded at boot. The effective address is
`base + offset`.

The core used to cache that value and offered `set_address_pending()` plus
`apply_config()` to change it "at a safe moment". That split forced the
application to know when the moment was:

- inside a frame callback it is **not** safe — the transaction's reception
  window is frozen and `apply_config()` would reopen it under the reply that
  is about to be echoed;
- while a reply is on the wire it is not safe either — the core refuses;
- after the reply, only if the application remembers to do it, and a DIP
  change at runtime has no natural trigger at all.

## Decision

1. **Ask, do not store.** `callbacks.address()` is called once per received
   frame and returns the current address (1..247). The core keeps no copy.
2. **The reply keeps the captured address.** `req->address` is fixed when the
   frame is taken, so a write to the address register is answered with the
   address the master used; the new value applies from the next frame on.
3. **Out of range means silent.** A callback result outside 1..247 makes the
   core ignore every frame, instead of making address 0 look like a broadcast.
4. `set_address_pending()`, `apply_config()` and the cached address are
   removed; `get_address()` remains and delegates to the callback.

## Consequences

- A register write, a DIP change and the power-up sequence need no apply step,
  no signal and no timing decision; there is no window to reopen.
- Validation belongs in the register callback: returning
  `ILLEGAL_DATA_VALUE` keeps the old value and answers with an exception.
- One short callback per frame, in the caller's context.
- The library no longer decides when the address changes; that is the
  application's model of its own product.

## Considered: a second receive buffer instead of closing reception

If the core copied a captured frame into another buffer it could reopen
reception immediately, so a delayed `advance()` would not leave reception off.
Rejected:

- It does not change what the master sees. Without `advance()` there is no
  reply; reception being open does not create one.
- The bus is half-duplex: reception is off during the reply anyway, so "never
  close reception" only holds between frames.
- The copy is not free: 256 bytes per channel plus either interrupt time or
  another, shorter closed window.
- If the starvation case ever needs a guard, a timeout on `CAPTURED` costs one
  timer and no RAM.

Recorded as a boundary: the library's contract is "call `advance()` when told";
scheduling is the application's responsibility.
