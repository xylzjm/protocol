# Glossary

Domain terms for the `modbus_slave` stack. Code identifiers are in parentheses.
`README.md` states the contracts; this file fixes the names, so reviews, specs
and commit messages use the same words as the code.

- **ADU** — application data unit. On the wire: `[address][PDU][CRC16]`,
  4 to 256 bytes.
- **PDU** — protocol data unit: the function code and its data, everything
  between the address and the CRC.
- **T3.5** — the inter-frame silence: 3.5 character times, fixed 1.75 ms above
  19200 baud. A frame ends when the bus has been quiet this long.
- **DE** — driver-enable of the RS485 transceiver; the core asserts it only
  while transmitting.
- **TXE / TC** — UART transmit-empty and transmission-complete interrupts: the
  core fills the data register on TXE and closes the frame on TC.
- **Session** (`enum modbus_slave_session`) — the single state the core is in:
  `STOPPED`, `DISCARDING`, `IDLE`, `RECEIVING`, `CAPTURED`, `SENDING`. It
  implies whether reception is on and what the timer means.
- **Transaction** — one received ADU taken to its conclusion: ignored,
  executed as a broadcast, or answered. It starts when the T3.5 interrupt
  freezes reception and ends when reception is open again.
- **Frame advance** (`modbus_slave_advance()`) — the single operation the
  application calls per received-frame event. It runs the waiting transaction
  and returns `NONE`, `HANDLED` or `SENDING`.
- **Reception window** — the span in which the core owns reception switching
  and keeps reception closed, from the T3.5 freeze until the transaction ends
  (at `advance()`, TC, or the watchdog). The application never touches it.
- **TX watchdog** — the one-shot timer armed while a reply is sent. On expiry
  the core aborts the transmission in the timer interrupt, releases DE and
  resynchronizes reception.
- **Resynchronization** — waiting one complete T3.5 of bus silence before
  accepting a byte as the start of a frame. Used at start-up, after an
  overlong frame, and after an aborted transmission.
- **Slave address** — asked from `callbacks.address()` once a frame has passed
  its length and CRC checks, never cached. A product may derive it from a
  writable register plus a DIP-switch offset; a value outside 1..247 silences
  the slave.
- **Application AO** — the context that receives the frame event and calls
  `advance()`. There is no library backend: the AO already serializes events.
- **Hardware capabilities** (`modbus_slave_port_line_t`,
  `modbus_slave_port_timer_t`, `modbus_slave_port_critical_t`) — the board's
  side: the RS485 line (direction, one byte out, end of stream), the one-shot
  timer, and a nesting interrupt mask. Each carries its own context.
- **Event port** (`modbus_slave_port_event_t`) — the core's three outbound
  notifications, called from interrupt context.
- **Broadcast** — slave address 0. Only the four write function codes execute;
  every other broadcast is ignored, and nothing is ever answered.
