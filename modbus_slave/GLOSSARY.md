# Glossary

Domain terms for the `modbus_slave` stack. Code identifiers are in parentheses.
`README.md` states the contracts; this file fixes the names, so reviews, specs
and commit messages use the same words as the code.

- **ADU** — application data unit. On the wire: `[address][PDU][CRC16]`,
  4 to 256 bytes.
- **PDU** — protocol data unit: the function code and its data, everything
  between the address and the CRC.
- **Transaction** — one received ADU taken to its conclusion: ignored,
  executed as a broadcast, or answered. It starts when the T3.5 interrupt
  freezes reception and ends when reception is open again.
- **Frame advance** (`modbus_slave_advance()`) — the single operation a
  backend calls per received frame. It runs one transaction and returns
  `NONE`, `HANDLED` or `SENDING`.
- **Reception window** — the period during which the core keeps reception
  switched off. The core opens and closes it; no backend touches it.
- **Resynchronization** — waiting one complete T3.5 of bus silence before
  accepting a byte as the start of a frame. Used at start-up, after an
  overlong frame, after an aborted transmission, and around a reconfiguration.
- **Backend** (`backend/qp`, planned `bare`/`freertos`) — the module that
  drives the core: it receives frame events and calls `advance()`.
- **Hardware port** (`modbus_slave_port_t`) — the board's capabilities:
  line direction, one byte out, one-shot timer, critical section.
- **Event port** (`modbus_slave_event_port_t`) — the core's three outbound
  notifications, called from interrupt context.
- **Broadcast** — slave address 0. Write function codes execute; nothing is
  ever answered.
