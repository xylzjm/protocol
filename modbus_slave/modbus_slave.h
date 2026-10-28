/* SPDX-License-Identifier: MIT */

#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

/**
 * @file modbus_slave.h
 * @brief Modbus RTU slave protocol stack core, pure C99.
 *
 * The core is a transport-framing and PDU-execution engine. It contains the
 * RTU RX/TX state machines, the T3.5 inter-frame timer, CRC16, the request
 * validators for the supported function codes, and the response builders.
 *
 * The core does not own hardware and does not schedule itself:
 *
 * - The board supplies hardware capabilities through modbus_slave_port_t
 *   (serial direction control, byte output, one-shot timer, critical
 *   sections), passed to modbus_slave_ctor().
 * - A server backend (QP, bare metal loop, RTOS task, ...) drives the core:
 *   it receives frame events through modbus_slave_event_port_t and then
 *   calls modbus_slave_receive()/modbus_slave_execute()/modbus_slave_send()
 *   in its own context.
 *
 * The core includes only <stdint.h> and <stddef.h>; it never includes QP,
 * an RTOS, or a vendor header. Multiple instances may coexist: there is no
 * mutable global state.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Result / exception codes ------------------------------------------------ */

/** @brief Result of a core operation. */
typedef enum {
    MODBUS_SLAVE_OK        = 0,
    MODBUS_SLAVE_ERR_ARG   = -1,
    MODBUS_SLAVE_ERR_STATE = -2,
    MODBUS_SLAVE_ERR_IO    = -3
} modbus_slave_status_t;

/** @brief Modbus exception codes used in exception responses. */
typedef enum {
    MODBUS_SLAVE_EX_NONE                 = 0x00,
    MODBUS_SLAVE_EX_ILLEGAL_FUNCTION     = 0x01,
    MODBUS_SLAVE_EX_ILLEGAL_DATA_ADDRESS = 0x02,
    MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE   = 0x03,
    MODBUS_SLAVE_EX_SLAVE_DEVICE_FAILURE = 0x04,
    MODBUS_SLAVE_EX_ACKNOWLEDGE          = 0x05,
    MODBUS_SLAVE_EX_SLAVE_BUSY           = 0x06,
    MODBUS_SLAVE_EX_MEMORY_PARITY_ERROR  = 0x08,
    MODBUS_SLAVE_EX_GATEWAY_PATH_FAILED  = 0x0A,
    MODBUS_SLAVE_EX_GATEWAY_TGT_FAILED   = 0x0B
} modbus_slave_exception_t;

/** @brief Outcome of modbus_slave_receive() for one received ADU. */
typedef enum {
    MODBUS_SLAVE_RX_IGNORE = 0, /**< CRC/address/length error, broadcast read, broadcast 0x11. */
    MODBUS_SLAVE_RX_UNICAST,    /**< Valid request that expects a response. */
    MODBUS_SLAVE_RX_BROADCAST   /**< Valid broadcast write; execute, never respond. */
} modbus_slave_rx_status_t;

/* Hardware capability interface (board-provided) -------------------------- */

/**
 * @brief Hardware operations the core needs from the board.
 *
 * All six operations are required: modbus_slave_ctor() rejects a port that
 * leaves any of them NULL.
 *
 * - serial_enable(rx, tx): rx=1 enables reception, releases the RS485
 *   driver-enable (DE), and disables the TXE and TC interrupts (clearing a
 *   pending TC), so a finished transmission cannot keep interrupting;
 *   tx=1 asserts DE, clears TC and enables the TXE+TC interrupts; (0,0)
 *   disables both directions and releases DE. Each call must leave the
 *   direction it does not select switched off.
 * - put_byte(byte): writes one byte to the UART data register, non-blocking,
 *   called only from the TXE interrupt.
 * - timer_enable(timeout_us): arms the one-shot inter-frame/TX timer; when it
 *   expires the board must call modbus_slave_timer_expired(). At low baud
 *   rates the timeout can reach several seconds, so the timer has to support
 *   long durations.
 * - timer_disable(): stops the timer and clears a pending expiry.
 * - critical_enter()/critical_exit(state): save and disable interrupts,
 *   restore the saved state. The token must nest correctly (PRIMASK/BASEPRI
 *   on QV/QK, portSET/portCLEAR_INTERRUPT_MASK_FROM_ISR on FreeRTOS).
 */
typedef struct {
    void* ctx;
    modbus_slave_status_t (*serial_enable)(void* ctx,
                                           uint8_t rx,
                                           uint8_t tx);
    modbus_slave_status_t (*put_byte)(void* ctx,
                                      uint8_t byte);
    modbus_slave_status_t (*timer_enable)(void* ctx,
                                          uint32_t timeout_us);
    modbus_slave_status_t (*timer_disable)(void* ctx);
    uint32_t (*critical_enter)(void* ctx);
    void (*critical_exit)(void* ctx,
                          uint32_t state);
} modbus_slave_port_t;

/* Event interface (server-backend-provided) ------------------------------- */

/**
 * @brief Frame-level events the core reports to its server backend.
 *
 * All three functions are called from interrupt context: on_frame_received
 * from the T3.5 expiry (receive is already switched off), on_frame_sent from
 * the TC interrupt, on_tx_timeout from the TX-timeout expiry. A backend that
 * cannot work in interrupt context must defer, for example by posting an
 * event to its active object or task.
 *
 * Any of the three function pointers may be NULL; a NULL event is dropped.
 */
typedef struct {
    void* ctx;
    void (*on_frame_received)(void* ctx);
    void (*on_frame_sent)(void* ctx);
    void (*on_tx_timeout)(void* ctx);
} modbus_slave_event_port_t;

/* Register/coil callbacks ------------------------------------------------- */

/** @brief Direction of a coil/register callback. */
typedef enum {
    MODBUS_SLAVE_REG_READ  = 0, /**< Callback fills the buffer. */
    MODBUS_SLAVE_REG_WRITE = 1  /**< Callback reads (and must not modify) the buffer. */
} modbus_slave_register_mode_t;

/**
 * @brief Coil or discrete-input callback.
 *
 * Addresses are 0-based wire addresses; no +1 adjustment happens anywhere.
 * In read mode the callback writes @p count packed bits into @p buffer,
 * LSB first, unused high bits of the last byte cleared. In write mode the
 * callback reads @p count packed bits from @p buffer in the same layout.
 * Out-of-range addresses must return MODBUS_SLAVE_EX_ILLEGAL_DATA_ADDRESS.
 */
typedef modbus_slave_exception_t (*modbus_slave_bits_fn)(
    void* ctx,
    uint16_t start,
    uint16_t count,
    uint8_t* buffer,
    modbus_slave_register_mode_t mode);

/**
 * @brief Holding/input register callback.
 *
 * Addresses are 0-based wire addresses. The buffer carries registers in wire
 * order, big-endian, 2 bytes each (use modbus_slave_get_u16() and
 * modbus_slave_put_u16() to convert). In read mode the callback fills
 * 2 * @p count bytes; in write mode it reads them and must not modify them.
 * Out-of-range addresses must return
 * MODBUS_SLAVE_EX_ILLEGAL_DATA_ADDRESS.
 *
 * A callback runs synchronously inside the server backend's context; it has
 * to be short and must not block. When the data is not ready, return
 * MODBUS_SLAVE_EX_SLAVE_BUSY or MODBUS_SLAVE_EX_SLAVE_DEVICE_FAILURE: the
 * stack never defers a response to another context.
 */
typedef modbus_slave_exception_t (*modbus_slave_registers_fn)(
    void* ctx,
    uint16_t start,
    uint16_t count,
    uint8_t* buffer,
    modbus_slave_register_mode_t mode);

/** @brief The four data-space callbacks plus their shared context. */
typedef struct {
    void* ctx;
    modbus_slave_bits_fn coils;                  /**< Read/write. */
    modbus_slave_bits_fn discrete_inputs;        /**< Read-only. */
    modbus_slave_registers_fn holding_registers; /**< Read/write. */
    modbus_slave_registers_fn input_registers;   /**< Read-only. */
} modbus_slave_callbacks_t;

/* Configuration / instance ----------------------------------------------- */

/** @brief Reported by function code 0x11 (Report Slave ID). */
typedef struct {
    uint8_t slave_id;
    uint8_t is_running;         /**< Non-zero: the "Running" indicator is ON. */
    const uint8_t* additional;  /**< Optional extra bytes, may be NULL if length is 0. */
    uint16_t additional_length; /**< <= 249, see MODBUS_SLAVE_ADDITIONAL_MAX. */
} modbus_slave_identity_t;

/**
 * @brief Construction configuration, copied by modbus_slave_ctor().
 *
 * rx_buffer and tx_buffer are owned by the caller, must each hold at least
 * 256 bytes, and must not overlap.
 */
typedef struct {
    uint8_t address;   /**< 1..247. */
    uint32_t baudrate; /**< > 0. */
    uint8_t* rx_buffer;
    uint16_t rx_capacity; /**< >= 256. */
    uint8_t* tx_buffer;
    uint16_t tx_capacity;     /**< >= 256. */
    modbus_slave_port_t port; /**< Board-provided hardware capabilities. */
    modbus_slave_callbacks_t callbacks;
    modbus_slave_identity_t identity;
} modbus_slave_config_t;

/**
 * @brief Slave instance. The caller owns the storage; members are private.
 *
 * The struct is public so instances can live in static arrays (for example
 * in a board module) without dynamic allocation. Only the functions declared
 * in this header may touch the members.
 */
typedef struct modbus_slave {
    /* ---- copied at construction ---- */
    modbus_slave_port_t port;           /**< Board-provided hardware capabilities. */
    modbus_slave_callbacks_t callbacks; /**< Four data-space callbacks + ctx. */
    modbus_slave_identity_t identity;   /**< 0x11 report data. */

    uint8_t* rx_buffer; /**< Caller-owned, >= 256 bytes. */
    uint16_t rx_capacity;
    uint8_t* tx_buffer; /**< Caller-owned, >= 256 bytes, disjoint from rx_buffer. */
    uint16_t tx_capacity;

    uint32_t baudrate; /**< Fixed after construction. */
    uint32_t t35_us;   /**< T3.5 derived from baudrate. */

    uint8_t address;         /**< Current slave address, 1..247. */
    uint8_t pending_address; /**< Address waiting to take effect. */
    uint8_t address_pending; /**< pending_address is valid. */

    /* ---- server backend attachment ---- */
    modbus_slave_event_port_t event_port;

    /* ---- runtime state (private) ---- */
    uint8_t rx_state;   /**< modbus_slave_rx_state_t. */
    uint8_t tx_state;   /**< modbus_slave_tx_state_t. */
    uint8_t timer_mode; /**< modbus_slave_timer_mode_t. */

    uint16_t rx_pos;       /**< Bytes of the current ADU in rx_buffer. */
    uint16_t pdu_len;      /**< Request PDU length. */
    uint16_t response_len; /**< Built response PDU length; 0 when none is pending. */
    uint16_t tx_len;       /**< ADU length being transmitted. */
    uint16_t tx_pos;       /**< Index of the next byte to transmit. */

    uint8_t request_address; /**< Address of the current request. */
    uint8_t function;        /**< Current function code. */
    uint8_t broadcast;       /**< Current request is a broadcast. */
    uint8_t started;         /**< modbus_slave_start() has run. */
} modbus_slave_t;

/* API --------------------------------------------------------------------- */

/**
 * @brief Construct a slave instance.
 *
 * Copies the configuration into @p me and derives T3.5 from the baudrate.
 * No hardware is touched. The instance must not be running: call
 * modbus_slave_stop() first if it is.
 *
 * Returns MODBUS_SLAVE_ERR_ARG when @p me or
 * @p config is NULL, the address is outside 1..247, the baudrate is 0, a
 * buffer is NULL or smaller than 256 bytes, the two buffers overlap, any of
 * the six port operations is NULL, any of the four callbacks is NULL, or the
 * identity has additional_length > 249 (or a non-NULL length with a NULL
 * pointer).
 */
modbus_slave_status_t modbus_slave_ctor(modbus_slave_t* me,
                                        const modbus_slave_config_t* config);

/**
 * @brief Attach the server backend's event callbacks.
 *
 * Must be called before modbus_slave_start(); afterwards it returns
 * MODBUS_SLAVE_ERR_STATE. Calling it again after modbus_slave_stop() is
 * allowed and replaces the port. The struct is copied. @p event_port may be
 * NULL, which detaches any previously attached port: the core keeps working
 * but drops all events.
 */
modbus_slave_status_t modbus_slave_set_event_port(
    modbus_slave_t* me,
    const modbus_slave_event_port_t* event_port);

/**
 * @brief Start the slave: open reception and resynchronize on bus idle.
 *
 * Idempotent; a repeat call re-opens reception, which makes it safe as the
 * entry action of a backend's "idle" state. The RX frame machine waits one
 * T3.5 of bus idle before accepting a frame, so a frame already in flight
 * when the slave starts cannot be mistaken for a complete one.
 *
 * Returns MODBUS_SLAVE_ERR_IO when the port failed to switch the serial
 * direction or to arm the inter-frame timer. The instance then stays
 * stopped (a first start is rolled back); call stop() and retry.
 */
modbus_slave_status_t modbus_slave_start(modbus_slave_t* me);

/**
 * @brief Stop the slave: close serial and timer, reset the machines.
 *
 * The RX machine returns to its initial state, so a later start()
 * resynchronizes on bus idle again.
 *
 * Returns MODBUS_SLAVE_ERR_IO when the port failed to stop the timer or the
 * serial port; the software state is reset either way, so the caller may
 * still start() again.
 */
modbus_slave_status_t modbus_slave_stop(modbus_slave_t* me);

/* ISR entry points (board interrupt glue) --------------------------------- */

/** Feed one received byte into the RX machine. Call from the RX interrupt. */
modbus_slave_status_t modbus_slave_rx_byte(modbus_slave_t* me,
                                           uint8_t byte);

/** Report the one-shot timer expiry (T3.5 or TX timeout). Call from the timer ISR. */
modbus_slave_status_t modbus_slave_timer_expired(modbus_slave_t* me);

/** Report UART TXE. The core emits one byte or ignores the interrupt. */
modbus_slave_status_t modbus_slave_tx_ready(modbus_slave_t* me);

/** Report UART TC (transmission complete). Closes TX, reopens RX, notifies the backend. */
modbus_slave_status_t modbus_slave_tx_complete(modbus_slave_t* me);

/* Primitives for the server backend --------------------------------------- */

/**
 * @brief Validate the received ADU and capture the request.
 *
 * Checks CRC, length and address, and applies the broadcast policy: only
 * write functions (0x05, 0x06, 0x0F, 0x10) are valid broadcasts. Returns
 * MODBUS_SLAVE_RX_IGNORE for anything that must be dropped silently,
 * MODBUS_SLAVE_RX_BROADCAST for an accepted broadcast write (execute it but
 * never respond), and MODBUS_SLAVE_RX_UNICAST for a valid addressed request.
 * Leaves the RX machine in its idle state either way; the backend re-opens
 * reception with modbus_slave_enable_rx() for IGNORE and BROADCAST results.
 */
modbus_slave_rx_status_t modbus_slave_receive(modbus_slave_t* me);

/**
 * @brief Execute the captured request and build the response PDU.
 *
 * Runs the data callbacks for the matching function code and writes the
 * response PDU (or exception response) to the transmit buffer. For a
 * broadcast the response is built but must be discarded: modbus_slave_send()
 * refuses it, and the backend re-opens reception instead.
 */
void modbus_slave_execute(modbus_slave_t* me);

/**
 * @brief Wrap the response PDU in an ADU and start transmitting it.
 *
 * Prepends the slave address, appends the CRC and switches the serial port
 * to transmit. Transmission is driven by the TXE/TC interrupts. The built
 * response is consumed by the call, so a second send() without a fresh
 * execute() is rejected.
 *
 * Returns MODBUS_SLAVE_ERR_STATE when a transmission is already in progress,
 * no response has been built, or the pending request was a broadcast (a
 * broadcast is never answered, even if execute() built a PDU for it).
 */
modbus_slave_status_t modbus_slave_send(modbus_slave_t* me);

/** Abandon an ongoing transmission and release the bus. Reception re-opens. */
void modbus_slave_abort_tx(modbus_slave_t* me);

/** Re-enable reception (and release DE). No-op before start(). */
void modbus_slave_enable_rx(modbus_slave_t* me);

/** Disable reception and assert nothing else; used around configuration changes. */
void modbus_slave_disable_rx(modbus_slave_t* me);

/**
 * @brief Store a new slave address; it takes effect at the next apply.
 *
 * Values outside 1..247 are ignored. Call before posting a configuration
 * event to the backend; the backend applies it in its idle state.
 */
void modbus_slave_set_address_pending(modbus_slave_t* me,
                                      uint8_t address);

/** Apply the pending address, if any. Backends call this in their idle entry. */
void modbus_slave_apply_config(modbus_slave_t* me);

/** Current effective slave address. */
uint8_t modbus_slave_get_address(const modbus_slave_t* me);

/** Set the "Running" indicator reported by function code 0x11. */
void modbus_slave_set_running(modbus_slave_t* me,
                              uint8_t is_running);

/* Wire-order helpers ------------------------------------------------------ */

/** Read a big-endian 16-bit value, as used by Modbus registers on the wire. */
uint16_t modbus_slave_get_u16(const uint8_t* buffer);

/** Write a big-endian 16-bit value, as used by Modbus registers on the wire. */
void modbus_slave_put_u16(uint8_t* buffer,
                          uint16_t value);

#ifdef __cplusplus
}
#endif

#endif
