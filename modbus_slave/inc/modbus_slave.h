/* SPDX-License-Identifier: MIT */

#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

/**
 * @file modbus_slave.h
 * @brief Modbus RTU slave core: RTU framing, transactions and PDU execution.
 *
 * The core owns the transaction and its reception window; the application
 * calls modbus_slave_advance() once per received frame. See README.md for the
 * board contract and the integration recipe.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

/** @brief Outcome of modbus_slave_advance() for one received frame. */
typedef enum {
    MODBUS_SLAVE_STEP_NONE = 0, /**< Nothing was waiting. */
    MODBUS_SLAVE_STEP_HANDLED,  /**< Consumed; reception is open again. */
    MODBUS_SLAVE_STEP_SENDING   /**< A reply is on the wire. */
} modbus_slave_step_t;

/**
 * @brief The RS485 line: direction, one byte out, end of stream.
 *
 * All three operations are required. direction(rx, tx) must leave the
 * direction it does not select switched off; put_byte() and tx_done() are
 * called from the TXE interrupt.
 */
typedef struct {
    void* ctx;
    modbus_slave_status_t (*direction)(void* ctx,
                                       uint8_t rx,
                                       uint8_t tx);
    modbus_slave_status_t (*put_byte)(void* ctx,
                                      uint8_t byte);
    modbus_slave_status_t (*tx_done)(void* ctx);
} modbus_slave_port_line_t;

/**
 * @brief The one-shot timer behind T3.5 and the TX watchdog.
 *
 * Both operations are required. arm() must also work from the timer's own
 * expiry interrupt and clear a pending match; the expiry reaches the core
 * through modbus_slave_timer_expired().
 */
typedef struct {
    void* ctx;
    modbus_slave_status_t (*arm)(void* ctx,
                                 uint32_t timeout_us);
    modbus_slave_status_t (*cancel)(void* ctx);
} modbus_slave_port_timer_t;

/**
 * @brief Interrupt mask that nests: enter() increments a count, exit()
 * decrements and re-enables interrupts at zero.
 *
 * Both operations are required. The count belongs to the board.
 */
typedef struct {
    void* ctx;
    void (*enter)(void* ctx);
    void (*exit)(void* ctx);
} modbus_slave_port_critical_t;

/**
 * @brief Frame events reported to the application, from interrupt context.
 *
 * Any callback may be NULL; a NULL event is dropped.
 */
typedef struct {
    void* ctx;
    void (*on_frame_received)(void* ctx);
    void (*on_frame_sent)(void* ctx);
    void (*on_tx_timeout)(void* ctx);
} modbus_slave_port_event_t;

/** @brief Direction of a coil/register callback. */
typedef enum {
    MODBUS_SLAVE_REG_READ  = 0, /**< Callback fills the buffer. */
    MODBUS_SLAVE_REG_WRITE = 1  /**< Callback reads the buffer; do not modify it. */
} modbus_slave_register_mode_t;

/**
 * @brief Coil or discrete-input callback.
 *
 * Addresses are 0-based wire addresses; bits are packed LSB-first, unused
 * high bits of the last byte cleared. Return
 * MODBUS_SLAVE_EX_ILLEGAL_DATA_ADDRESS when out of range.
 */
typedef modbus_slave_exception_t (*modbus_slave_bits_fn)(
    void* ctx,
    uint16_t start,
    uint16_t count,
    uint8_t* buffer,
    modbus_slave_register_mode_t mode);

/**
 * @brief Holding- or input-register callback.
 *
 * Addresses are 0-based; the buffer holds @p count big-endian 16-bit registers
 * (use modbus_slave_get_u16() and modbus_slave_put_u16()). Return
 * MODBUS_SLAVE_EX_ILLEGAL_DATA_ADDRESS when out of range.
 */
typedef modbus_slave_exception_t (*modbus_slave_registers_fn)(
    void* ctx,
    uint16_t start,
    uint16_t count,
    uint8_t* buffer,
    modbus_slave_register_mode_t mode);

/**
 * @brief Current slave address, asked before every received frame.
 *
 * Return 1..247; any other value silences the slave.
 */
typedef uint8_t (*modbus_slave_address_fn)(void* ctx);

/** @brief The four data-space callbacks, the address callback and their ctx. */
typedef struct {
    void* ctx;
    modbus_slave_bits_fn coils;
    modbus_slave_bits_fn discrete_inputs;
    modbus_slave_registers_fn holding_registers;
    modbus_slave_registers_fn input_registers;
    modbus_slave_address_fn address;
} modbus_slave_callbacks_t;

/** @brief Reported by function code 0x11 (Report Slave ID). */
typedef struct {
    uint8_t slave_id;
    uint8_t is_running;         /**< Non-zero: the Running indicator is ON. */
    const uint8_t* additional;  /**< Extra bytes; may be NULL when unused. */
    uint16_t additional_length; /**< <= 249. */
} modbus_slave_identity_t;

/**
 * @brief Construction configuration, copied by modbus_slave_ctor().
 *
 * baudrate > 0; rx_buffer and tx_buffer are caller-owned, at least 256 bytes
 * each, and disjoint.
 */
typedef struct {
    uint32_t baudrate;
    uint8_t* rx_buffer;
    uint16_t rx_capacity;
    uint8_t* tx_buffer;
    uint16_t tx_capacity;
    modbus_slave_port_line_t line;
    modbus_slave_port_timer_t timer;
    modbus_slave_port_critical_t critical;
    modbus_slave_callbacks_t callbacks;
    modbus_slave_identity_t identity;
} modbus_slave_config_t;

/**
 * @brief Slave instance. The caller owns the storage; members are private.
 *
 * Public so instances can live in static arrays without dynamic allocation.
 */
typedef struct modbus_slave {
    modbus_slave_port_line_t line;
    modbus_slave_port_timer_t timer;
    modbus_slave_port_critical_t critical;
    modbus_slave_callbacks_t callbacks;
    modbus_slave_identity_t identity;

    uint8_t* rx_buffer;
    uint16_t rx_capacity;
    uint8_t* tx_buffer;
    uint16_t tx_capacity;

    uint32_t baudrate;
    uint32_t t35_us;

    modbus_slave_port_event_t event;

    uint8_t session;
    uint16_t rx_pos;
    uint16_t tx_len;
    uint16_t tx_pos;
} modbus_slave_t;

/**
 * @brief Construct a slave instance. No hardware is touched.
 *
 * Stop a running instance first. Returns MODBUS_SLAVE_ERR_ARG on a NULL
 * pointer, baudrate 0, a buffer smaller than 256 bytes or overlapping, a
 * missing port operation or callback, or identity.additional_length > 249.
 */
modbus_slave_status_t modbus_slave_ctor(modbus_slave_t* me,
                                        const modbus_slave_config_t* config);

/**
 * @brief Attach the application's event callbacks.
 *
 * Allowed while stopped; returns MODBUS_SLAVE_ERR_STATE once started. The
 * struct is copied, and @p event_port may be NULL to detach.
 */
modbus_slave_status_t modbus_slave_set_event_port(
    modbus_slave_t* me,
    const modbus_slave_port_event_t* event_port);

/**
 * @brief Open reception and wait one T3.5 of bus idle.
 *
 * Idempotent, and safe as the entry action of the application's serving
 * state. Returns MODBUS_SLAVE_ERR_STATE while a reply is on the wire, or
 * MODBUS_SLAVE_ERR_IO when a capability fails (a failed first start is
 * rolled back).
 */
modbus_slave_status_t modbus_slave_start(modbus_slave_t* me);

/**
 * @brief Close reception and the timer; the instance returns to STOPPED.
 *
 * Returns MODBUS_SLAVE_ERR_IO when a capability fails; the state is reset
 * either way.
 */
modbus_slave_status_t modbus_slave_stop(modbus_slave_t* me);

/** @brief RX interrupt: feed one received byte. */
modbus_slave_status_t modbus_slave_rx_byte(modbus_slave_t* me,
                                           uint8_t byte);

/** @brief Timer interrupt: T3.5 expiry or TX watchdog. */
modbus_slave_status_t modbus_slave_timer_expired(modbus_slave_t* me);

/** @brief TXE interrupt: emit the next byte or mute the request. */
modbus_slave_status_t modbus_slave_tx_ready(modbus_slave_t* me);

/** @brief TC interrupt: close TX, reopen RX, notify on_frame_sent. */
modbus_slave_status_t modbus_slave_tx_complete(modbus_slave_t* me);

/**
 * @brief Take the captured frame to its conclusion: validate, execute,
 * reply.
 *
 * Call once per on_frame_received event, in the application's context. The
 * reception window stays inside the core: every outcome reopens it or hands
 * it to the TX path.
 *
 * @return MODBUS_SLAVE_STEP_NONE if no frame was waiting (stale event or
 * stopped), MODBUS_SLAVE_STEP_HANDLED if consumed with reception open,
 * MODBUS_SLAVE_STEP_SENDING if a reply is on the wire.
 */
modbus_slave_step_t modbus_slave_advance(modbus_slave_t* me);

/**
 * @brief Current slave address, as the address callback reports it.
 *
 * Returns 0 when @p me is NULL.
 */
uint8_t modbus_slave_get_address(const modbus_slave_t* me);

/** @brief Set the "Running" indicator reported by function code 0x11. */
void modbus_slave_set_running(modbus_slave_t* me,
                              uint8_t is_running);

/** @brief Read a big-endian 16-bit register value. */
uint16_t modbus_slave_get_u16(const uint8_t* buffer);

/** @brief Write a big-endian 16-bit register value. */
void modbus_slave_put_u16(uint8_t* buffer,
                          uint16_t value);

#ifdef __cplusplus
}
#endif

#endif
