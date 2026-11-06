/* SPDX-License-Identifier: MIT */

#ifndef I2C_SOFT_H
#define I2C_SOFT_H

/**
 * @file i2c_soft.h
 * @brief Bit-banged I2C master: pins, bit timing and interrupt masking are
 *        injected through ports.
 *
 * The library owns the wire protocol only - START, repeated START, STOP,
 * byte shifting, acknowledge slots and bus recovery. The board owns the two
 * pins, the delay and the critical section and supplies them in
 * i2c_soft_config_t, so this header needs nothing but <stddef.h> and
 * <stdint.h>: no MCU, no vendor header, no RTOS. Every call blocks for the
 * whole transfer; see README.md for the port contracts, the timing and the
 * limits.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Result of an operation. */
typedef enum {
    I2C_SOFT_OK       = 0,
    I2C_SOFT_ERR_ARG  = -1, /**< NULL, zero length or reserved address. */
    I2C_SOFT_ERR_NACK = -2, /**< Address or write data was not acknowledged. */
    I2C_SOFT_ERR_BUS  = -3  /**< SCL low, or SDA still low after recovery. */
} i2c_soft_status_t;

/**
 * @brief The two pins, their readback and the bit delay.
 *
 * All five operations are required; i2c_soft_ctor() rejects a configuration
 * that leaves any of them NULL. Writing 1 releases the line and writing 0
 * pulls it low, so the board must configure both pins open-drain with a
 * pull-up: the library never drives a line high. delay_us() must delay at
 * least the requested time while interrupts are masked, because the library
 * calls it inside a critical section: a delay that waits for a tick
 * interrupt would hang there.
 */
typedef struct {
    void* ctx;                                   /**< Board context of every operation. */
    void (*sda_write)(void* ctx, uint8_t level); /**< 0 pulls low, 1 releases. */
    void (*scl_write)(void* ctx, uint8_t level); /**< 0 pulls low, 1 releases. */
    uint8_t (*sda_read)(void* ctx);              /**< Line level: 1 = high. */
    uint8_t (*scl_read)(void* ctx);              /**< Line level: 1 = high. */
    void (*delay_us)(void* ctx, uint32_t us);    /**< Delay at least us. */
} i2c_soft_port_t;

/**
 * @brief A nesting interrupt mask, held for one edge group at a time.
 *
 * Both operations are required. enter() masks interrupts and increments the
 * nesting count; exit() decrements and unmasks only when the count reaches
 * zero. Nesting is what lets a caller wrap its own critical section around a
 * transaction while the library's inner pairs simply balance out. Wire both
 * to the counter the rest of the firmware uses - with QP that is
 * QF_CRIT_ENTRY()/QF_CRIT_EXIT() - rather than to a save/restore token.
 */
typedef struct {
    void* ctx;
    void (*enter)(void* ctx);
    void (*exit)(void* ctx);
} i2c_soft_port_critical_t;

/** @brief Construction input: the GPIO port and the critical port. */
typedef struct {
    i2c_soft_port_t port;              /**< Pins, readback and bit delay. */
    i2c_soft_port_critical_t critical; /**< Nesting interrupt mask. */
} i2c_soft_config_t;

/**
 * @brief One bus instance; the storage belongs to the caller.
 *
 * Static allocation only: there is no dynamic memory and no mutable global
 * state, so any number of instances coexist. The members are private: build
 * the instance with i2c_soft_ctor() and leave them alone afterwards.
 */
typedef struct {
    i2c_soft_port_t port;              /**< Private; set by i2c_soft_ctor(). */
    i2c_soft_port_critical_t critical; /**< Private; set by i2c_soft_ctor(). */
} i2c_soft_t;

/**
 * @brief Half SCL period in microseconds (5 us, nominally 100 kHz).
 *
 * Define it to another value before including this header to change the bit
 * rate; 5 us is a safe starting point for standard mode. Every half period
 * is spent in delay_us(), so the real SCL frequency also carries the port's
 * call overhead: the library does not promise an exact 100 kHz, and a value
 * far below 5 us breaks the I2C timing rules.
 */
#ifndef I2C_SOFT_HALF_PERIOD_US
#define I2C_SOFT_HALF_PERIOD_US 5U
#endif

/**
 * @brief Build an instance in RAM from a configuration.
 *
 * Copies the two ports and returns: no port operation runs and no GPIO is
 * touched, so construction cannot disturb a bus that is already running.
 *
 * @param me Instance to build; must not be NULL.
 * @param config Ports to copy; every operation must be non-NULL.
 * @return I2C_SOFT_OK, or I2C_SOFT_ERR_ARG for a NULL argument or a missing
 *         operation.
 * @note Construction only: i2c_soft_init() must still run before the first
 *       transaction.
 */
i2c_soft_status_t i2c_soft_ctor(i2c_soft_t* me,
                                const i2c_soft_config_t* config);

/**
 * @brief Release the bus, check it, and recover a stuck SDA if needed.
 *
 * Releases SDA and SCL and waits one half period, then: a low SCL returns
 * I2C_SOFT_ERR_BUS without a single pulse; a low SDA runs one
 * i2c_soft_bus_recover(); afterwards both lines must read high.
 *
 * @param me Instance built by i2c_soft_ctor(); must not be NULL.
 * @return I2C_SOFT_OK when the bus is idle-high, I2C_SOFT_ERR_ARG for a NULL
 *         instance, or I2C_SOFT_ERR_BUS when SCL is low or SDA stays low.
 * @note Blocking; task context only, never an ISR. Call it explicitly before
 *       the first transaction (the transaction APIs never initialize
 *       implicitly) and as often as needed afterwards.
 */
i2c_soft_status_t i2c_soft_init(i2c_soft_t* me);

/**
 * @brief Write one message: START, address+W, data, STOP.
 *
 * @param me Initialized instance.
 * @param addr7 7-bit slave address, 0x01..0x77.
 * @param data Bytes to send; must stay valid for the whole call.
 * @param len Number of bytes; must be greater than zero.
 * @return I2C_SOFT_OK, I2C_SOFT_ERR_ARG for a bad argument, or
 *         I2C_SOFT_ERR_NACK when the address or a byte is NACKed; a STOP is
 *         sent first and the remaining bytes are not.
 * @note Blocking; task context only. The library is not thread-safe: the
 *       caller serializes access to an instance and to the bus.
 */
i2c_soft_status_t i2c_soft_write(i2c_soft_t* me,
                                 uint8_t addr7,
                                 const uint8_t* data,
                                 size_t len);

/**
 * @brief Read one message: START, address+R, data, STOP.
 *
 * The master acknowledges every byte but the last, which it leaves NACKed
 * before the STOP.
 *
 * @param me Initialized instance.
 * @param addr7 7-bit slave address, 0x01..0x77.
 * @param data Output buffer; must stay valid for the whole call.
 * @param len Number of bytes; must be greater than zero.
 * @return I2C_SOFT_OK, I2C_SOFT_ERR_ARG for a bad argument, or
 *         I2C_SOFT_ERR_NACK when the address is NACKed (a STOP is sent).
 * @note Blocking; task context only.
 */
i2c_soft_status_t i2c_soft_read(i2c_soft_t* me,
                                uint8_t addr7,
                                uint8_t* data,
                                size_t len);

/**
 * @brief Combined transfer: START, address+W, write, repeated START,
 *        address+R, read, STOP (the usual register read).
 *
 * @param me Initialized instance.
 * @param addr7 7-bit slave address, 0x01..0x77.
 * @param wr Bytes to send first; must stay valid for the whole call.
 * @param wr_len Write length; must be greater than zero.
 * @param rd Buffer for the read phase; must stay valid for the whole call.
 * @param rd_len Read length; must be greater than zero.
 * @return I2C_SOFT_OK, I2C_SOFT_ERR_ARG for a bad argument, or
 *         I2C_SOFT_ERR_NACK on the first NACKed address or write byte; a
 *         STOP is sent, no repeated START follows and rd is not touched.
 * @note Blocking; task context only.
 */
i2c_soft_status_t i2c_soft_write_read(i2c_soft_t* me,
                                      uint8_t addr7,
                                      const uint8_t* wr,
                                      size_t wr_len,
                                      uint8_t* rd,
                                      size_t rd_len);

/**
 * @brief Address-only probe: START, address+W, acknowledge check, STOP.
 *
 * @param me Initialized instance.
 * @param addr7 7-bit slave address, 0x01..0x77.
 * @return I2C_SOFT_OK when the slave acknowledges, I2C_SOFT_ERR_NACK when it
 *         does not (a STOP is sent either way), or I2C_SOFT_ERR_ARG for a
 *         bad argument.
 * @note Blocking; task context only. Probe answers whether a device is on
 *       the bus; it carries no data, so a register read still needs
 *       i2c_soft_write_read().
 */
i2c_soft_status_t i2c_soft_probe(i2c_soft_t* me,
                                 uint8_t addr7);

/**
 * @brief Free an SDA held low by clocking at most nine pulses and a STOP.
 *
 * Releases both lines first: a low SCL is only reported, never forced.
 * Software cannot make a rising edge while a slave holds the clock down, and
 * driving against it would make two outputs fight.
 *
 * @param me Initialized instance.
 * @return I2C_SOFT_OK when SDA is high afterwards, I2C_SOFT_ERR_ARG for a
 *         NULL instance, or I2C_SOFT_ERR_BUS when SCL is low or SDA is still
 *         low after the pulses.
 * @note Blocking; task context only. Transactions never recover on their
 *       own: after I2C_SOFT_ERR_NACK the caller decides whether to call
 *       this.
 */
i2c_soft_status_t i2c_soft_bus_recover(i2c_soft_t* me);

#ifdef __cplusplus
}
#endif

#endif /* I2C_SOFT_H */
