/* SPDX-License-Identifier: GPL-3.0-or-later */

#ifndef MODBUS_SLAVE_QP_H
#define MODBUS_SLAVE_QP_H

/**
 * @file modbus_slave_qp.h
 * @brief QP/C (QHsm) backend for the modbus_slave core.
 *
 * The backend is a QHsm component, not an active object: the application AO
 * of each channel owns it and dispatches protocol events into it with
 * QASM_DISPATCH(). The backend drives the pure-C core
 * (modbus_slave_t) through the receive/execute/send primitives.
 *
 * Because this component includes qpc.h (QP is GPL-3.0-or-later or
 * commercial), it is licensed GPL-3.0-or-later, unlike the MIT core.
 *
 * Application usage:
 *
 * @code
 * typedef struct {
 *     QActive super;                    // offset 0
 *     modbus_slave_qp_t backend;        // QHsm, holds the core pointer
 * } ModbusApp;
 *
 * // in the app state handling MB_* signals:
 * QASM_DISPATCH(&me->backend.super, e, 0u);
 * @endcode
 *
 * The backend also contributes a block of six signals (MB_*_SIG). The
 * project's single QP signal header (main.h) composes that block with the
 * application's own signals; see MODBUS_SLAVE_QP_SIG_BASE and
 * modbus_slave_qp_signal_t below.
 *
 * The AO's event queue must hold at least 8 events so the three ISR-posted
 * protocol events can never overflow it (QF_NO_MARGIN).
 */

#include "modbus_slave.h"
#include "qpc.h"

/**
 * @brief First value of this backend's signal block.
 *
 * Assign it in the build's global macros (the compiler's -D definition), so
 * every translation unit -- including backend/qp/modbus_slave_qp.c, which
 * includes this header directly -- sees the same value. The default keeps
 * the block at Q_USER_SIG.
 *
 * The project's signal header (main.h) then includes the block headers in
 * the same order; the next block, or the application enum, starts at
 * MB_SIG_NEXT:
 *
 * @code
 * // build macro: -DMODBUS_SLAVE_QP_SIG_BASE=Q_USER_SIG
 *
 * // main.h
 * #include "qpc.h"
 * #include "modbus_slave_qp.h"      // MB_*_SIG at MODBUS_SLAVE_QP_SIG_BASE
 *
 * enum AppSignals {
 *     SENSOR_DATA_SIG = MB_SIG_NEXT,
 *     SENSOR_FAULT_SIG,
 * };
 * @endcode
 *
 * Do not #define it in main.h: a header-only definition reaches only the
 * files that include main.h, not the backend's own sources.
 */
#ifndef MODBUS_SLAVE_QP_SIG_BASE
#define MODBUS_SLAVE_QP_SIG_BASE (Q_USER_SIG)
#endif

/**
 * @brief Signals the application AO must dispatch into the backend HSM.
 *
 * The block occupies six consecutive values. The project's signal header
 * (typically main.h) includes this header; the next block, or the application
 * enum, starts at MB_SIG_NEXT, so every signal in the firmware has a unique
 * value:
 *
 * @code
 * // main.h
 * #include "qpc.h"
 * #include "modbus_slave_qp.h"
 *
 * enum AppSignals {
 *     SENSOR_DATA_SIG = MB_SIG_NEXT,
 *     SENSOR_FAULT_SIG,
 * };
 * @endcode
 *
 * MB_FRAME_RCVED_SIG / MB_FRAME_SENT_SIG / MB_TX_TIMEOUT_SIG are posted by
 * the backend's event port from interrupt context; MB_START_SIG /
 * MB_STOP_SIG / MB_CONFIG_SIG are posted by the application.
 */
typedef enum {
    MB_FRAME_RCVED_SIG = MODBUS_SLAVE_QP_SIG_BASE, /**< A frame completed; process it. */
    MB_FRAME_SENT_SIG,                             /**< The response went out on the wire. */
    MB_TX_TIMEOUT_SIG,                             /**< Transmission watchdog expired. */
    MB_START_SIG,                                  /**< Enter service: apply pending config. */
    MB_STOP_SIG,                                   /**< Leave service. */
    MB_CONFIG_SIG,                                 /**< Apply the pending configuration. */

    /** First value after this block; the next block or the app starts here. */
    MB_SIG_NEXT
} modbus_slave_qp_signal_t;

/**
 * @brief Backend instance, embedded in the application AO.
 *
 * Members are private; only the functions declared here may touch them.
 */
typedef struct {
    QHsm super;            /**< QP state machine, must stay at offset 0. */
    modbus_slave_t* slave; /**< Core instance owned by the board module. */
    QActive* app;          /**< This channel's AO, receives the events. */
    uint8_t last_error;    /**< Saturating count of TX timeouts; see tx_timeout_count(). */
    uint8_t fault;         /**< Non-zero: construction failed; never start the core. */
} modbus_slave_qp_t;

/**
 * @brief Construct the backend and attach it to the core's event port.
 *
 * Must run before modbus_slave_start() (typically right after the board has
 * constructed the core). @p slave and @p app must not be NULL; both are
 * stored, not copied.
 *
 * If an argument is NULL, or the core has already been started so the event
 * port cannot be attached, the backend records a fault and stays in
 * STOPPED: init() will not start the core and MB_START_SIG is ignored.
 */
void modbus_slave_qp_ctor(modbus_slave_qp_t* me,
                          modbus_slave_t* slave,
                          QActive* app);

/**
 * @brief Run the initial transition (QASM_INIT).
 *
 * Call it once, after QACTIVE_START(): that is when QP has given the AO its
 * priority and its event queue, so an ISR-posted protocol event has
 * somewhere to go. Calling it before QACTIVE_START() would start reception
 * too early, when posts to the AO have no valid target. It may also be
 * called from the AO's own initial transition.
 *
 * Enters the idle state, whose entry applies the pending configuration and
 * starts the core, so the slave is in service right after this call. If
 * construction faulted, the backend enters STOPPED and never starts the
 * core.
 */
void modbus_slave_qp_init(modbus_slave_qp_t* me);

/**
 * @brief Number of TX watchdog expiries since construction, saturating at 255.
 *
 * Diagnostic only: a non-zero count means a response did not drain in time
 * (for example a baudrate mismatch or a stuck DE line). Returns 0 for a NULL
 * instance.
 */
uint8_t modbus_slave_qp_tx_timeout_count(
    const modbus_slave_qp_t* me);

#endif
