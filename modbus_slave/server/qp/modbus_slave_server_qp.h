/* SPDX-License-Identifier: GPL-3.0-or-later */

#ifndef MODBUS_SLAVE_SERVER_QP_H
#define MODBUS_SLAVE_SERVER_QP_H

/**
 * @file modbus_slave_server_qp.h
 * @brief QP/C (QHsm) server backend for the modbus_slave core.
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
 *     modbus_slave_server_qp_t server;  // QHsm, holds the core pointer
 * } ModbusApp;
 *
 * // in the app state handling MB_* signals:
 * QASM_DISPATCH(&me->server.super, e, 0u);
 * @endcode
 *
 * The AO's event queue must hold at least 8 events so the three ISR-posted
 * protocol events can never overflow it (QF_NO_MARGIN).
 */

#include "modbus_slave.h"
#include "qpc.h"

/**
 * @brief Signals the application AO must dispatch into the server HSM.
 *
 * MB_FRAME_RCVED_SIG / MB_FRAME_SENT_SIG / MB_TX_TIMEOUT_SIG are posted by
 * the backend's event port from interrupt context; MB_START_SIG /
 * MB_STOP_SIG / MB_CONFIG_SIG are posted by the application. Applications
 * define their own signals starting at MB_APP_SIG_BASE.
 */
enum ModbusSlaveServerQpSignals {
    MB_FRAME_RCVED_SIG = Q_USER_SIG, /**< A frame completed; process it. */
    MB_FRAME_SENT_SIG,               /**< The response went out on the wire. */
    MB_TX_TIMEOUT_SIG,               /**< Transmission watchdog expired. */
    MB_START_SIG,                    /**< Enter service: apply pending config. */
    MB_STOP_SIG,                     /**< Leave service. */
    MB_CONFIG_SIG,                   /**< Apply the pending configuration. */
    MB_APP_SIG_BASE                  /**< First application-defined signal. */
};

/**
 * @brief Server backend instance, embedded in the application AO.
 *
 * Members are private; only the functions declared here may touch them.
 */
typedef struct {
    QHsm super;            /**< QP state machine, must stay at offset 0. */
    modbus_slave_t* slave; /**< Core instance owned by the board module. */
    QActive* app;          /**< This channel's AO, receives the events. */
    uint8_t last_error;    /**< Saturating count of TX timeouts; see tx_timeout_count(). */
    uint8_t fault;         /**< Non-zero: construction failed; never start the core. */
} modbus_slave_server_qp_t;

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
void modbus_slave_server_qp_ctor(modbus_slave_server_qp_t* me,
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
void modbus_slave_server_qp_init(modbus_slave_server_qp_t* me);

/**
 * @brief Number of TX watchdog expiries since construction, saturating at 255.
 *
 * Diagnostic only: a non-zero count means a response did not drain in time
 * (for example a baudrate mismatch or a stuck DE line). Returns 0 for a NULL
 * instance.
 */
uint8_t modbus_slave_server_qp_tx_timeout_count(
    const modbus_slave_server_qp_t* me);

#endif
