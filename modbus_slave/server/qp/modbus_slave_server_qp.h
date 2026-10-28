#ifndef MODBUS_SLAVE_SERVER_QP_H
#define MODBUS_SLAVE_SERVER_QP_H

/* SPDX-License-Identifier: GPL-3.0-or-later */

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
    uint8_t last_error;    /**< Saturating count of TX timeouts. */
} modbus_slave_server_qp_t;

/**
 * @brief Construct the backend and attach it to the core's event port.
 *
 * Must run before modbus_slave_start(): attaching a port after start()
 * fails with MODBUS_SLAVE_ERR_STATE. @p slave and @p app must not be NULL;
 * both are stored, not copied.
 */
void modbus_slave_server_qp_ctor(modbus_slave_server_qp_t* me,
                                 modbus_slave_t* slave,
                                 QActive* app);

/**
 * @brief Run the initial transition (QASM_INIT).
 *
 * Enters the idle state, whose entry applies the pending configuration and
 * starts the core, so the slave is in service right after this call.
 */
void modbus_slave_server_qp_init(modbus_slave_server_qp_t* me);

#endif
