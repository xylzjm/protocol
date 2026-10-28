/* SPDX-License-Identifier: GPL-3.0-or-later */

#include "modbus_slave_server_qp.h"

/* QP/C server backend: a three-state QHsm (STOPPED/IDLE/SENDING) that the
 * application AO drives with QASM_DISPATCH(). The protocol events arrive
 * through the core's event port, which posts them directly to this channel's
 * AO — directed posting, never publish, so two channels cannot cross-talk.
 *
 * State chart (see README):
 *
 *   STOPPED --START--> IDLE --FRAME_RCVED(unicast)--> SENDING
 *      ^                 |  ^                            |
 *      |    STOP         |  | FRAME_SENT / TX_TIMEOUT    | STOP
 *      +-----------------+  +----------------------------+
 */

static QState server_initial(modbus_slave_server_qp_t* const me,
                             void const* const par);
static QState server_stopped(modbus_slave_server_qp_t* const me,
                             QEvt const* const e);
static QState server_idle(modbus_slave_server_qp_t* const me,
                          QEvt const* const e);
static QState server_sending(modbus_slave_server_qp_t* const me,
                             QEvt const* const e);

/* Event port: core ISR -> application AO --------------------------------- */

static void server_frame_received(void* ctx)
{
    modbus_slave_server_qp_t* me = (modbus_slave_server_qp_t*)ctx;
    static QEvt const evt        = QEVT_INITIALIZER(MB_FRAME_RCVED_SIG);

    QACTIVE_POST(me->app, &evt, 0);
}

static void server_frame_sent(void* ctx)
{
    modbus_slave_server_qp_t* me = (modbus_slave_server_qp_t*)ctx;
    static QEvt const evt        = QEVT_INITIALIZER(MB_FRAME_SENT_SIG);

    QACTIVE_POST(me->app, &evt, 0);
}

static void server_tx_timeout(void* ctx)
{
    modbus_slave_server_qp_t* me = (modbus_slave_server_qp_t*)ctx;
    static QEvt const evt        = QEVT_INITIALIZER(MB_TX_TIMEOUT_SIG);

    QACTIVE_POST(me->app, &evt, 0);
}

/* Construction ------------------------------------------------------------ */

void modbus_slave_server_qp_ctor(modbus_slave_server_qp_t* me,
                                 modbus_slave_t* slave,
                                 QActive* app)
{
    modbus_slave_event_port_t event_port;

    QHsm_ctor(&me->super, Q_STATE_CAST(&server_initial));
    me->slave      = slave;
    me->app        = app;
    me->last_error = 0U;
    me->fault      = 0U;

    if ( slave == NULL || app == NULL ) {
        me->fault = 1U;
        return;
    }

    event_port.ctx               = me;
    event_port.on_frame_received = server_frame_received;
    event_port.on_frame_sent     = server_frame_sent;
    event_port.on_tx_timeout     = server_tx_timeout;

    if ( modbus_slave_set_event_port(slave, &event_port)
         != MODBUS_SLAVE_OK ) {
        /* The core was already started: it could never report a frame to
         * this backend. Refuse to run blind instead of silently missing
         * every event. */
        me->fault = 1U;
    }
}

void modbus_slave_server_qp_init(modbus_slave_server_qp_t* me)
{
    QASM_INIT(&me->super, NULL, 0u);
}

uint8_t modbus_slave_server_qp_tx_timeout_count(
    const modbus_slave_server_qp_t* me)
{
    return (me != NULL) ? me->last_error : 0U;
}

/* States ------------------------------------------------------------------ */

static QState server_initial(modbus_slave_server_qp_t* const me,
                             void const* const par)
{
    Q_UNUSED_PAR(par);

    if ( me->fault != 0U ) {
        /* Construction could not attach the event port (or an argument was
         * NULL): never start a core this backend cannot hear. */
        return Q_TRAN(&server_stopped);
    }

    /* Idle's entry applies the pending configuration and starts the core,
     * so the slave is in service as soon as the initial transition lands. */
    return Q_TRAN(&server_idle);
}

static QState server_stopped(modbus_slave_server_qp_t* const me,
                             QEvt const* const e)
{
    switch ( e->sig ) {
        case Q_ENTRY_SIG:
            modbus_slave_stop(me->slave);
            return Q_HANDLED();

        case MB_START_SIG:
            if ( me->fault != 0U ) {
                return Q_HANDLED(); /* half-wired: stay down */
            }
            return Q_TRAN(&server_idle);

        case MB_CONFIG_SIG:
            /* Keep the pending address in the core; the idle entry applies
             * it on the next start. */
            return Q_HANDLED();

        default:
            return Q_SUPER(&QHsm_top);
    }
}

static QState server_idle(modbus_slave_server_qp_t* const me,
                          QEvt const* const e)
{
    switch ( e->sig ) {
        case Q_ENTRY_SIG:
            modbus_slave_apply_config(me->slave);
            (void)modbus_slave_start(me->slave); /* idempotent, ensures RX on */
            return Q_HANDLED();

        case MB_FRAME_RCVED_SIG: {
            modbus_slave_rx_status_t status;

            status = modbus_slave_receive(me->slave);
            switch ( status ) {
                case MODBUS_SLAVE_RX_UNICAST:
                    modbus_slave_execute(me->slave);
                    if ( modbus_slave_send(me->slave)
                         == MODBUS_SLAVE_OK ) {
                        return Q_TRAN(&server_sending);
                    }
                    /* send() refused: reopen RX so the bus is not left
                     * deaf, then keep waiting for the next request. */
                    modbus_slave_enable_rx(me->slave);
                    return Q_HANDLED();

                case MODBUS_SLAVE_RX_BROADCAST:
                    modbus_slave_execute(me->slave);
                    modbus_slave_enable_rx(me->slave);
                    return Q_HANDLED();

                default: /* MODBUS_SLAVE_RX_IGNORE */
                    modbus_slave_enable_rx(me->slave);
                    return Q_HANDLED();
            }
        }

        case MB_CONFIG_SIG:
            modbus_slave_disable_rx(me->slave);
            modbus_slave_apply_config(me->slave);
            modbus_slave_enable_rx(me->slave);
            return Q_HANDLED();

        case MB_STOP_SIG:
            return Q_TRAN(&server_stopped);

        /* Stale completion events from a transmission that is already
         * over (e.g. a TX timeout racing the TC interrupt): ignore. */
        case MB_FRAME_SENT_SIG:
        case MB_TX_TIMEOUT_SIG:
        default:
            return Q_SUPER(&QHsm_top);
    }
}

static QState server_sending(modbus_slave_server_qp_t* const me,
                             QEvt const* const e)
{
    switch ( e->sig ) {
        case MB_FRAME_SENT_SIG:
            return Q_TRAN(&server_idle);

        case MB_TX_TIMEOUT_SIG:
            /* The watchdog already stopped the byte stream; abandon the
             * transmission, release the bus and count the incident. */
            modbus_slave_abort_tx(me->slave);
            if ( me->last_error < 0xFFU ) {
                me->last_error++;
            }
            return Q_TRAN(&server_idle);

        case MB_STOP_SIG:
            return Q_TRAN(&server_stopped);

        case MB_CONFIG_SIG:
            /* Leave the pending address stored; the next idle entry
             * applies it. */
            return Q_HANDLED();

        case MB_FRAME_RCVED_SIG:
        default:
            return Q_SUPER(&QHsm_top);
    }
}
