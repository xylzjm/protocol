/* SPDX-License-Identifier: MIT */

#include "modbus_slave_internal.h"

/* Core object: construction, lifecycle, the receive/execute/send primitives
 * the application drives, the ISR entry points, the event port and the
 * configuration helpers. */

/* T3.5 -------------------------------------------------------------------- */

/** Inter-frame delay in microseconds; 3.5 character times, fixed 1.75 ms above 19200 baud. */
static uint32_t t35_from_baudrate(uint32_t baudrate)
{
    if ( baudrate > 19200U ) {
        return 1750U;
    }
    return (38500000U + baudrate - 1U) / baudrate;
}

/* Construction ------------------------------------------------------------ */

static uint8_t config_valid(const modbus_slave_config_t* config)
{
    uintptr_t rx_low;
    uintptr_t rx_high;
    uintptr_t tx_low;
    uintptr_t tx_high;

    if ( config->baudrate == 0U ) {
        return 0U;
    }
    if ( config->rx_buffer == NULL || config->tx_buffer == NULL
         || config->rx_capacity < (uint16_t)MODBUS_SLAVE_ADU_SIZE_MAX
         || config->tx_capacity < (uint16_t)MODBUS_SLAVE_ADU_SIZE_MAX ) {
        return 0U;
    }

    /* The two buffers must not overlap: RX DMA/interrupts and TX readout can
     * run concurrently. uintptr_t comparisons keep this well-defined. */
    rx_low  = (uintptr_t)config->rx_buffer;
    rx_high = rx_low + config->rx_capacity;
    tx_low  = (uintptr_t)config->tx_buffer;
    tx_high = tx_low + config->tx_capacity;
    if ( rx_low < tx_high && tx_low < rx_high ) {
        return 0U;
    }

    if ( config->line.direction == NULL
         || config->line.put_byte == NULL
         || config->line.tx_done == NULL
         || config->timer.arm == NULL
         || config->timer.cancel == NULL
         || config->critical.enter == NULL
         || config->critical.exit == NULL ) {
        return 0U;
    }

    if ( config->callbacks.coils == NULL
         || config->callbacks.discrete_inputs == NULL
         || config->callbacks.holding_registers == NULL
         || config->callbacks.input_registers == NULL
         || config->callbacks.address == NULL ) {
        return 0U;
    }

    if ( config->identity.additional_length
         > (uint16_t)MODBUS_SLAVE_ADDITIONAL_MAX ) {
        return 0U;
    }
    if ( config->identity.additional_length > 0U
         && config->identity.additional == NULL ) {
        return 0U;
    }

    return 1U;
}

modbus_slave_status_t modbus_slave_ctor(modbus_slave_t* me,
                                        const modbus_slave_config_t* config)
{
    if ( me == NULL || config == NULL || config_valid(config) == 0U ) {
        return MODBUS_SLAVE_ERR_ARG;
    }

    me->line            = config->line;
    me->timer           = config->timer;
    me->critical        = config->critical;
    me->callbacks       = config->callbacks;
    me->identity        = config->identity;
    me->rx_buffer       = config->rx_buffer;
    me->rx_capacity     = config->rx_capacity;
    me->tx_buffer       = config->tx_buffer;
    me->tx_capacity     = config->tx_capacity;
    me->baudrate = config->baudrate;
    me->t35_us   = t35_from_baudrate(config->baudrate);

    me->event.ctx               = NULL;
    me->event.on_frame_received = NULL;
    me->event.on_frame_sent     = NULL;
    me->event.on_tx_timeout     = NULL;

    me->session = (uint8_t)MODBUS_SLAVE_SESSION_STOPPED;

    me->rx_pos = 0U;
    me->tx_len = 0U;
    me->tx_pos = 0U;

    return MODBUS_SLAVE_OK;
}

/* Event port -------------------------------------------------------------- */

modbus_slave_status_t modbus_slave_set_event_port(
    modbus_slave_t* me,
    const modbus_slave_port_event_t* event_port)
{
    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }
    if ( me->session != (uint8_t)MODBUS_SLAVE_SESSION_STOPPED ) {
        return MODBUS_SLAVE_ERR_STATE;
    }

    if ( event_port == NULL ) {
        me->event.ctx               = NULL;
        me->event.on_frame_received = NULL;
        me->event.on_frame_sent     = NULL;
        me->event.on_tx_timeout     = NULL;
    } else {
        me->event = *event_port;
    }
    return MODBUS_SLAVE_OK;
}

/* Lifecycle --------------------------------------------------------------- */

modbus_slave_status_t modbus_slave_start(modbus_slave_t* me)
{
    uint8_t failed = 0U;

    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }
    if ( me->session == (uint8_t)MODBUS_SLAVE_SESSION_SENDING ) {
        /* A response is on the wire; start() must not switch reception on
         * under it. Tear the transmission down with stop() first. */
        return MODBUS_SLAVE_ERR_STATE;
    }

    me->critical.enter(me->critical.ctx);

    if ( me->session == (uint8_t)MODBUS_SLAVE_SESSION_STOPPED ) {
        /* Enter DISCARDING before touching the port: the timeout may land
         * immediately, and it must find the matching state. */
        me->session = (uint8_t)MODBUS_SLAVE_SESSION_DISCARDING;
        me->rx_pos  = 0U;

        /* Open reception and wait out one T3.5 of bus idle, so a frame that
         * was already in flight cannot be mistaken for a complete one. */
        if ( me->line.direction(me->line.ctx, 1U, 0U)
             != MODBUS_SLAVE_OK ) {
            failed = 1U;
        }
        if ( me->timer.arm(me->timer.ctx, me->t35_us)
             != MODBUS_SLAVE_OK ) {
            failed = 1U;
        }

        if ( failed != 0U ) {
            /* Roll the half-started instance back so a retry starts clean. */
            me->session = (uint8_t)MODBUS_SLAVE_SESSION_STOPPED;
            (void)me->timer.cancel(me->timer.ctx);
            (void)me->line.direction(me->line.ctx, 0U, 0U);
        }
    } else if ( me->session == (uint8_t)MODBUS_SLAVE_SESSION_DISCARDING
                || me->session == (uint8_t)MODBUS_SLAVE_SESSION_IDLE ) {
        /* Idempotent re-entry (e.g. the application's serving entry): make sure
         * reception is on without disturbing a frame in collection. */
        if ( me->line.direction(me->line.ctx, 1U, 0U)
             != MODBUS_SLAVE_OK ) {
            failed = 1U;
        }
    }

    me->critical.exit(me->critical.ctx);
    return (failed != 0U) ? MODBUS_SLAVE_ERR_IO : MODBUS_SLAVE_OK;
}

modbus_slave_status_t modbus_slave_stop(modbus_slave_t* me)
{
    uint8_t failed = 0U;

    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }

    me->critical.enter(me->critical.ctx);

    me->session = (uint8_t)MODBUS_SLAVE_SESSION_STOPPED;
    me->rx_pos  = 0U;

    if ( me->timer.cancel(me->timer.ctx) != MODBUS_SLAVE_OK ) {
        failed = 1U;
    }
    if ( me->line.direction(me->line.ctx, 0U, 0U) != MODBUS_SLAVE_OK ) {
        failed = 1U;
    }

    me->critical.exit(me->critical.ctx);
    return (failed != 0U) ? MODBUS_SLAVE_ERR_IO : MODBUS_SLAVE_OK;
}

/* ISR entry points -------------------------------------------------------- */

modbus_slave_status_t modbus_slave_rx_byte(modbus_slave_t* me,
                                           uint8_t byte)
{
    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }
    if ( me->session == (uint8_t)MODBUS_SLAVE_SESSION_STOPPED ) {
        return MODBUS_SLAVE_ERR_STATE;
    }

    modbus_slave_rtu_rx_byte(me, byte);
    return MODBUS_SLAVE_OK;
}

modbus_slave_status_t modbus_slave_timer_expired(modbus_slave_t* me)
{
    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }
    if ( me->session == (uint8_t)MODBUS_SLAVE_SESSION_STOPPED ) {
        return MODBUS_SLAVE_ERR_STATE;
    }

    modbus_slave_rtu_timer_expired(me);
    return MODBUS_SLAVE_OK;
}

modbus_slave_status_t modbus_slave_tx_ready(modbus_slave_t* me)
{
    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }
    if ( me->session == (uint8_t)MODBUS_SLAVE_SESSION_STOPPED ) {
        return MODBUS_SLAVE_ERR_STATE;
    }

    modbus_slave_rtu_tx_ready(me);
    return MODBUS_SLAVE_OK;
}

modbus_slave_status_t modbus_slave_tx_complete(modbus_slave_t* me)
{
    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }
    if ( me->session == (uint8_t)MODBUS_SLAVE_SESSION_STOPPED ) {
        return MODBUS_SLAVE_ERR_STATE;
    }

    modbus_slave_rtu_tx_complete(me);
    return MODBUS_SLAVE_OK;
}

/* The application's one operation per frame -------------------------------- */

static void reopen_rx(modbus_slave_t* me)
{
    me->session = (uint8_t)MODBUS_SLAVE_SESSION_IDLE;
    (void)me->line.direction(me->line.ctx, 1U, 0U);
}

modbus_slave_step_t modbus_slave_advance(modbus_slave_t* me)
{
    modbus_slave_request_t req  = {0U};
    modbus_slave_response_t rep = {0U};
    modbus_slave_exception_t exception;
    enum modbus_slave_capture capture;
    uint16_t pdu_length;

    if ( me == NULL ) {
        return MODBUS_SLAVE_STEP_NONE;
    }
    if ( me->session == (uint8_t)MODBUS_SLAVE_SESSION_SENDING ) {
        /* A response is on the wire: the TX path owns the reception window
         * and releases it at TC or on the watchdog. A frame event arriving
         * now is stale; leave everything as it is. */
        return MODBUS_SLAVE_STEP_SENDING;
    }
    if ( me->session != (uint8_t)MODBUS_SLAVE_SESSION_CAPTURED ) {
        /* No frozen window is held by anyone (stray event, or stopped). */
        return MODBUS_SLAVE_STEP_NONE;
    }

    /* The frame is frozen and reception is off; this transaction owns the
     * window from here on, and every exit below either re-opens it or hands
     * it to the TX path. */
    capture = modbus_slave_adu_capture(me, &req);
    if ( capture == MODBUS_SLAVE_CAPTURE_IGNORE ) {
        reopen_rx(me);
        return MODBUS_SLAVE_STEP_HANDLED;
    }

    exception = modbus_slave_adu_decode(me, &req);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        modbus_slave_adu_exception(&rep, req.function, exception);
    } else {
        modbus_slave_fc_execute(me, &req, &rep);
    }

    if ( capture == MODBUS_SLAVE_CAPTURE_BROADCAST ) {
        /* A broadcast is never answered, so the transaction ends here and
         * this is the only place that releases its window. */
        reopen_rx(me);
        return MODBUS_SLAVE_STEP_HANDLED;
    }

    if ( rep.function == 0U ) {
        /* Nothing to send (defensive: a unicast always builds a reply, if
         * only an exception). The transaction ends here as well. */
        reopen_rx(me);
        return MODBUS_SLAVE_STEP_HANDLED;
    }

    /* Wrap the reply in an ADU, arm the watchdog and start transmitting.
     * Reception stays frozen -- the request buffer must not change under the
     * transmitter -- and the TX path releases it at TC or on the watchdog. */
    pdu_length = modbus_slave_adu_finish(me, &rep);
    modbus_slave_adu_transmit(me, &req, pdu_length);
    return MODBUS_SLAVE_STEP_SENDING;
}

/* Address ----------------------------------------------------------------- */

uint8_t modbus_slave_get_address(const modbus_slave_t* me)
{
    if ( me == NULL ) {
        return 0U;
    }
    return me->callbacks.address(me->callbacks.ctx);
}

void modbus_slave_set_running(modbus_slave_t* me,
                              uint8_t is_running)
{
    if ( me == NULL ) {
        return;
    }
    me->identity.is_running = (uint8_t)((is_running != 0U) ? 1U : 0U);
}

/* Wire-order helpers ------------------------------------------------------ */

uint16_t modbus_slave_get_u16(const uint8_t* buffer)
{
    return (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
}

void modbus_slave_put_u16(uint8_t* buffer,
                          uint16_t value)
{
    buffer[0] = (uint8_t)(value >> 8);
    buffer[1] = (uint8_t)value;
}

/* Event notifications (called from the RTU layer, ISR context) ------------ */

void modbus_slave_notify_frame_received(modbus_slave_t* me)
{
    if ( me->event.on_frame_received != NULL ) {
        me->event.on_frame_received(me->event.ctx);
    }
}

void modbus_slave_notify_frame_sent(modbus_slave_t* me)
{
    if ( me->event.on_frame_sent != NULL ) {
        me->event.on_frame_sent(me->event.ctx);
    }
}

void modbus_slave_notify_tx_timeout(modbus_slave_t* me)
{
    if ( me->event.on_tx_timeout != NULL ) {
        me->event.on_tx_timeout(me->event.ctx);
    }
}
