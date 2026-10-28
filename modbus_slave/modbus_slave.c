/* SPDX-License-Identifier: MIT */

#include "modbus_slave_internal.h"

/* Core object: construction, lifecycle, the receive/execute/send primitives
 * the server backend drives, the ISR entry points, the event port and the
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

    if ( config->address < MODBUS_SLAVE_ADDRESS_MIN
         || config->address > MODBUS_SLAVE_ADDRESS_MAX ) {
        return 0U;
    }
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

    if ( config->port.serial_enable == NULL
         || config->port.put_byte == NULL
         || config->port.timer_enable == NULL
         || config->port.timer_disable == NULL
         || config->port.critical_enter == NULL
         || config->port.critical_exit == NULL ) {
        return 0U;
    }

    if ( config->callbacks.coils == NULL
         || config->callbacks.discrete_inputs == NULL
         || config->callbacks.holding_registers == NULL
         || config->callbacks.input_registers == NULL ) {
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

    me->port            = config->port;
    me->callbacks       = config->callbacks;
    me->identity        = config->identity;
    me->rx_buffer       = config->rx_buffer;
    me->rx_capacity     = config->rx_capacity;
    me->tx_buffer       = config->tx_buffer;
    me->tx_capacity     = config->tx_capacity;
    me->baudrate        = config->baudrate;
    me->t35_us          = t35_from_baudrate(config->baudrate);
    me->address         = config->address;
    me->pending_address = config->address;
    me->address_pending = 0U;

    me->event_port.ctx               = NULL;
    me->event_port.on_frame_received = NULL;
    me->event_port.on_frame_sent     = NULL;
    me->event_port.on_tx_timeout     = NULL;

    me->rx_state   = (uint8_t)MODBUS_SLAVE_RX_STATE_INIT;
    me->tx_state   = (uint8_t)MODBUS_SLAVE_TX_STATE_IDLE;
    me->timer_mode = (uint8_t)MODBUS_SLAVE_TIMER_NONE;

    me->rx_pos       = 0U;
    me->pdu_len      = 0U;
    me->response_len = 0U;
    me->tx_len       = 0U;
    me->tx_pos       = 0U;

    me->request_address = config->address;
    me->function        = 0U;
    me->broadcast       = 0U;
    me->started         = 0U;

    return MODBUS_SLAVE_OK;
}

/* Event port -------------------------------------------------------------- */

modbus_slave_status_t modbus_slave_set_event_port(
    modbus_slave_t* me,
    const modbus_slave_event_port_t* event_port)
{
    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }
    if ( me->started != 0U ) {
        return MODBUS_SLAVE_ERR_STATE;
    }

    if ( event_port == NULL ) {
        me->event_port.ctx               = NULL;
        me->event_port.on_frame_received = NULL;
        me->event_port.on_frame_sent     = NULL;
        me->event_port.on_tx_timeout     = NULL;
    } else {
        me->event_port = *event_port;
    }
    return MODBUS_SLAVE_OK;
}

/* Lifecycle --------------------------------------------------------------- */

modbus_slave_status_t modbus_slave_start(modbus_slave_t* me)
{
    uint32_t token;
    uint8_t failed = 0U;

    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }

    token = me->port.critical_enter(me->port.ctx);

    if ( me->started == 0U ) {
        me->started  = 1U;
        me->rx_state = (uint8_t)MODBUS_SLAVE_RX_STATE_INIT;
        me->tx_state = (uint8_t)MODBUS_SLAVE_TX_STATE_IDLE;
        me->rx_pos   = 0U;

        /* Set the mode before touching the timer: an expiry that lands
         * immediately must already find the matching mode. */
        me->timer_mode = (uint8_t)MODBUS_SLAVE_TIMER_RX_T35;

        /* Open reception and wait out one T3.5 of bus idle, so a frame that
         * was already in flight cannot be mistaken for a complete one. */
        if ( me->port.serial_enable(me->port.ctx, 1U, 0U)
             != MODBUS_SLAVE_OK ) {
            failed = 1U;
        }
        if ( me->port.timer_enable(me->port.ctx, me->t35_us)
             != MODBUS_SLAVE_OK ) {
            failed = 1U;
        }

        if ( failed != 0U ) {
            /* Roll the half-started instance back so a retry starts clean. */
            me->started    = 0U;
            me->timer_mode = (uint8_t)MODBUS_SLAVE_TIMER_NONE;
            (void)me->port.timer_disable(me->port.ctx);
            (void)me->port.serial_enable(me->port.ctx, 0U, 0U);
        }
    } else if ( me->rx_state == (uint8_t)MODBUS_SLAVE_RX_STATE_INIT
                || me->rx_state == (uint8_t)MODBUS_SLAVE_RX_STATE_IDLE ) {
        /* Idempotent re-entry (e.g. the backend's IDLE entry): make sure
         * reception is on without disturbing a frame in collection. */
        if ( me->port.serial_enable(me->port.ctx, 1U, 0U)
             != MODBUS_SLAVE_OK ) {
            failed = 1U;
        }
    }

    me->port.critical_exit(me->port.ctx, token);
    return (failed != 0U) ? MODBUS_SLAVE_ERR_IO : MODBUS_SLAVE_OK;
}

modbus_slave_status_t modbus_slave_stop(modbus_slave_t* me)
{
    uint32_t token;
    uint8_t failed = 0U;

    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }

    token = me->port.critical_enter(me->port.ctx);

    me->started    = 0U;
    me->rx_state   = (uint8_t)MODBUS_SLAVE_RX_STATE_INIT;
    me->tx_state   = (uint8_t)MODBUS_SLAVE_TX_STATE_IDLE;
    me->timer_mode = (uint8_t)MODBUS_SLAVE_TIMER_NONE;
    me->rx_pos     = 0U;

    if ( me->port.timer_disable(me->port.ctx) != MODBUS_SLAVE_OK ) {
        failed = 1U;
    }
    if ( me->port.serial_enable(me->port.ctx, 0U, 0U) != MODBUS_SLAVE_OK ) {
        failed = 1U;
    }

    me->port.critical_exit(me->port.ctx, token);
    return (failed != 0U) ? MODBUS_SLAVE_ERR_IO : MODBUS_SLAVE_OK;
}

/* ISR entry points -------------------------------------------------------- */

modbus_slave_status_t modbus_slave_rx_byte(modbus_slave_t* me,
                                           uint8_t byte)
{
    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }
    if ( me->started == 0U ) {
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
    if ( me->started == 0U ) {
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
    if ( me->started == 0U ) {
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
    if ( me->started == 0U ) {
        return MODBUS_SLAVE_ERR_STATE;
    }

    modbus_slave_rtu_tx_complete(me);
    return MODBUS_SLAVE_OK;
}

/* Backend primitives ------------------------------------------------------ */

modbus_slave_rx_status_t modbus_slave_receive(modbus_slave_t* me)
{
    uint8_t address;
    uint16_t frame_len;
    uint16_t crc;

    if ( me == NULL
         || me->rx_state != (uint8_t)MODBUS_SLAVE_RX_STATE_WAIT_PROCESS ) {
        return MODBUS_SLAVE_RX_IGNORE;
    }

    /* WAIT_PROCESS -> IDLE: whatever the verdict, the frame machine is free
     * again; the backend re-opens reception for IGNORE/BROADCAST. */
    me->rx_state = (uint8_t)MODBUS_SLAVE_RX_STATE_IDLE;

    frame_len = me->rx_pos;
    if ( frame_len < (uint16_t)MODBUS_SLAVE_ADU_SIZE_MIN ) {
        return MODBUS_SLAVE_RX_IGNORE;
    }

    crc = modbus_slave_crc(me->rx_buffer, (uint16_t)(frame_len - 2U));
    if ( me->rx_buffer[frame_len - 2U] != (uint8_t)(crc & 0xFFU)
         || me->rx_buffer[frame_len - 1U] != (uint8_t)(crc >> 8) ) {
        return MODBUS_SLAVE_RX_IGNORE;
    }

    address = me->rx_buffer[0];
    if ( address != me->address
         && address != (uint8_t)MODBUS_SLAVE_ADDRESS_BROADCAST ) {
        return MODBUS_SLAVE_RX_IGNORE;
    }

    me->request_address = address;
    me->function        = me->rx_buffer[1];
    me->pdu_len         = (uint16_t)(frame_len - 3U); /* address + CRC */

    if ( address == (uint8_t)MODBUS_SLAVE_ADDRESS_BROADCAST ) {
        me->broadcast = 1U;
        switch ( me->function ) {
            case MODBUS_SLAVE_FC_WRITE_SINGLE_COIL:
            case MODBUS_SLAVE_FC_WRITE_SINGLE_REGISTER:
            case MODBUS_SLAVE_FC_WRITE_MULTIPLE_COILS:
            case MODBUS_SLAVE_FC_WRITE_MULTIPLE_REGS:
                return MODBUS_SLAVE_RX_BROADCAST;

            default:
                /* Broadcast reads (and 0x11) are ignored, no callback runs. */
                return MODBUS_SLAVE_RX_IGNORE;
        }
    }

    me->broadcast = 0U;
    return MODBUS_SLAVE_RX_UNICAST;
}

void modbus_slave_execute(modbus_slave_t* me)
{
    if ( me == NULL ) {
        return;
    }
    modbus_slave_pdu_execute(me);
}

modbus_slave_status_t modbus_slave_send(modbus_slave_t* me)
{
    if ( me == NULL ) {
        return MODBUS_SLAVE_ERR_ARG;
    }
    if ( me->started == 0U
         || me->tx_state != (uint8_t)MODBUS_SLAVE_TX_STATE_IDLE
         || me->response_len == 0U ) {
        return MODBUS_SLAVE_ERR_STATE;
    }
    /* A broadcast is never answered: execute() may have built a PDU for the
     * callbacks, but it must not reach the wire. */
    if ( me->broadcast != 0U ) {
        return MODBUS_SLAVE_ERR_STATE;
    }

    modbus_slave_rtu_send(me);
    return MODBUS_SLAVE_OK;
}

void modbus_slave_abort_tx(modbus_slave_t* me)
{
    if ( me == NULL ) {
        return;
    }
    modbus_slave_rtu_abort_tx(me);
}

void modbus_slave_enable_rx(modbus_slave_t* me)
{
    if ( me == NULL || me->started == 0U ) {
        return;
    }
    (void)me->port.serial_enable(me->port.ctx, 1U, 0U);
}

void modbus_slave_disable_rx(modbus_slave_t* me)
{
    if ( me == NULL || me->started == 0U ) {
        return;
    }
    (void)me->port.serial_enable(me->port.ctx, 0U, 0U);
}

/* Configuration ----------------------------------------------------------- */

void modbus_slave_set_address_pending(modbus_slave_t* me,
                                      uint8_t address)
{
    if ( me == NULL || address < MODBUS_SLAVE_ADDRESS_MIN
         || address > MODBUS_SLAVE_ADDRESS_MAX ) {
        return;
    }
    me->pending_address = address;
    me->address_pending = 1U;
}

void modbus_slave_apply_config(modbus_slave_t* me)
{
    if ( me == NULL || me->address_pending == 0U ) {
        return;
    }
    me->address         = me->pending_address;
    me->address_pending = 0U;
}

uint8_t modbus_slave_get_address(const modbus_slave_t* me)
{
    if ( me == NULL ) {
        return 0U;
    }
    return me->address;
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
    if ( me->event_port.on_frame_received != NULL ) {
        me->event_port.on_frame_received(me->event_port.ctx);
    }
}

void modbus_slave_notify_frame_sent(modbus_slave_t* me)
{
    if ( me->event_port.on_frame_sent != NULL ) {
        me->event_port.on_frame_sent(me->event_port.ctx);
    }
}

void modbus_slave_notify_tx_timeout(modbus_slave_t* me)
{
    if ( me->event_port.on_tx_timeout != NULL ) {
        me->event_port.on_tx_timeout(me->event_port.ctx);
    }
}
