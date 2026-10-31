/* SPDX-License-Identifier: MIT */

#include "modbus_slave_internal.h"

void modbus_slave_rtu_resync(modbus_slave_t* me)
{
    me->session = (uint8_t)MODBUS_SLAVE_SESSION_DISCARDING;
    me->rx_pos  = 0U;

    (void)me->line.direction(me->line.ctx, 1U, 0U);
    (void)me->timer.arm(me->timer.ctx, me->t35_us);
}

void modbus_slave_rtu_rx_byte(modbus_slave_t* me,
                              uint8_t byte)
{
    switch ( (enum modbus_slave_session)me->session ) {
        case MODBUS_SLAVE_SESSION_DISCARDING:
            (void)me->timer.arm(me->timer.ctx, me->t35_us);
            break;

        case MODBUS_SLAVE_SESSION_IDLE:
            me->rx_buffer[0] = byte;
            me->rx_pos       = 1U;
            me->session      = (uint8_t)MODBUS_SLAVE_SESSION_RECEIVING;
            (void)me->timer.arm(me->timer.ctx, me->t35_us);
            break;

        case MODBUS_SLAVE_SESSION_RECEIVING:
            if ( me->rx_pos >= (uint16_t)MODBUS_SLAVE_ADU_SIZE_MAX ) {
                modbus_slave_rtu_resync(me);
            } else {
                me->rx_buffer[me->rx_pos] = byte;
                me->rx_pos++;
                (void)me->timer.arm(me->timer.ctx, me->t35_us);
            }
            break;

        case MODBUS_SLAVE_SESSION_STOPPED:
        case MODBUS_SLAVE_SESSION_CAPTURED:
        case MODBUS_SLAVE_SESSION_SENDING:
        default:
            break;
    }
}

void modbus_slave_rtu_timer_expired(modbus_slave_t* me)
{
    switch ( (enum modbus_slave_session)me->session ) {
        case MODBUS_SLAVE_SESSION_DISCARDING:
            me->session = (uint8_t)MODBUS_SLAVE_SESSION_IDLE;
            break;

        case MODBUS_SLAVE_SESSION_RECEIVING:
            me->session = (uint8_t)MODBUS_SLAVE_SESSION_CAPTURED;
            (void)me->line.direction(me->line.ctx, 0U, 0U);
            modbus_slave_notify_frame_received(me);
            break;

        case MODBUS_SLAVE_SESSION_SENDING:
            modbus_slave_rtu_resync(me);
            modbus_slave_notify_tx_timeout(me);
            break;

        case MODBUS_SLAVE_SESSION_STOPPED:
        case MODBUS_SLAVE_SESSION_IDLE:
        case MODBUS_SLAVE_SESSION_CAPTURED:
        default:
            break;
    }
}

void modbus_slave_rtu_tx_ready(modbus_slave_t* me)
{
    if ( me->session == (uint8_t)MODBUS_SLAVE_SESSION_SENDING
         && me->tx_pos < me->tx_len ) {
        (void)me->line.put_byte(me->line.ctx, me->tx_buffer[me->tx_pos]);
        me->tx_pos++;

        if ( me->tx_pos == me->tx_len ) {
            (void)me->line.tx_done(me->line.ctx);
        }
    } else {
        (void)me->line.tx_done(me->line.ctx);
    }
}

void modbus_slave_rtu_tx_complete(modbus_slave_t* me)
{
    if ( me->session != (uint8_t)MODBUS_SLAVE_SESSION_SENDING ) {
        return;
    }

    (void)me->timer.cancel(me->timer.ctx);
    me->session = (uint8_t)MODBUS_SLAVE_SESSION_IDLE;

    (void)me->line.direction(me->line.ctx, 1U, 0U);
    modbus_slave_notify_frame_sent(me);
}

void modbus_slave_rtu_start_tx(modbus_slave_t* me)
{
    uint32_t tx_timeout_us;

    tx_timeout_us = (uint32_t)(((uint64_t)me->tx_len * 11U * 1000000U)
                               / me->baudrate)
                    + 10000U;

    me->session = (uint8_t)MODBUS_SLAVE_SESSION_SENDING;
    (void)me->timer.arm(me->timer.ctx, tx_timeout_us);

    (void)me->line.direction(me->line.ctx, 0U, 1U);
}
