/* SPDX-License-Identifier: MIT */

#include "modbus_slave_internal.h"

/* RTU framing: the receive and transmit machines, the T3.5 inter-frame timer
 * and the TX watchdog. This module never touches the PDU: it only decides when
 * a frame is complete and drives the serial port through the board's port.
 * The session state says what reception is doing and what the timer means. */

/* Resynchronization ------------------------------------------------------- */

void modbus_slave_rtu_resync(modbus_slave_t* me)
{
    me->session = (uint8_t)MODBUS_SLAVE_SESSION_DISCARDING;
    me->rx_pos  = 0U;

    /* Enter the state before touching the port: an expiry that lands
     * immediately must already find it. Reception stays open while the bytes
     * are dropped; the gap after the last one is what makes the edge of the
     * next frame trustworthy. */
    (void)me->port.serial_enable(me->port.ctx, 1U, 0U);
    (void)me->port.timer_enable(me->port.ctx, me->t35_us);
}

/* RX frame machine -------------------------------------------------------- */

void modbus_slave_rtu_rx_byte(modbus_slave_t* me,
                              uint8_t byte)
{
    switch ( (enum modbus_slave_session)me->session ) {
        case MODBUS_SLAVE_SESSION_DISCARDING:
            /* The bus has not been idle for T3.5 yet, so the frame edge is
             * unknown: discard the byte and keep waiting for a full gap. */
            (void)me->port.timer_enable(me->port.ctx, me->t35_us);
            break;

        case MODBUS_SLAVE_SESSION_IDLE:
            me->rx_buffer[0] = byte;
            me->rx_pos       = 1U;
            me->session      = (uint8_t)MODBUS_SLAVE_SESSION_RECEIVING;
            (void)me->port.timer_enable(me->port.ctx, me->t35_us);
            break;

        case MODBUS_SLAVE_SESSION_RECEIVING:
            if ( me->rx_pos >= (uint16_t)MODBUS_SLAVE_ADU_SIZE_MAX ) {
                /* Overlong: a legal RTU ADU never exceeds 256 bytes, so this
                 * is noise. Drop everything until the bus goes idle again. */
                modbus_slave_rtu_resync(me);
            } else {
                me->rx_buffer[me->rx_pos] = byte;
                me->rx_pos++;
                (void)me->port.timer_enable(me->port.ctx, me->t35_us);
            }
            break;

        case MODBUS_SLAVE_SESSION_STOPPED:
        case MODBUS_SLAVE_SESSION_WAITING:
        case MODBUS_SLAVE_SESSION_SENDING:
        default:
            /* Reception is off or frozen: a stray byte must neither start a
             * frame nor disturb a captured one. */
            break;
    }
}

/* Timer expiry ------------------------------------------------------------ */

void modbus_slave_rtu_timer_expired(modbus_slave_t* me)
{
    switch ( (enum modbus_slave_session)me->session ) {
        case MODBUS_SLAVE_SESSION_DISCARDING:
            /* One T3.5 of silence: the frame edges are trustworthy now. */
            me->session = (uint8_t)MODBUS_SLAVE_SESSION_IDLE;
            break;

        case MODBUS_SLAVE_SESSION_RECEIVING:
            /* End of frame: freeze reception so the next request cannot
             * overwrite this one while the backend processes it. */
            me->session = (uint8_t)MODBUS_SLAVE_SESSION_WAITING;
            (void)me->port.serial_enable(me->port.ctx, 0U, 0U);
            modbus_slave_notify_frame_received(me);
            break;

        case MODBUS_SLAVE_SESSION_SENDING:
            /* The response will never drain. Release the line here, in the
             * interrupt: a shared RS485 segment stays blocked while DE is
             * asserted, so recovery must not depend on when the backend's
             * active object gets to run. */
            modbus_slave_rtu_resync(me);
            modbus_slave_notify_tx_timeout(me);
            break;

        case MODBUS_SLAVE_SESSION_STOPPED:
        case MODBUS_SLAVE_SESSION_IDLE:
        case MODBUS_SLAVE_SESSION_WAITING:
        default:
            /* No timer is armed in these states; a spurious expiry is a no-op. */
            break;
    }
}

/* TX frame machine -------------------------------------------------------- */

void modbus_slave_rtu_tx_ready(modbus_slave_t* me)
{
    if ( me->session == (uint8_t)MODBUS_SLAVE_SESSION_SENDING
         && me->tx_pos < me->tx_len ) {
        (void)me->port.put_byte(me->port.ctx, me->tx_buffer[me->tx_pos]);
        me->tx_pos++;

        if ( me->tx_pos == me->tx_len ) {
            /* The last byte is in the data register, so no more TXE is
             * needed. Mute it now: a level-triggered TXE would otherwise
             * keep interrupting until TC. TC stays enabled and closes the
             * frame (releasing DE only when the last bit is out). */
            (void)me->port.tx_done(me->port.ctx);
        }
    } else {
        /* Nothing left to send: a stray TXE, or the TX watchdog already
         * moved the machine out of SENDING. Mute TXE here as well. */
        (void)me->port.tx_done(me->port.ctx);
    }
}

void modbus_slave_rtu_tx_complete(modbus_slave_t* me)
{
    if ( me->session != (uint8_t)MODBUS_SLAVE_SESSION_SENDING ) {
        return;
    }

    /* Disarm the watchdog first: the frame is over, whatever it was doing. */
    (void)me->port.timer_disable(me->port.ctx);
    me->session = (uint8_t)MODBUS_SLAVE_SESSION_IDLE;

    /* TC is the last bit leaving the shift register, so DE may be released
     * and reception reopened right here. */
    (void)me->port.serial_enable(me->port.ctx, 1U, 0U);
    modbus_slave_notify_frame_sent(me);
}

void modbus_slave_rtu_start_tx(modbus_slave_t* me)
{
    uint32_t tx_timeout_us;

    /* 11 bit times per byte (start + 8 data + parity + stop) plus 10 ms of
     * margin. 64-bit math first: at low baud rates the 32-bit product would
     * overflow. */
    tx_timeout_us = (uint32_t)(((uint64_t)me->tx_len * 11U * 1000000U)
                               / me->baudrate)
                    + 10000U;

    /* Enter SENDING before arming the timer, so an expiry that lands
     * immediately already finds the matching state. */
    me->session = (uint8_t)MODBUS_SLAVE_SESSION_SENDING;
    (void)me->port.timer_enable(me->port.ctx, tx_timeout_us);

    /* RX off, DE on, TC cleared, TXE+TC interrupts enabled: the TXE
     * interrupt that follows delivers the first byte through tx_ready(). */
    (void)me->port.serial_enable(me->port.ctx, 0U, 1U);
}
