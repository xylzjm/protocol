/* SPDX-License-Identifier: MIT */

#include "modbus_slave_internal.h"

/* RTU framing: RX/TX frame machines, the T3.5 inter-frame timer and the TX
 * watchdog. This module never touches the PDU: it only decides when a frame
 * is complete and drives the serial port through the board's port. */

/* Release the line and resynchronize reception after an aborted response. */
static void abort_tx(modbus_slave_t* me);

/* RX frame machine -------------------------------------------------------- */

void modbus_slave_rtu_rx_byte(modbus_slave_t* me,
                              uint8_t byte)
{
    if ( me->tx_state != (uint8_t)MODBUS_SLAVE_TX_STATE_IDLE ) {
        /* A stray byte (late RXNE, DE switching glitch, or a port that did
         * not fully mute the receiver) must neither start a frame nor
         * replace the TX watchdog while a response is on the wire. */
        return;
    }

    switch ( (enum modbus_slave_rx_state)me->rx_state ) {
        case MODBUS_SLAVE_RX_STATE_INIT:
            /* The bus has not been idle for T3.5 yet, so the frame edge is
             * unknown: discard the byte and keep waiting for a full gap. */
            break;

        case MODBUS_SLAVE_RX_STATE_IDLE:
            me->rx_buffer[0] = byte;
            me->rx_pos       = 1U;
            me->rx_state     = (uint8_t)MODBUS_SLAVE_RX_STATE_RCV;
            break;

        case MODBUS_SLAVE_RX_STATE_RCV:
            if ( me->rx_pos >= (uint16_t)MODBUS_SLAVE_ADU_SIZE_MAX ) {
                /* Overlong: a legal RTU ADU never exceeds 256 bytes, so this
                 * is noise. Drop everything until the bus goes idle again. */
                me->rx_state = (uint8_t)MODBUS_SLAVE_RX_STATE_ERROR;
            } else {
                me->rx_buffer[me->rx_pos] = byte;
                me->rx_pos++;
            }
            break;

        case MODBUS_SLAVE_RX_STATE_WAIT_PROCESS:
        case MODBUS_SLAVE_RX_STATE_ERROR:
        default:
            /* WAIT_PROCESS: reception is off, a stray byte must not disturb
             * the captured frame. ERROR: keep discarding below. */
            break;
    }

    /* (Re)arm T3.5 while bytes keep arriving; the gap after the last byte
     * ends the frame (RCV), the startup synchronization (INIT), or the
     * overflow discard window (ERROR). */
    if ( me->rx_state == (uint8_t)MODBUS_SLAVE_RX_STATE_INIT
         || me->rx_state == (uint8_t)MODBUS_SLAVE_RX_STATE_RCV
         || me->rx_state == (uint8_t)MODBUS_SLAVE_RX_STATE_ERROR ) {
        me->timer_mode = (uint8_t)MODBUS_SLAVE_TIMER_RX_T35;
        (void)me->port.timer_enable(me->port.ctx, me->t35_us);
    }
}

/* Timer expiry ------------------------------------------------------------ */

void modbus_slave_rtu_timer_expired(modbus_slave_t* me)
{
    uint8_t mode = me->timer_mode;

    me->timer_mode = (uint8_t)MODBUS_SLAVE_TIMER_NONE;

    if ( mode == (uint8_t)MODBUS_SLAVE_TIMER_RX_T35 ) {
        switch ( (enum modbus_slave_rx_state)me->rx_state ) {
            case MODBUS_SLAVE_RX_STATE_INIT:
                /* One T3.5 of silence: the frame edges are trustworthy now. */
                me->rx_state = (uint8_t)MODBUS_SLAVE_RX_STATE_IDLE;
                break;

            case MODBUS_SLAVE_RX_STATE_RCV:
                /* End of frame: freeze reception so the next request cannot
                 * overwrite this one while the backend processes it. */
                me->rx_state = (uint8_t)MODBUS_SLAVE_RX_STATE_WAIT_PROCESS;
                (void)me->port.serial_enable(me->port.ctx, 0U, 0U);
                modbus_slave_notify_frame_received(me);
                break;

            case MODBUS_SLAVE_RX_STATE_ERROR:
                /* The overlong frame has ended; start listening again. */
                me->rx_state = (uint8_t)MODBUS_SLAVE_RX_STATE_IDLE;
                (void)me->port.serial_enable(me->port.ctx, 1U, 0U);
                break;

            default:
                /* IDLE/WAIT_PROCESS: nothing armed this timer here. */
                break;
        }
    } else if ( mode == (uint8_t)MODBUS_SLAVE_TIMER_TX_TIMEOUT ) {
        if ( me->tx_state == (uint8_t)MODBUS_SLAVE_TX_STATE_XMIT ) {
            /* The response will never drain. Release the line here, in the
             * interrupt: a shared RS485 segment stays blocked while DE is
             * asserted, so recovery must not depend on when the backend's
             * active object gets to run. */
            abort_tx(me);
            modbus_slave_notify_tx_timeout(me);
        }
    } else {
        /* Spurious expiry with no timer armed. */
    }
}

/* TX frame machine -------------------------------------------------------- */

void modbus_slave_rtu_tx_ready(modbus_slave_t* me)
{
    if ( me->tx_state == (uint8_t)MODBUS_SLAVE_TX_STATE_XMIT
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
         * moved the machine to IDLE. Mute TXE here as well. */
        (void)me->port.tx_done(me->port.ctx);
    }
}

void modbus_slave_rtu_tx_complete(modbus_slave_t* me)
{
    if ( me->tx_state != (uint8_t)MODBUS_SLAVE_TX_STATE_XMIT ) {
        return;
    }
    me->tx_state = (uint8_t)MODBUS_SLAVE_TX_STATE_IDLE;

    if ( me->timer_mode == (uint8_t)MODBUS_SLAVE_TIMER_TX_TIMEOUT ) {
        (void)me->port.timer_disable(me->port.ctx);
        me->timer_mode = (uint8_t)MODBUS_SLAVE_TIMER_NONE;
    }

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

    /* Arm the watchdog before enabling TX so no byte can go out unprotected.
     * Set its mode first: an expiry that lands immediately must find it. */
    me->tx_state   = (uint8_t)MODBUS_SLAVE_TX_STATE_XMIT;
    me->timer_mode = (uint8_t)MODBUS_SLAVE_TIMER_TX_TIMEOUT;
    (void)me->port.timer_enable(me->port.ctx, tx_timeout_us);

    /* RX off, DE on, TC cleared, TXE+TC interrupts enabled: the TXE
     * interrupt that follows delivers the first byte through tx_ready(). */
    (void)me->port.serial_enable(me->port.ctx, 0U, 1U);
}

static void abort_tx(modbus_slave_t* me)
{
    me->tx_state = (uint8_t)MODBUS_SLAVE_TX_STATE_IDLE;

    /* Resynchronize: bytes of the truncated response may still be in flight,
     * so the receiver must see a full T3.5 of idle before it accepts the next
     * frame. The state changes before reception opens, so no byte can slip
     * into a frame that is about to be discarded. */
    me->rx_state = (uint8_t)MODBUS_SLAVE_RX_STATE_INIT;
    me->rx_pos   = 0U;

    /* Disable TXE and TC (a pending TC is cleared with it) and release DE, so
     * the line is free for every slave again. */
    (void)me->port.serial_enable(me->port.ctx, 1U, 0U);

    me->timer_mode = (uint8_t)MODBUS_SLAVE_TIMER_RX_T35;
    (void)me->port.timer_enable(me->port.ctx, me->t35_us);
}
