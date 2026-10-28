/* SPDX-License-Identifier: MIT */

#include "modbus_slave_internal.h"

/* PDU execution: the nine supported function codes plus the exception path.
 * execute() builds the response PDU into tx_buffer[1..]; send() later wraps
 * it with the slave address and the CRC. A broadcast request builds its
 * response too — the backend simply never sends it. */

static void make_exception(modbus_slave_t* me,
                           modbus_slave_exception_t exception)
{
    me->tx_buffer[1] = (uint8_t)(me->function | MODBUS_SLAVE_FC_ERROR_MASK);
    me->tx_buffer[2] = (uint8_t)exception;
    me->response_len = 2U;
}

/* Request helpers --------------------------------------------------------- */

static uint16_t request_start(const modbus_slave_t* me)
{
    return modbus_slave_get_u16(&me->rx_buffer[MODBUS_SLAVE_ADU_PDU_OFFSET
                                               + MODBUS_SLAVE_PDU_START_OFFSET]);
}

static uint16_t request_quantity(const modbus_slave_t* me)
{
    return modbus_slave_get_u16(&me->rx_buffer[MODBUS_SLAVE_ADU_PDU_OFFSET
                                               + MODBUS_SLAVE_PDU_QUANTITY_OFFSET]);
}

static uint8_t request_byte_count(const modbus_slave_t* me)
{
    return me->rx_buffer[MODBUS_SLAVE_ADU_PDU_OFFSET
                         + MODBUS_SLAVE_PDU_BYTE_COUNT_OFFSET];
}

/* Data payload of 0x0F/0x10 requests: first data byte, after the byte count. */
static uint8_t* request_data(modbus_slave_t* me)
{
    return &me->rx_buffer[MODBUS_SLAVE_ADU_PDU_OFFSET
                          + MODBUS_SLAVE_PDU_DATA_OFFSET];
}

/* Echo the 5 request PDU bytes (function + address/value) for the write
 * functions that answer with an unchanged copy of their request. */
static void echo_address_quantity(modbus_slave_t* me)
{
    uint16_t index;

    for ( index = 1U; index < 6U; ++index ) {
        me->tx_buffer[index] = me->rx_buffer[index];
    }
    me->response_len = 5U;
}

/* 0x01 / 0x02: Read Coils / Read Discrete Inputs -------------------------- */

static void read_bits(modbus_slave_t* me,
                      modbus_slave_bits_fn callback)
{
    uint16_t count;
    uint8_t byte_count;
    modbus_slave_exception_t exception;

    if ( me->pdu_len != 5U ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    count = request_quantity(me);
    if ( count == 0U || count > MODBUS_SLAVE_BITS_READ_MAX ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    /* The callback packs the bits LSB-first straight into the response and
     * reports address errors (or SLAVE_BUSY) as an exception response. */
    exception = callback(me->callbacks.ctx, request_start(me), count,
                         &me->tx_buffer[3U], MODBUS_SLAVE_REG_READ);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        make_exception(me, exception);
        return;
    }

    byte_count       = (uint8_t)((count + 7U) / 8U);
    me->tx_buffer[1] = me->function;
    me->tx_buffer[2] = byte_count;
    me->response_len = (uint16_t)(2U + byte_count);
}

/* 0x03 / 0x04: Read Holding / Input Registers ----------------------------- */

static void read_registers(modbus_slave_t* me,
                           modbus_slave_registers_fn callback)
{
    uint16_t count;
    uint16_t byte_count;
    modbus_slave_exception_t exception;

    if ( me->pdu_len != 5U ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    count = request_quantity(me);
    if ( count == 0U || count > MODBUS_SLAVE_REGS_READ_MAX ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    /* The callback writes the registers big-endian into the response and
     * reports address errors (or SLAVE_BUSY) as an exception response. */
    exception = callback(me->callbacks.ctx, request_start(me), count,
                         &me->tx_buffer[3U], MODBUS_SLAVE_REG_READ);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        make_exception(me, exception);
        return;
    }

    byte_count       = (uint16_t)(count * 2U);
    me->tx_buffer[1] = me->function;
    me->tx_buffer[2] = (uint8_t)byte_count;
    me->response_len = (uint16_t)(2U + byte_count);
}

/* 0x05: Write Single Coil ------------------------------------------------- */

static void write_single_coil(modbus_slave_t* me)
{
    uint16_t value;
    uint8_t packed;
    modbus_slave_exception_t exception;

    if ( me->pdu_len != 5U ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    value = modbus_slave_get_u16(&me->rx_buffer[4U]);
    if ( value != 0xFF00U && value != 0x0000U ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    /* The callback sees one packed bit, the same layout 0x0F uses. */
    packed    = (uint8_t)((value == 0xFF00U) ? 0x01U : 0x00U);
    exception = me->callbacks.coils(me->callbacks.ctx, request_start(me), 1U,
                                    &packed, MODBUS_SLAVE_REG_WRITE);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        make_exception(me, exception);
        return;
    }

    echo_address_quantity(me);
}

/* 0x06: Write Single Register --------------------------------------------- */

static void write_single_register(modbus_slave_t* me)
{
    modbus_slave_exception_t exception;

    if ( me->pdu_len != 5U ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    /* The callback reads the 2 wire-order bytes of the request. */
    exception = me->callbacks.holding_registers(
        me->callbacks.ctx, request_start(me), 1U, &me->rx_buffer[4U],
        MODBUS_SLAVE_REG_WRITE);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        make_exception(me, exception);
        return;
    }

    echo_address_quantity(me);
}

/* 0x0F: Write Multiple Coils ---------------------------------------------- */

static void write_multiple_coils(modbus_slave_t* me)
{
    uint16_t count;
    uint16_t expected;
    modbus_slave_exception_t exception;

    /* Function + start + quantity + byte count + at least one data byte. */
    if ( me->pdu_len < 7U ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    count = request_quantity(me);
    if ( count == 0U || count > MODBUS_SLAVE_BITS_WRITE_MAX ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    expected = (uint16_t)((count + 7U) / 8U);
    if ( request_byte_count(me) != (uint8_t)expected
         || me->pdu_len != (uint16_t)(6U + expected) ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    exception = me->callbacks.coils(me->callbacks.ctx, request_start(me), count,
                                    request_data(me), MODBUS_SLAVE_REG_WRITE);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        make_exception(me, exception);
        return;
    }

    echo_address_quantity(me);
}

/* 0x10: Write Multiple Registers ------------------------------------------ */

static void write_multiple_registers(modbus_slave_t* me)
{
    uint16_t count;
    uint16_t expected;
    modbus_slave_exception_t exception;

    /* Function + start + quantity + byte count + at least one data byte. */
    if ( me->pdu_len < 7U ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    count = request_quantity(me);
    if ( count == 0U || count > MODBUS_SLAVE_REGS_WRITE_MAX ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    expected = (uint16_t)(count * 2U);
    if ( request_byte_count(me) != (uint8_t)expected
         || me->pdu_len != (uint16_t)(6U + expected) ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    exception = me->callbacks.holding_registers(
        me->callbacks.ctx, request_start(me), count, request_data(me),
        MODBUS_SLAVE_REG_WRITE);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        make_exception(me, exception);
        return;
    }

    echo_address_quantity(me);
}

/* 0x11: Report Slave ID --------------------------------------------------- */

static void report_slave_id(modbus_slave_t* me)
{
    uint16_t additional;
    uint16_t index;

    if ( me->pdu_len != 1U ) {
        make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    me->tx_buffer[1] = me->function;
    me->tx_buffer[3] = me->identity.slave_id;
    me->tx_buffer[4] = (uint8_t)((me->identity.is_running != 0U) ? 0xFFU : 0x00U);

    additional = me->identity.additional_length;
    for ( index = 0U; index < additional; ++index ) {
        me->tx_buffer[5U + index] = me->identity.additional[index];
    }

    me->tx_buffer[2] = (uint8_t)(2U + additional); /* id + run + extra */
    me->response_len = (uint16_t)(4U + additional);
}

/* Dispatcher -------------------------------------------------------------- */

void modbus_slave_pdu_execute(modbus_slave_t* me)
{
    switch ( me->function ) {
        case MODBUS_SLAVE_FC_READ_COILS:
            read_bits(me, me->callbacks.coils);
            break;

        case MODBUS_SLAVE_FC_READ_DISCRETE_INPUTS:
            read_bits(me, me->callbacks.discrete_inputs);
            break;

        case MODBUS_SLAVE_FC_READ_HOLDING_REGISTERS:
            read_registers(me, me->callbacks.holding_registers);
            break;

        case MODBUS_SLAVE_FC_READ_INPUT_REGISTERS:
            read_registers(me, me->callbacks.input_registers);
            break;

        case MODBUS_SLAVE_FC_WRITE_SINGLE_COIL:
            write_single_coil(me);
            break;

        case MODBUS_SLAVE_FC_WRITE_SINGLE_REGISTER:
            write_single_register(me);
            break;

        case MODBUS_SLAVE_FC_WRITE_MULTIPLE_COILS:
            write_multiple_coils(me);
            break;

        case MODBUS_SLAVE_FC_WRITE_MULTIPLE_REGS:
            write_multiple_registers(me);
            break;

        case MODBUS_SLAVE_FC_REPORT_SLAVE_ID:
            report_slave_id(me);
            break;

        default:
            make_exception(me, MODBUS_SLAVE_EX_ILLEGAL_FUNCTION);
            break;
    }
}
