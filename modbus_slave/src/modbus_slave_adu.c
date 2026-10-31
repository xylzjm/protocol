/* SPDX-License-Identifier: MIT */

#include "modbus_slave_internal.h"

/* RTU frame codec: the only module that knows the layout of an ADU
 * ([address][PDU][CRC16]) and the shape of every request PDU. It reads the
 * receive buffer, writes the transmit buffer, and never touches the port --
 * so decoding is a pure function of the captured bytes. */

/* Request shapes ---------------------------------------------------------- */

typedef struct {
    uint8_t function;      /**< Wire function code. */
    uint8_t kind;          /**< enum modbus_slave_request_kind. */
    uint8_t pdu_length;    /**< Exact PDU length; 0 = variable. */
    uint16_t quantity_max; /**< 0 = the PDU carries no quantity. */
    uint8_t units;         /**< 1 = bits, 2 = registers. */
} shape_t;

static const shape_t shapes[] = {
    {MODBUS_SLAVE_FC_READ_COILS, MODBUS_SLAVE_REQ_READ_BITS,
     5U, MODBUS_SLAVE_BITS_READ_MAX, 1U},
    {MODBUS_SLAVE_FC_READ_DISCRETE_INPUTS, MODBUS_SLAVE_REQ_READ_BITS,
     5U, MODBUS_SLAVE_BITS_READ_MAX, 1U},
    {MODBUS_SLAVE_FC_READ_HOLDING_REGISTERS, MODBUS_SLAVE_REQ_READ_REGS,
     5U, MODBUS_SLAVE_REGS_READ_MAX, 2U},
    {MODBUS_SLAVE_FC_READ_INPUT_REGISTERS, MODBUS_SLAVE_REQ_READ_REGS,
     5U, MODBUS_SLAVE_REGS_READ_MAX, 2U},
    {MODBUS_SLAVE_FC_WRITE_SINGLE_COIL, MODBUS_SLAVE_REQ_WRITE_SINGLE_COIL,
     5U, 1U, 0U},
    {MODBUS_SLAVE_FC_WRITE_SINGLE_REGISTER, MODBUS_SLAVE_REQ_WRITE_SINGLE_REG,
     5U, 1U, 0U},
    {MODBUS_SLAVE_FC_WRITE_MULTIPLE_COILS, MODBUS_SLAVE_REQ_WRITE_BITS,
     0U, MODBUS_SLAVE_BITS_WRITE_MAX, 1U},
    {MODBUS_SLAVE_FC_WRITE_MULTIPLE_REGS, MODBUS_SLAVE_REQ_WRITE_REGS,
     0U, MODBUS_SLAVE_REGS_WRITE_MAX, 2U},
    {MODBUS_SLAVE_FC_REPORT_SLAVE_ID, MODBUS_SLAVE_REQ_REPORT_ID,
     1U, 0U, 0U}};

static const shape_t* find_shape(uint8_t function)
{
    uint16_t i;

    for ( i = 0U; i < (uint16_t)(sizeof shapes / sizeof shapes[0]); ++i ) {
        if ( shapes[i].function == function ) {
            return &shapes[i];
        }
    }
    return NULL;
}

/* Capture ----------------------------------------------------------------- */

enum modbus_slave_capture modbus_slave_adu_capture(modbus_slave_t* me,
                                                   modbus_slave_request_t* req)
{
    uint16_t frame_len = me->rx_pos;
    uint16_t crc;
    uint8_t address;
    uint8_t mine;

    if ( frame_len < (uint16_t)MODBUS_SLAVE_ADU_SIZE_MIN ) {
        return MODBUS_SLAVE_CAPTURE_IGNORE;
    }

    crc = modbus_slave_crc(me->rx_buffer, (uint16_t)(frame_len - 2U));
    if ( me->rx_buffer[frame_len - 2U] != (uint8_t)(crc & 0xFFU)
         || me->rx_buffer[frame_len - 1U] != (uint8_t)(crc >> 8) ) {
        return MODBUS_SLAVE_CAPTURE_IGNORE;
    }

    /* The address is a question, not stored state: ask it once per frame so a
     * register write or a DIP-switch change applies to the next one. An
     * out-of-range answer leaves the slave silent instead of making every
     * frame look like a broadcast. */
    mine = me->callbacks.address(me->callbacks.ctx);
    if ( mine < (uint8_t)MODBUS_SLAVE_ADDRESS_MIN
         || mine > (uint8_t)MODBUS_SLAVE_ADDRESS_MAX ) {
        return MODBUS_SLAVE_CAPTURE_IGNORE;
    }

    address = me->rx_buffer[0];
    if ( address != mine
         && address != (uint8_t)MODBUS_SLAVE_ADDRESS_BROADCAST ) {
        return MODBUS_SLAVE_CAPTURE_IGNORE;
    }

    req->address    = address;
    req->function   = me->rx_buffer[1];
    req->pdu_length = (uint16_t)(frame_len - 3U); /* address + CRC */

    if ( address == (uint8_t)MODBUS_SLAVE_ADDRESS_BROADCAST ) {
        /* Only the write function codes are valid broadcasts. */
        switch ( req->function ) {
            case MODBUS_SLAVE_FC_WRITE_SINGLE_COIL:
            case MODBUS_SLAVE_FC_WRITE_SINGLE_REGISTER:
            case MODBUS_SLAVE_FC_WRITE_MULTIPLE_COILS:
            case MODBUS_SLAVE_FC_WRITE_MULTIPLE_REGS:
                return MODBUS_SLAVE_CAPTURE_BROADCAST;

            default:
                return MODBUS_SLAVE_CAPTURE_IGNORE;
        }
    }

    return MODBUS_SLAVE_CAPTURE_UNICAST;
}

/* Decode ------------------------------------------------------------------ */

modbus_slave_exception_t modbus_slave_adu_decode(modbus_slave_t* me,
                                                 modbus_slave_request_t* req)
{
    uint8_t* pdu         = &me->rx_buffer[MODBUS_SLAVE_ADU_PDU_OFFSET];
    const shape_t* shape = find_shape(req->function);
    uint16_t expected;

    if ( shape == NULL ) {
        return MODBUS_SLAVE_EX_ILLEGAL_FUNCTION;
    }
    req->kind = shape->kind;

    /* One validation order for every function code: length, then quantity,
     * then the byte count of a multiple write. */
    if ( shape->pdu_length != 0U ) {
        if ( req->pdu_length != (uint16_t)shape->pdu_length ) {
            return MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE;
        }
    } else if ( req->pdu_length < 7U ) {
        /* function + start + quantity + byte count + at least one byte. */
        return MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE;
    }

    switch ( shape->kind ) {
        case MODBUS_SLAVE_REQ_READ_BITS:
        case MODBUS_SLAVE_REQ_READ_REGS:
            req->start = modbus_slave_get_u16(
                &pdu[MODBUS_SLAVE_PDU_START_OFFSET]);
            req->quantity = modbus_slave_get_u16(
                &pdu[MODBUS_SLAVE_PDU_QUANTITY_OFFSET]);
            if ( req->quantity == 0U
                 || req->quantity > shape->quantity_max ) {
                return MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE;
            }
            break;

        case MODBUS_SLAVE_REQ_WRITE_SINGLE_COIL: {
            uint16_t value;

            req->start = modbus_slave_get_u16(
                &pdu[MODBUS_SLAVE_PDU_START_OFFSET]);
            req->quantity = 1U;
            value         = modbus_slave_get_u16(
                &pdu[MODBUS_SLAVE_PDU_QUANTITY_OFFSET]);
            if ( value != 0xFF00U && value != 0x0000U ) {
                return MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE;
            }
            req->coil_value = (uint8_t)((value == 0xFF00U) ? 1U : 0U);
            break;
        }

        case MODBUS_SLAVE_REQ_WRITE_SINGLE_REG:
            req->start = modbus_slave_get_u16(
                &pdu[MODBUS_SLAVE_PDU_START_OFFSET]);
            req->quantity    = 1U;
            req->data        = &pdu[MODBUS_SLAVE_PDU_QUANTITY_OFFSET];
            req->data_length = 2U;
            break;

        case MODBUS_SLAVE_REQ_WRITE_BITS:
        case MODBUS_SLAVE_REQ_WRITE_REGS:
            req->start = modbus_slave_get_u16(
                &pdu[MODBUS_SLAVE_PDU_START_OFFSET]);
            req->quantity = modbus_slave_get_u16(
                &pdu[MODBUS_SLAVE_PDU_QUANTITY_OFFSET]);
            if ( req->quantity == 0U
                 || req->quantity > shape->quantity_max ) {
                return MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE;
            }
            expected = (shape->units == 1U)
                           ? (uint16_t)((req->quantity + 7U) / 8U)
                           : (uint16_t)(req->quantity * 2U);
            if ( pdu[MODBUS_SLAVE_PDU_BYTE_COUNT_OFFSET] != (uint8_t)expected
                 || req->pdu_length != (uint16_t)(6U + expected) ) {
                return MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE;
            }
            req->data        = &pdu[MODBUS_SLAVE_PDU_DATA_OFFSET];
            req->data_length = expected;
            break;

        case MODBUS_SLAVE_REQ_REPORT_ID:
        default:
            break;
    }

    return MODBUS_SLAVE_EX_NONE;
}

/* Reply ------------------------------------------------------------------- */

uint8_t* modbus_slave_adu_payload(modbus_slave_t* me,
                                  modbus_slave_response_t* rep,
                                  uint8_t function)
{
    rep->function = function;
    return &me->tx_buffer[MODBUS_SLAVE_ADU_PAYLOAD_OFFSET];
}

void modbus_slave_adu_byte_count(modbus_slave_response_t* rep,
                                 uint16_t count)
{
    rep->byte_count = count;
}

void modbus_slave_adu_echo(modbus_slave_response_t* rep,
                           uint8_t function)
{
    rep->function = function;
    rep->echo     = 1U;
}

void modbus_slave_adu_exception(modbus_slave_response_t* rep,
                                uint8_t function,
                                modbus_slave_exception_t exception)
{
    rep->function  = function;
    rep->exception = (uint8_t)exception;
}

uint16_t modbus_slave_adu_finish(modbus_slave_t* me,
                                 const modbus_slave_response_t* rep)
{
    uint16_t length;
    uint16_t index;

    if ( rep->exception != 0U ) {
        me->tx_buffer[1] = (uint8_t)(rep->function | MODBUS_SLAVE_FC_ERROR_MASK);
        me->tx_buffer[2] = rep->exception;
        length           = 2U;
    } else if ( rep->echo != 0U ) {
        /* The write replies are the first five request PDU bytes: function,
         * address and value/quantity. */
        for ( index = 1U; index < 6U; ++index ) {
            me->tx_buffer[index] = me->rx_buffer[index];
        }
        length = 5U;
    } else {
        me->tx_buffer[1] = rep->function;
        me->tx_buffer[2] = (uint8_t)rep->byte_count;
        length           = (uint16_t)(2U + rep->byte_count);
    }

    return length;
}

void modbus_slave_adu_transmit(modbus_slave_t* me,
                               const modbus_slave_request_t* req,
                               uint16_t pdu_length)
{
    uint16_t crc;

    me->tx_buffer[0]               = req->address;
    crc                            = modbus_slave_crc(
        me->tx_buffer, (uint16_t)(pdu_length + 1U));
    me->tx_buffer[1U + pdu_length] = (uint8_t)(crc & 0xFFU);
    me->tx_buffer[2U + pdu_length] = (uint8_t)(crc >> 8);

    me->tx_len = (uint16_t)(1U + pdu_length
                            + (uint16_t)MODBUS_SLAVE_ADU_CRC_SIZE);
    me->tx_pos = 0U;

    modbus_slave_rtu_start_tx(me);
}
