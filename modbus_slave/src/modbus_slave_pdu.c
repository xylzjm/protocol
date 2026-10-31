/* SPDX-License-Identifier: MIT */

#include "modbus_slave_internal.h"

/* PDU execution: the nine supported function codes. Each handler calls the
 * matching callback and fills the reply; the frame codec (adu.c) owns every
 * byte layout, and modbus_slave_advance() sends the result. */

/* 0x01 / 0x02: Read Coils / Read Discrete Inputs -------------------------- */

static void read_bits(modbus_slave_t* me,
                      const modbus_slave_request_t* req,
                      modbus_slave_response_t* rep,
                      modbus_slave_bits_fn callback)
{
    uint8_t* payload = modbus_slave_adu_payload(me, rep, req->function);
    modbus_slave_exception_t exception;

    /* The callback packs the bits LSB-first, starting at the first requested
     * item, straight into the reply payload. */
    exception = callback(me->callbacks.ctx, req->start, req->quantity, payload,
                         MODBUS_SLAVE_REG_READ);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        modbus_slave_adu_exception(rep, req->function, exception);
        return;
    }

    modbus_slave_adu_byte_count(rep, (uint16_t)((req->quantity + 7U) / 8U));
}

/* 0x03 / 0x04: Read Holding / Input Registers ----------------------------- */

static void read_registers(modbus_slave_t* me,
                           const modbus_slave_request_t* req,
                           modbus_slave_response_t* rep,
                           modbus_slave_registers_fn callback)
{
    uint8_t* payload = modbus_slave_adu_payload(me, rep, req->function);
    modbus_slave_exception_t exception;

    exception = callback(me->callbacks.ctx, req->start, req->quantity, payload,
                         MODBUS_SLAVE_REG_READ);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        modbus_slave_adu_exception(rep, req->function, exception);
        return;
    }

    modbus_slave_adu_byte_count(rep, (uint16_t)(req->quantity * 2U));
}

/* 0x05: Write Single Coil ------------------------------------------------- */

static void write_single_coil(modbus_slave_t* me,
                              const modbus_slave_request_t* req,
                              modbus_slave_response_t* rep)
{
    uint8_t packed = req->coil_value;
    modbus_slave_exception_t exception;

    /* The callback sees one packed bit, the same layout 0x0F uses. */
    exception = me->callbacks.coils(me->callbacks.ctx, req->start, 1U, &packed,
                                    MODBUS_SLAVE_REG_WRITE);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        modbus_slave_adu_exception(rep, req->function, exception);
        return;
    }

    modbus_slave_adu_echo(rep, req->function);
}

/* 0x06: Write Single Register --------------------------------------------- */

static void write_single_register(modbus_slave_t* me,
                                  const modbus_slave_request_t* req,
                                  modbus_slave_response_t* rep)
{
    modbus_slave_exception_t exception;

    exception = me->callbacks.holding_registers(
        me->callbacks.ctx, req->start, 1U, req->data, MODBUS_SLAVE_REG_WRITE);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        modbus_slave_adu_exception(rep, req->function, exception);
        return;
    }

    modbus_slave_adu_echo(rep, req->function);
}

/* 0x0F / 0x10: Write Multiple Coils / Registers --------------------------- */

static void write_multiple(modbus_slave_t* me,
                           const modbus_slave_request_t* req,
                           modbus_slave_response_t* rep,
                           modbus_slave_registers_fn callback)
{
    modbus_slave_exception_t exception;

    exception = callback(me->callbacks.ctx, req->start, req->quantity,
                         req->data, MODBUS_SLAVE_REG_WRITE);
    if ( exception != MODBUS_SLAVE_EX_NONE ) {
        modbus_slave_adu_exception(rep, req->function, exception);
        return;
    }

    modbus_slave_adu_echo(rep, req->function);
}

/* 0x11: Report Slave ID --------------------------------------------------- */

static void report_slave_id(modbus_slave_t* me,
                            const modbus_slave_request_t* req,
                            modbus_slave_response_t* rep)
{
    uint8_t* payload    = modbus_slave_adu_payload(me, rep, req->function);
    uint16_t additional = me->identity.additional_length;
    uint16_t index;

    payload[0] = me->identity.slave_id;
    payload[1] = (uint8_t)((me->identity.is_running != 0U) ? 0xFFU : 0x00U);
    for ( index = 0U; index < additional; ++index ) {
        payload[2U + index] = me->identity.additional[index];
    }

    modbus_slave_adu_byte_count(rep, (uint16_t)(2U + additional));
}

/* Dispatcher -------------------------------------------------------------- */

void modbus_slave_pdu_execute(modbus_slave_t* me,
                              const modbus_slave_request_t* req,
                              modbus_slave_response_t* rep)
{
    switch ( req->function ) {
        case MODBUS_SLAVE_FC_READ_COILS:
            read_bits(me, req, rep, me->callbacks.coils);
            break;

        case MODBUS_SLAVE_FC_READ_DISCRETE_INPUTS:
            read_bits(me, req, rep, me->callbacks.discrete_inputs);
            break;

        case MODBUS_SLAVE_FC_READ_HOLDING_REGISTERS:
            read_registers(me, req, rep, me->callbacks.holding_registers);
            break;

        case MODBUS_SLAVE_FC_READ_INPUT_REGISTERS:
            read_registers(me, req, rep, me->callbacks.input_registers);
            break;

        case MODBUS_SLAVE_FC_WRITE_SINGLE_COIL:
            write_single_coil(me, req, rep);
            break;

        case MODBUS_SLAVE_FC_WRITE_SINGLE_REGISTER:
            write_single_register(me, req, rep);
            break;

        case MODBUS_SLAVE_FC_WRITE_MULTIPLE_COILS:
            write_multiple(me, req, rep, me->callbacks.coils);
            break;

        case MODBUS_SLAVE_FC_WRITE_MULTIPLE_REGS:
            write_multiple(me, req, rep, me->callbacks.holding_registers);
            break;

        case MODBUS_SLAVE_FC_REPORT_SLAVE_ID:
            report_slave_id(me, req, rep);
            break;

        default:
            /* adu_decode() rejects unsupported codes; keep the reply sane. */
            modbus_slave_adu_exception(rep, req->function,
                                       MODBUS_SLAVE_EX_ILLEGAL_FUNCTION);
            break;
    }
}
