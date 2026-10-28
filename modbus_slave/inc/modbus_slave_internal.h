/* SPDX-License-Identifier: MIT */

#ifndef MODBUS_SLAVE_INTERNAL_H
#define MODBUS_SLAVE_INTERNAL_H

/**
 * @file modbus_slave_internal.h
 * @brief Internal constants, state enumerations and cross-module glue.
 *
 * Shared by the modbus_slave core translation units only. Not part of the
 * public API: applications and board modules include modbus_slave.h.
 */

#include "modbus_slave.h"

/* Frame layout ------------------------------------------------------------ */

/** Address of a broadcast request; broadcast frames are never answered. */
#define MODBUS_SLAVE_ADDRESS_BROADCAST 0x00U

/** Smallest legal slave address. */
#define MODBUS_SLAVE_ADDRESS_MIN 0x01U

/** Largest legal slave address. */
#define MODBUS_SLAVE_ADDRESS_MAX 0xF7U

/** Largest RTU ADU: address + PDU(253) + CRC16. */
#define MODBUS_SLAVE_ADU_SIZE_MAX 256U

/** Byte offset of the PDU inside an ADU. */
#define MODBUS_SLAVE_ADU_PDU_OFFSET 1U

/** CRC16 trailer size, low byte first on the wire. */
#define MODBUS_SLAVE_ADU_CRC_SIZE 2U

/** Smallest possible request ADU: address + function + CRC16. */
#define MODBUS_SLAVE_ADU_SIZE_MIN 4U

/** Response marker: the function code is bitwise-OR'ed into an exception response. */
#define MODBUS_SLAVE_FC_ERROR_MASK 0x80U

/* Function codes ---------------------------------------------------------- */

#define MODBUS_SLAVE_FC_READ_COILS             0x01U
#define MODBUS_SLAVE_FC_READ_DISCRETE_INPUTS   0x02U
#define MODBUS_SLAVE_FC_READ_HOLDING_REGISTERS 0x03U
#define MODBUS_SLAVE_FC_READ_INPUT_REGISTERS   0x04U
#define MODBUS_SLAVE_FC_WRITE_SINGLE_COIL      0x05U
#define MODBUS_SLAVE_FC_WRITE_SINGLE_REGISTER  0x06U
#define MODBUS_SLAVE_FC_WRITE_MULTIPLE_COILS   0x0FU
#define MODBUS_SLAVE_FC_WRITE_MULTIPLE_REGS    0x10U
#define MODBUS_SLAVE_FC_REPORT_SLAVE_ID        0x11U

/* Quantity limits --------------------------------------------------------- */

#define MODBUS_SLAVE_BITS_READ_MAX  2000U
#define MODBUS_SLAVE_BITS_WRITE_MAX 1968U
#define MODBUS_SLAVE_REGS_READ_MAX  125U
#define MODBUS_SLAVE_REGS_WRITE_MAX 123U

/* 0x11 additional data cap: the RTU ADU is bounded at 256 bytes and a
 * Report Slave ID response needs 7 bytes around it (address, function, byte
 * count, slave id, run indicator, 2 CRC bytes), so at most 249 additional
 * bytes fit. */
#define MODBUS_SLAVE_ADDITIONAL_MAX 249U

/* Request PDU layout ------------------------------------------------------ */

/** PDU byte index of the 16-bit big-endian start address. */
#define MODBUS_SLAVE_PDU_START_OFFSET 1U

/** PDU byte index of the 16-bit big-endian quantity. */
#define MODBUS_SLAVE_PDU_QUANTITY_OFFSET 3U

/** PDU byte index of the byte count in 0x0F/0x10 requests. */
#define MODBUS_SLAVE_PDU_BYTE_COUNT_OFFSET 5U

/** PDU byte index of the first data byte in 0x0F/0x10 requests. */
#define MODBUS_SLAVE_PDU_DATA_OFFSET 6U

/* RX frame machine states ------------------------------------------------- */

enum modbus_slave_rx_state {
    MODBUS_SLAVE_RX_STATE_INIT         = 0, /**< Waiting for the first bus-idle window. */
    MODBUS_SLAVE_RX_STATE_IDLE         = 1, /**< Ready for the first byte of a frame. */
    MODBUS_SLAVE_RX_STATE_RCV          = 2, /**< Collecting frame bytes. */
    MODBUS_SLAVE_RX_STATE_WAIT_PROCESS = 3, /**< Frame complete, reception off, notify. */
    MODBUS_SLAVE_RX_STATE_ERROR        = 4  /**< Overlong frame; discard until bus idle. */
};

/* TX frame machine states ------------------------------------------------- */

enum modbus_slave_tx_state {
    MODBUS_SLAVE_TX_STATE_IDLE = 0,
    MODBUS_SLAVE_TX_STATE_XMIT = 1
};

/* Timer modes ------------------------------------------------------------- */

enum modbus_slave_timer_mode {
    MODBUS_SLAVE_TIMER_NONE       = 0,
    MODBUS_SLAVE_TIMER_RX_T35     = 1,
    MODBUS_SLAVE_TIMER_TX_TIMEOUT = 2
};

/* Cross-module glue ------------------------------------------------------- */

/** CRC16/Modbus over buffer[0..length-1], initial value 0xFFFF, reflected 0xA001. */
uint16_t modbus_slave_crc(const uint8_t* buffer,
                          uint16_t length);

/** RX byte machine: framing, T3.5 reload, overflow detection. */
void modbus_slave_rtu_rx_byte(modbus_slave_t* me,
                              uint8_t byte);

/** Timer expiry machine: T3.5 end-of-frame or TX timeout. */
void modbus_slave_rtu_timer_expired(modbus_slave_t* me);

/** TXE machine: emit the next byte or tolerate the trailing empty interrupt. */
void modbus_slave_rtu_tx_ready(modbus_slave_t* me);

/** TC machine: close TX, reopen RX, notify the backend. */
void modbus_slave_rtu_tx_complete(modbus_slave_t* me);

/** Assemble the ADU around the built response PDU and start transmitting. */
void modbus_slave_rtu_send(modbus_slave_t* me);

/** Abandon the transmission, release the bus, reopen reception. */
void modbus_slave_rtu_abort_tx(modbus_slave_t* me);

/** Execute the captured request; builds the response PDU into tx_buffer + 1. */
void modbus_slave_pdu_execute(modbus_slave_t* me);

/** Report a complete frame to the backend (from the T3.5 interrupt). */
void modbus_slave_notify_frame_received(modbus_slave_t* me);

/** Report a completed transmission to the backend (from the TC interrupt). */
void modbus_slave_notify_frame_sent(modbus_slave_t* me);

/** Report a TX timeout to the backend (from the timer interrupt). */
void modbus_slave_notify_tx_timeout(modbus_slave_t* me);

#endif
