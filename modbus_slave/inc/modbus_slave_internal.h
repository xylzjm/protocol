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

/** ADU byte index of the first reply payload byte: address, function, count. */
#define MODBUS_SLAVE_ADU_PAYLOAD_OFFSET 3U

/* Request verdict --------------------------------------------------------- */

/** Verdict of modbus_slave_adu_capture(): what the captured frame is. */
enum modbus_slave_capture {
    MODBUS_SLAVE_CAPTURE_IGNORE    = 0, /**< Dropped silently; reception must reopen. */
    MODBUS_SLAVE_CAPTURE_UNICAST,       /**< Addressed request that owes a response. */
    MODBUS_SLAVE_CAPTURE_BROADCAST      /**< Accepted broadcast write; never answered. */
};

/** Request kinds: the shapes the frame codec knows how to decode. */
enum modbus_slave_request_kind {
    MODBUS_SLAVE_REQ_READ_BITS = 0, /**< Function codes 0x01 and 0x02. */
    MODBUS_SLAVE_REQ_READ_REGS,     /**< 0x03 and 0x04. */
    MODBUS_SLAVE_REQ_WRITE_SINGLE_COIL,
    MODBUS_SLAVE_REQ_WRITE_SINGLE_REG,
    MODBUS_SLAVE_REQ_WRITE_BITS,
    MODBUS_SLAVE_REQ_WRITE_REGS,
    MODBUS_SLAVE_REQ_REPORT_ID
};

/** @brief Decoded request: what the frame asks for, and the bytes it came from. */
typedef struct {
    uint8_t  address;      /**< Address byte of the ADU. */
    uint8_t  function;     /**< Function code. */
    uint8_t  kind;         /**< enum modbus_slave_request_kind. */
    uint16_t pdu_length;   /**< Bytes of PDU in the receive buffer. */
    uint16_t start;        /**< First address. */
    uint16_t quantity;     /**< Items requested; 1 for a single write. */
    uint8_t* data;         /**< Write payload; read-only for the callback. */
    uint16_t data_length;  /**< Payload bytes. */
    uint8_t  coil_value;   /**< Packed bit for write-single-coil. */
} modbus_slave_request_t;

/** @brief Reply being built; the codec turns it into tx_buffer[1..]. */
typedef struct {
    uint8_t  function;   /**< 0 until a reply is built. */
    uint16_t byte_count; /**< Payload length of a read/report reply. */
    uint8_t  echo;       /**< The reply is a copy of the request head. */
    uint8_t  exception;  /**< Non-zero: exception reply. */
} modbus_slave_response_t;

/* Session ----------------------------------------------------------------- */

/** The one state the core is in. It implies what reception is doing and what
 *  the one-shot timer means, so the two can never disagree. */
enum modbus_slave_session {
    MODBUS_SLAVE_SESSION_STOPPED    = 0, /**< Out of service; reception off. */
    MODBUS_SLAVE_SESSION_DISCARDING,     /**< Reception on, bytes dropped, waiting out a T3.5. */
    MODBUS_SLAVE_SESSION_IDLE,           /**< Reception on, ready for the first byte. */
    MODBUS_SLAVE_SESSION_RECEIVING,      /**< Reception on, collecting frame bytes. */
    MODBUS_SLAVE_SESSION_CAPTURED,       /**< Frame frozen; it is the application's turn. */
    MODBUS_SLAVE_SESSION_SENDING         /**< Reply on the wire; the watchdog is armed. */
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

/** Enter DISCARDING: open reception and wait out a T3.5 of bus idle. */
void modbus_slave_rtu_resync(modbus_slave_t* me);

/** TXE machine: emit the next byte or tolerate the trailing empty interrupt. */
void modbus_slave_rtu_tx_ready(modbus_slave_t* me);

/** TC machine: close TX, reopen RX, notify the event port. */
void modbus_slave_rtu_tx_complete(modbus_slave_t* me);

/** Arm the TX watchdog and switch the port to transmit. */
void modbus_slave_rtu_start_tx(modbus_slave_t* me);

/* Frame codec (src/modbus_slave_adu.c) ------------------------------------ */

/** Capture the ADU in rx_buffer: length, CRC, address and broadcast policy. */
enum modbus_slave_capture modbus_slave_adu_capture(modbus_slave_t* me,
                                                   modbus_slave_request_t* req);

/** Decode the request shape: one validation order, one exception. */
modbus_slave_exception_t modbus_slave_adu_decode(modbus_slave_t* me,
                                                 modbus_slave_request_t* req);

/** Point the reply at its payload area and record the function code. */
uint8_t* modbus_slave_adu_payload(modbus_slave_t* me,
                                  modbus_slave_response_t* rep,
                                  uint8_t function);

/** Record the payload length of a read or report reply. */
void modbus_slave_adu_byte_count(modbus_slave_response_t* rep,
                                 uint16_t count);

/** Make the reply a copy of the request head (the write replies). */
void modbus_slave_adu_echo(modbus_slave_response_t* rep,
                           uint8_t function);

/** Make the reply an exception. */
void modbus_slave_adu_exception(modbus_slave_response_t* rep,
                                uint8_t function,
                                modbus_slave_exception_t exception);

/** Encode the reply into tx_buffer[1..] and return the reply PDU length. */
uint16_t modbus_slave_adu_finish(modbus_slave_t* me,
                                 const modbus_slave_response_t* rep);

/** Prepend the address, append the CRC and hand the ADU to the TX machine. */
void modbus_slave_adu_transmit(modbus_slave_t* me,
                               const modbus_slave_request_t* req,
                               uint16_t pdu_length);

/** Execute one decoded request: the function code decides the callback. */
void modbus_slave_fc_execute(modbus_slave_t* me,
                             const modbus_slave_request_t* req,
                             modbus_slave_response_t* rep);

/** Report a complete frame to the application (from the T3.5 interrupt). */
void modbus_slave_notify_frame_received(modbus_slave_t* me);

/** Report a completed transmission to the application (from the TC interrupt). */
void modbus_slave_notify_frame_sent(modbus_slave_t* me);

/** Report a TX timeout to the application (from the timer interrupt). */
void modbus_slave_notify_tx_timeout(modbus_slave_t* me);

#endif
