/* SPDX-License-Identifier: MIT */

#ifndef MODBUS_SLAVE_INTERNAL_H
#define MODBUS_SLAVE_INTERNAL_H

/**
 * @file modbus_slave_internal.h
 * @brief Constants, state and glue shared by the core translation units.
 *
 * Not part of the public API; applications and board modules include
 * modbus_slave.h.
 */

#include "modbus_slave.h"

#define MODBUS_SLAVE_ADDRESS_BROADCAST 0x00U
#define MODBUS_SLAVE_ADDRESS_MIN       0x01U
#define MODBUS_SLAVE_ADDRESS_MAX       0xF7U

#define MODBUS_SLAVE_ADU_SIZE_MAX    256U
#define MODBUS_SLAVE_ADU_PDU_OFFSET  1U
#define MODBUS_SLAVE_ADU_CRC_SIZE    2U
#define MODBUS_SLAVE_ADU_SIZE_MIN    4U
#define MODBUS_SLAVE_FC_ERROR_MASK   0x80U

#define MODBUS_SLAVE_FC_READ_COILS             0x01U
#define MODBUS_SLAVE_FC_READ_DISCRETE_INPUTS   0x02U
#define MODBUS_SLAVE_FC_READ_HOLDING_REGISTERS 0x03U
#define MODBUS_SLAVE_FC_READ_INPUT_REGISTERS   0x04U
#define MODBUS_SLAVE_FC_WRITE_SINGLE_COIL      0x05U
#define MODBUS_SLAVE_FC_WRITE_SINGLE_REGISTER  0x06U
#define MODBUS_SLAVE_FC_WRITE_MULTIPLE_COILS   0x0FU
#define MODBUS_SLAVE_FC_WRITE_MULTIPLE_REGS    0x10U
#define MODBUS_SLAVE_FC_REPORT_SLAVE_ID        0x11U

#define MODBUS_SLAVE_BITS_READ_MAX  2000U
#define MODBUS_SLAVE_BITS_WRITE_MAX 1968U
#define MODBUS_SLAVE_REGS_READ_MAX  125U
#define MODBUS_SLAVE_REGS_WRITE_MAX 123U

#define MODBUS_SLAVE_ADDITIONAL_MAX 249U

#define MODBUS_SLAVE_PDU_START_OFFSET      1U
#define MODBUS_SLAVE_PDU_QUANTITY_OFFSET   3U
#define MODBUS_SLAVE_PDU_BYTE_COUNT_OFFSET 5U
#define MODBUS_SLAVE_PDU_DATA_OFFSET       6U
#define MODBUS_SLAVE_ADU_PAYLOAD_OFFSET    3U

/** @brief Verdict of modbus_slave_adu_capture() on the captured frame. */
enum modbus_slave_capture {
    MODBUS_SLAVE_CAPTURE_IGNORE = 0,
    MODBUS_SLAVE_CAPTURE_UNICAST,
    MODBUS_SLAVE_CAPTURE_BROADCAST
};

/** @brief Request shapes the frame codec knows how to decode. */
enum modbus_slave_request_kind {
    MODBUS_SLAVE_REQ_READ_BITS = 0,
    MODBUS_SLAVE_REQ_READ_REGS,
    MODBUS_SLAVE_REQ_WRITE_SINGLE_COIL,
    MODBUS_SLAVE_REQ_WRITE_SINGLE_REG,
    MODBUS_SLAVE_REQ_WRITE_BITS,
    MODBUS_SLAVE_REQ_WRITE_REGS,
    MODBUS_SLAVE_REQ_REPORT_ID
};

/** @brief Decoded request and the bytes it came from. */
typedef struct {
    uint8_t  address;
    uint8_t  function;
    uint8_t  kind;         /**< enum modbus_slave_request_kind. */
    uint16_t pdu_length;
    uint16_t start;
    uint16_t quantity;
    uint8_t* data;         /**< Write payload. */
    uint16_t data_length;
    uint8_t  coil_value;   /**< Packed bit for write-single-coil. */
} modbus_slave_request_t;

/** @brief Reply being built by the frame codec. */
typedef struct {
    uint8_t  function;
    uint16_t byte_count;
    uint8_t  echo;      /**< The reply copies the request head. */
    uint8_t  exception; /**< Non-zero: exception reply. */
} modbus_slave_response_t;

/** @brief The single core state; it implies reception and the timer's meaning. */
enum modbus_slave_session {
    MODBUS_SLAVE_SESSION_STOPPED = 0, /**< Out of service. */
    MODBUS_SLAVE_SESSION_DISCARDING,  /**< Reception on, bytes dropped. */
    MODBUS_SLAVE_SESSION_IDLE,        /**< Reception on, ready for a frame. */
    MODBUS_SLAVE_SESSION_RECEIVING,   /**< Collecting frame bytes. */
    MODBUS_SLAVE_SESSION_CAPTURED,    /**< Frame frozen; application's turn. */
    MODBUS_SLAVE_SESSION_SENDING      /**< Reply on the wire. */
};

/** @brief CRC16/Modbus over @p buffer. */
uint16_t modbus_slave_crc(const uint8_t* buffer,
                          uint16_t length);

/** @brief RX byte machine: framing and T3.5 reload. */
void modbus_slave_rtu_rx_byte(modbus_slave_t* me,
                              uint8_t byte);

/** @brief Timer expiry: end of frame or TX timeout. */
void modbus_slave_rtu_timer_expired(modbus_slave_t* me);

/** @brief Enter DISCARDING and wait out one T3.5 of bus idle. */
void modbus_slave_rtu_resync(modbus_slave_t* me);

/** @brief TXE machine: emit the next byte. */
void modbus_slave_rtu_tx_ready(modbus_slave_t* me);

/** @brief TC machine: close TX, reopen RX, notify the event port. */
void modbus_slave_rtu_tx_complete(modbus_slave_t* me);

/** @brief Arm the TX watchdog and switch the line to transmit. */
void modbus_slave_rtu_start_tx(modbus_slave_t* me);

/** @brief Capture the ADU in rx_buffer: length, CRC, address, broadcast. */
enum modbus_slave_capture modbus_slave_adu_capture(
    modbus_slave_t* me,
    modbus_slave_request_t* req);

/** @brief Decode the request shape; one validation order, one exception. */
modbus_slave_exception_t modbus_slave_adu_decode(
    modbus_slave_t* me,
    modbus_slave_request_t* req);

/** @brief Point the reply at its payload and record the function code. */
uint8_t* modbus_slave_adu_payload(modbus_slave_t* me,
                                  modbus_slave_response_t* rep,
                                  uint8_t function);

/** @brief Record the payload length of a read or report reply. */
void modbus_slave_adu_byte_count(modbus_slave_response_t* rep,
                                 uint16_t count);

/** @brief Make the reply a copy of the request head. */
void modbus_slave_adu_echo(modbus_slave_response_t* rep,
                           uint8_t function);

/** @brief Make the reply an exception. */
void modbus_slave_adu_exception(modbus_slave_response_t* rep,
                                uint8_t function,
                                modbus_slave_exception_t exception);

/** @brief Encode the reply and return the reply PDU length. */
uint16_t modbus_slave_adu_finish(modbus_slave_t* me,
                                 const modbus_slave_response_t* rep);

/** @brief Add address and CRC, then start transmitting. */
void modbus_slave_adu_transmit(modbus_slave_t* me,
                               const modbus_slave_request_t* req,
                               uint16_t pdu_length);

/** @brief Execute one decoded request. */
void modbus_slave_fc_execute(modbus_slave_t* me,
                             const modbus_slave_request_t* req,
                             modbus_slave_response_t* rep);

/** @brief Report a complete frame to the application. */
void modbus_slave_notify_frame_received(modbus_slave_t* me);

/** @brief Report a completed transmission to the application. */
void modbus_slave_notify_frame_sent(modbus_slave_t* me);

/** @brief Report a TX timeout to the application. */
void modbus_slave_notify_tx_timeout(modbus_slave_t* me);

#endif
