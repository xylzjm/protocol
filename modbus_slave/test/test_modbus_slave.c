/* SPDX-License-Identifier: MIT */

/*
 * Host test for the modbus_slave core.
 *
 * Drives the public seam -- advance(), the ISR entry points, start/stop -- with
 * a fake board and checks the bytes that would reach the wire. A second part
 * checks the frame codec's request shapes directly: decoding needs no port and
 * no interrupt, only the captured bytes.
 *
 * No framework and no instrumentation: build the core sources plus this file.
 *
 * The expected ADU is built here with an independent bit-wise CRC16, so a bug
 * in the table-driven implementation is not masked by the test.
 */

#include <stdio.h>
#include <string.h>

#include "modbus_slave_internal.h"

#define SLAVE_ADDR 1U
#define BAUDRATE   9600U
#define T35_US     4011U /* ceil(38500000 / 9600) */

static int checks;
static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        checks++;                                                            \
        if ( !(cond) ) {                                                     \
            failures++;                                                      \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                    \
    } while ( 0 )

/* Fake board -------------------------------------------------------------- */

typedef struct {
    uint8_t  rx_open;
    uint8_t  tx_open;
    uint8_t  de;
    uint8_t  irq_enabled;
    uint8_t  timer_armed;
    uint32_t timer_us;
    uint32_t crit_depth;
    uint8_t  tx_bytes[300];
    int      tx_len;
    uint8_t  tx_done;
    int      serial_calls;
    int      timer_arms;
    /* event port */
    int      ev_frame;
    int      ev_sent;
    int      ev_timeout;
} fake_t;

static fake_t port;

static modbus_slave_status_t fake_direction(void* ctx,
                                            uint8_t rx,
                                            uint8_t tx)
{
    fake_t* f = (fake_t*)ctx;

    f->serial_calls++;
    f->rx_open = rx;
    f->tx_open = tx;
    f->de      = tx;
    return MODBUS_SLAVE_OK;
}

static modbus_slave_status_t fake_put_byte(void* ctx,
                                           uint8_t byte)
{
    fake_t* f = (fake_t*)ctx;

    if ( f->tx_len < (int)sizeof f->tx_bytes ) {
        f->tx_bytes[f->tx_len] = byte;
        f->tx_len++;
    }
    return MODBUS_SLAVE_OK;
}

static modbus_slave_status_t fake_tx_done(void* ctx)
{
    fake_t* f = (fake_t*)ctx;

    f->tx_done = 1U;
    return MODBUS_SLAVE_OK;
}

static modbus_slave_status_t fake_arm(void* ctx,
                                      uint32_t timeout_us)
{
    fake_t* f = (fake_t*)ctx;

    f->timer_armed = 1U;
    f->timer_us    = timeout_us;
    f->timer_arms++;
    return MODBUS_SLAVE_OK;
}

static modbus_slave_status_t fake_cancel(void* ctx)
{
    fake_t* f = (fake_t*)ctx;

    f->timer_armed = 0U;
    return MODBUS_SLAVE_OK;
}

/* The nesting counter the port is required to keep: interrupts come back only
 * when the outermost section exits. */
static void fake_enter(void* ctx)
{
    fake_t* f = (fake_t*)ctx;

    f->irq_enabled = 0U;
    f->crit_depth++;
}

static void fake_exit(void* ctx)
{
    fake_t* f = (fake_t*)ctx;

    f->crit_depth--;
    if ( f->crit_depth == 0U ) {
        f->irq_enabled = 1U;
    }
}

static void fake_on_frame_received(void* ctx)
{
    ((fake_t*)ctx)->ev_frame++;
}

static void fake_on_frame_sent(void* ctx)
{
    ((fake_t*)ctx)->ev_sent++;
}

static void fake_on_tx_timeout(void* ctx)
{
    ((fake_t*)ctx)->ev_timeout++;
}

/* Data space -------------------------------------------------------------- */

typedef struct {
    uint16_t holding[8];
    uint16_t input[8];
    uint8_t  coils[16];
    uint8_t  discrete[16];
    uint8_t  address_offset; /* stands in for the DIP switch */
    int      calls;
} store_t;

static store_t store;

/* The address register (holding[7]) is the base; the offset plays the part of
 * the DIP switch. Out-of-range combinations are the core's to reject. */
static uint8_t test_address(void* ctx)
{
    (void)ctx;
    return (uint8_t)(store.holding[7] + store.address_offset);
}

static modbus_slave_exception_t bits_impl(uint8_t* bits,
                                          uint16_t start,
                                          uint16_t count,
                                          uint8_t* buffer,
                                          modbus_slave_register_mode_t mode)
{
    uint16_t i;
    uint16_t byte_count;

    if ( (uint32_t)start + count > 16U ) {
        return MODBUS_SLAVE_EX_ILLEGAL_DATA_ADDRESS;
    }

    if ( mode == MODBUS_SLAVE_REG_READ ) {
        byte_count = (uint16_t)((count + 7U) / 8U);
        for ( i = 0U; i < byte_count; ++i ) {
            buffer[i] = 0U;
        }
    }

    for ( i = 0U; i < count; ++i ) {
        uint16_t addr = (uint16_t)(start + i);
        uint8_t  mask = (uint8_t)(1U << (i & 7U));
        uint8_t* cell = &buffer[i / 8U];

        if ( mode == MODBUS_SLAVE_REG_READ ) {
            if ( bits[addr] != 0U ) {
                *cell = (uint8_t)(*cell | mask);
            }
        } else {
            bits[addr] = (uint8_t)((*cell & mask) != 0U ? 1U : 0U);
        }
    }
    return MODBUS_SLAVE_EX_NONE;
}

static modbus_slave_exception_t coils_cb(void* ctx,
                                         uint16_t start,
                                         uint16_t count,
                                         uint8_t* buffer,
                                         modbus_slave_register_mode_t mode)
{
    (void)ctx;
    store.calls++;
    return bits_impl(store.coils, start, count, buffer, mode);
}

static modbus_slave_exception_t discrete_cb(void* ctx,
                                            uint16_t start,
                                            uint16_t count,
                                            uint8_t* buffer,
                                            modbus_slave_register_mode_t mode)
{
    (void)ctx;
    store.calls++;
    return bits_impl(store.discrete, start, count, buffer, mode);
}

static modbus_slave_exception_t regs_impl(uint16_t* regs,
                                          uint16_t start,
                                          uint16_t count,
                                          uint8_t* buffer,
                                          modbus_slave_register_mode_t mode)
{
    uint16_t i;

    if ( (uint32_t)start + count > 8U ) {
        return MODBUS_SLAVE_EX_ILLEGAL_DATA_ADDRESS;
    }
    for ( i = 0U; i < count; ++i ) {
        if ( mode == MODBUS_SLAVE_REG_READ ) {
            modbus_slave_put_u16(&buffer[2U * i], regs[start + i]);
        } else {
            regs[start + i] = modbus_slave_get_u16(&buffer[2U * i]);
        }
    }
    return MODBUS_SLAVE_EX_NONE;
}

static modbus_slave_exception_t holding_cb(void* ctx,
                                           uint16_t start,
                                           uint16_t count,
                                           uint8_t* buffer,
                                           modbus_slave_register_mode_t mode)
{
    (void)ctx;
    store.calls++;
    return regs_impl(store.holding, start, count, buffer, mode);
}

static modbus_slave_exception_t input_cb(void* ctx,
                                         uint16_t start,
                                         uint16_t count,
                                         uint8_t* buffer,
                                         modbus_slave_register_mode_t mode)
{
    (void)ctx;
    store.calls++;
    return regs_impl(store.input, start, count, buffer, mode);
}

/* Harness ----------------------------------------------------------------- */

static modbus_slave_t slave;
static uint8_t        rx_buffer[256];
static uint8_t        tx_buffer[256];

static uint16_t crc16(const uint8_t* data,
                      int length)
{
    uint16_t crc = 0xFFFFU;
    int      i;
    int      bit;

    for ( i = 0; i < length; ++i ) {
        crc = (uint16_t)(crc ^ data[i]);
        for ( bit = 0; bit < 8; ++bit ) {
            if ( (crc & 1U) != 0U ) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

static int in_service;

static int setup(void)
{
    modbus_slave_config_t     cfg;
    modbus_slave_port_event_t events;

    if ( in_service != 0 ) {
        /* The core must be stopped before it is constructed again. */
        (void)modbus_slave_stop(&slave);
        in_service = 0;
    }

    memset(&port, 0, sizeof port);
    port.irq_enabled = 1U; /* interrupts start enabled */
    memset(&store, 0, sizeof store);
    memset(&slave, 0, sizeof slave);
    memset(&cfg, 0, sizeof cfg);

    /* Holding register 7 is the address base; the offset is the DIP switch. */
    store.holding[7] = (uint16_t)SLAVE_ADDR;

    cfg.baudrate    = BAUDRATE;
    cfg.rx_buffer   = rx_buffer;
    cfg.rx_capacity = (uint16_t)sizeof rx_buffer;
    cfg.tx_buffer   = tx_buffer;
    cfg.tx_capacity = (uint16_t)sizeof tx_buffer;

    cfg.line.ctx       = &port;
    cfg.line.direction = fake_direction;
    cfg.line.put_byte  = fake_put_byte;
    cfg.line.tx_done   = fake_tx_done;

    cfg.timer.ctx  = &port;
    cfg.timer.arm  = fake_arm;
    cfg.timer.cancel = fake_cancel;

    cfg.critical.ctx   = &port;
    cfg.critical.enter = fake_enter;
    cfg.critical.exit  = fake_exit;

    cfg.callbacks.ctx               = &store;
    cfg.callbacks.coils             = coils_cb;
    cfg.callbacks.discrete_inputs   = discrete_cb;
    cfg.callbacks.holding_registers = holding_cb;
    cfg.callbacks.input_registers   = input_cb;
    cfg.callbacks.address           = test_address;

    cfg.identity.slave_id         = 0x22U;
    cfg.identity.is_running       = 1U;
    cfg.identity.additional       = NULL;
    cfg.identity.additional_length = 0U;

    CHECK(modbus_slave_ctor(&slave, &cfg) == MODBUS_SLAVE_OK);

    events.ctx               = &port;
    events.on_frame_received = fake_on_frame_received;
    events.on_frame_sent     = fake_on_frame_sent;
    events.on_tx_timeout     = fake_on_tx_timeout;
    CHECK(modbus_slave_set_event_port(&slave, &events) == MODBUS_SLAVE_OK);

    CHECK(modbus_slave_start(&slave) == MODBUS_SLAVE_OK);
    in_service = 1;
    return 0;
}

/* One one-shot timer expiry: the T3.5 gap, or the TX watchdog. */
static void gap(void)
{
    (void)modbus_slave_timer_expired(&slave);
}

static void feed_frame(uint8_t address,
                       const uint8_t* body,
                       int body_len,
                       int corrupt)
{
    uint8_t  frame[300];
    int      n = 0;
    int      i;
    uint16_t crc;

    frame[n] = address;
    n++;
    for ( i = 0; i < body_len; ++i ) {
        frame[n] = body[i];
        n++;
    }
    crc         = crc16(frame, n);
    frame[n]    = (uint8_t)(crc & 0xFFU);
    n++;
    frame[n] = (uint8_t)(crc >> 8);
    n++;
    if ( corrupt != 0 ) {
        frame[n - 1] = (uint8_t)(frame[n - 1] ^ 0x5AU);
    }

    gap(); /* resynchronize if the core is waiting out a T3.5 */
    for ( i = 0; i < n; ++i ) {
        (void)modbus_slave_rx_byte(&slave, frame[i]);
    }
    gap(); /* the inter-frame gap that completes the frame */
}

static void feed(uint8_t address,
                 const uint8_t* body,
                 int body_len)
{
    feed_frame(address, body, body_len, 0);
}

/* Forget the bytes already transmitted, so the next reply starts at index 0. */
static void clear_tx(void)
{
    port.tx_len = 0;
}

/* Drive TXE until the whole ADU has been written, then close with TC. */
static void finish_tx(void)
{
    int i;

    for ( i = 0; i < 64; ++i ) {
        (void)modbus_slave_tx_ready(&slave);
    }
    (void)modbus_slave_tx_complete(&slave);
}

static void expect_adu_at(uint8_t address,
                          const uint8_t* pdu,
                          int pdu_len)
{
    uint8_t  expected[300];
    int      n = 0;
    int      i;
    uint16_t crc;

    expected[n] = address;
    n++;
    for ( i = 0; i < pdu_len; ++i ) {
        expected[n] = pdu[i];
        n++;
    }
    crc        = crc16(expected, n);
    expected[n] = (uint8_t)(crc & 0xFFU);
    n++;
    expected[n] = (uint8_t)(crc >> 8);
    n++;

    CHECK(port.tx_len == n);
    if ( port.tx_len == n ) {
        if ( memcmp(port.tx_bytes, expected, (size_t)n) != 0 ) {
            int k;
            printf("  ADU mismatch\n    expected:");
            for ( k = 0; k < n; ++k ) {
                printf(" %02X", expected[k]);
            }
            printf("\n    actual:  ");
            for ( k = 0; k < n; ++k ) {
                printf(" %02X", port.tx_bytes[k]);
            }
            printf("\n");
        }
        CHECK(memcmp(port.tx_bytes, expected, (size_t)n) == 0);
    }
}

static void expect_adu(const uint8_t* pdu,
                       int pdu_len)
{
    expect_adu_at((uint8_t)SLAVE_ADDR, pdu, pdu_len);
}

static void expect_exception(uint8_t function,
                             uint8_t exception)
{
    uint8_t pdu[2];

    pdu[0] = (uint8_t)(function | 0x80U);
    pdu[1] = exception;
    expect_adu(pdu, 2);
}

/* Cases ------------------------------------------------------------------- */

static void test_start(void)
{
    setup();
    CHECK(port.rx_open == 1U);
    CHECK(port.tx_open == 0U);
    CHECK(port.timer_armed == 1U);
    CHECK(port.timer_us == T35_US);
    CHECK(modbus_slave_get_address(&slave) == SLAVE_ADDR);

    gap(); /* start-up: the first T3.5 takes the RX machine to IDLE */

    /* A frame event with nothing captured is stale and changes nothing. */
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_NONE);
    CHECK(port.serial_calls > 0);
}

static void test_read_holding(void)
{
    const uint8_t req[] = { 0x03U, 0x00U, 0x00U, 0x00U, 0x02U };
    const uint8_t pdu[] = { 0x03U, 0x04U, 0x12U, 0x34U, 0x56U, 0x78U };

    setup();
    store.holding[0] = 0x1234U;
    store.holding[1] = 0x5678U;

    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(port.ev_frame == 1);

    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    CHECK(port.tx_open == 1U);
    CHECK(port.rx_open == 0U);
    CHECK(port.timer_us > 10000U); /* the watchdog, not the T3.5 */

    /* A second frame event while the reply is on the wire changes nothing. */
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);

    finish_tx();
    CHECK(port.ev_sent == 1);
    CHECK(port.tx_done == 1U);
    CHECK(port.rx_open == 1U);
    CHECK(port.tx_open == 0U);
    expect_adu(pdu, (int)sizeof pdu);
    CHECK(store.calls == 1);
}

static void test_read_coils(void)
{
    const uint8_t req[] = { 0x01U, 0x00U, 0x00U, 0x00U, 0x0AU };
    const uint8_t pdu[] = { 0x01U, 0x02U, 0xCFU, 0x01U };

    setup();
    store.coils[0] = 1U;
    store.coils[1] = 1U;
    store.coils[2] = 1U;
    store.coils[3] = 1U;
    store.coils[6] = 1U;
    store.coils[7] = 1U;
    store.coils[8] = 1U;

    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu(pdu, (int)sizeof pdu);
}

static void test_write_single_coil(void)
{
    const uint8_t req[] = { 0x05U, 0x00U, 0x03U, 0xFFU, 0x00U };

    setup();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu(req, (int)sizeof req); /* the response echoes the request */
    CHECK(store.coils[3] == 1U);
}

static void test_write_multiple_registers(void)
{
    const uint8_t req[] = { 0x10U, 0x00U, 0x00U, 0x00U, 0x02U,
                            0x04U, 0xBEU, 0xEFU, 0x01U, 0x02U };
    const uint8_t pdu[] = { 0x10U, 0x00U, 0x00U, 0x00U, 0x02U };

    setup();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu(pdu, (int)sizeof pdu);
    CHECK(store.holding[0] == 0xBEEFU);
    CHECK(store.holding[1] == 0x0102U);
}

static void test_report_slave_id(void)
{
    const uint8_t req[] = { 0x11U };
    const uint8_t pdu[] = { 0x11U, 0x02U, 0x22U, 0xFFU };

    setup();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu(pdu, (int)sizeof pdu);
}

static void test_read_discrete(void)
{
    const uint8_t req[] = { 0x02U, 0x00U, 0x00U, 0x00U, 0x0AU };
    const uint8_t pdu[] = { 0x02U, 0x02U, 0x02U, 0x00U };

    setup();
    store.discrete[1] = 1U;

    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu(pdu, (int)sizeof pdu);
}

static void test_read_input(void)
{
    const uint8_t req[] = { 0x04U, 0x00U, 0x00U, 0x00U, 0x02U };
    const uint8_t pdu[] = { 0x04U, 0x04U, 0xAAU, 0x55U, 0x00U, 0x11U };

    setup();
    store.input[0] = 0xAA55U;
    store.input[1] = 0x0011U;

    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu(pdu, (int)sizeof pdu);
}

static void test_write_single_register(void)
{
    const uint8_t req[] = { 0x06U, 0x00U, 0x02U, 0xBEU, 0xEFU };

    setup();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu(req, (int)sizeof req);
    CHECK(store.holding[2] == 0xBEEFU);
}

static void test_write_multiple_coils(void)
{
    const uint8_t req[] = { 0x0FU, 0x00U, 0x00U, 0x00U, 0x0AU,
                            0x02U, 0xCFU, 0x01U };
    const uint8_t pdu[] = { 0x0FU, 0x00U, 0x00U, 0x00U, 0x0AU };

    setup();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu(pdu, (int)sizeof pdu);
    CHECK(store.coils[0] == 1U);
    CHECK(store.coils[3] == 1U);
    CHECK(store.coils[4] == 0U);
    CHECK(store.coils[6] == 1U);
    CHECK(store.coils[8] == 1U);
}

static void test_write_multiple_coils_offset(void)
{
    const uint8_t req[] = { 0x0FU, 0x00U, 0x03U, 0x00U, 0x0AU,
                            0x02U, 0xFFU, 0x03U };
    const uint8_t pdu[] = { 0x0FU, 0x00U, 0x03U, 0x00U, 0x0AU };

    setup();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu(pdu, (int)sizeof pdu);

    /* Data bit i belongs to coil start + i, even when start is not on a
     * byte boundary. */
    CHECK(store.coils[2] == 0U);
    CHECK(store.coils[3] == 1U);
    CHECK(store.coils[10] == 1U);
    CHECK(store.coils[11] == 1U);
    CHECK(store.coils[12] == 1U);
    CHECK(store.coils[13] == 0U);
}

static void test_broadcast_write(void)
{
    const uint8_t req[] = { 0x06U, 0x00U, 0x01U, 0xABU, 0xCDU };

    setup();
    feed(0U, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_HANDLED);
    CHECK(port.tx_len == 0);
    CHECK(port.tx_open == 0U);
    CHECK(port.rx_open == 1U); /* the core re-opened reception itself */
    CHECK(port.ev_sent == 0);
    CHECK(store.holding[1] == 0xABCDU);
}

static void test_bad_crc(void)
{
    const uint8_t req[] = { 0x03U, 0x00U, 0x00U, 0x00U, 0x01U };

    setup();
    feed_frame(SLAVE_ADDR, req, (int)sizeof req, 1);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_HANDLED);
    CHECK(port.tx_len == 0);
    CHECK(port.rx_open == 1U);
    CHECK(store.calls == 0);
}

static void test_foreign_address(void)
{
    const uint8_t req[] = { 0x03U, 0x00U, 0x00U, 0x00U, 0x01U };

    setup();
    feed(7U, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_HANDLED);
    CHECK(port.tx_len == 0);
    CHECK(port.rx_open == 1U);
    CHECK(store.calls == 0);
}

static void test_illegal_function(void)
{
    const uint8_t req[] = { 0x08U, 0x00U, 0x00U, 0x12U, 0x34U };

    setup();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_exception(0x08U, MODBUS_SLAVE_EX_ILLEGAL_FUNCTION);
}

static void test_bad_quantity(void)
{
    const uint8_t req[] = { 0x03U, 0x00U, 0x00U, 0x00U, 0x00U };

    setup();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_exception(0x03U, MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
    CHECK(store.calls == 0);
}

static void test_out_of_range(void)
{
    const uint8_t req[] = { 0x03U, 0x00U, 0x06U, 0x00U, 0x04U };

    setup();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_exception(0x03U, MODBUS_SLAVE_EX_ILLEGAL_DATA_ADDRESS);
    CHECK(store.calls == 1);
}

static void test_tx_timeout(void)
{
    const uint8_t req[] = { 0x03U, 0x00U, 0x00U, 0x00U, 0x01U };

    setup();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    CHECK(port.timer_us > 10000U);

    gap(); /* the watchdog expires in the timer interrupt */
    CHECK(port.ev_timeout == 1);
    CHECK(port.rx_open == 1U);   /* DE released, reception back on */
    CHECK(port.tx_open == 0U);
    CHECK(port.timer_armed == 1U);
    CHECK(port.timer_us == T35_US); /* resynchronizing */

    /* The transaction is over, and reception is waiting out the gap. */
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_NONE);

    gap();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
}

static void test_address_callback(void)
{
    const uint8_t wr[]     = { 0x06U, 0x00U, 0x07U, 0x00U, 0x02U };
    const uint8_t rd[]     = { 0x03U, 0x00U, 0x00U, 0x00U, 0x01U };
    const uint8_t rd_pdu[] = { 0x03U, 0x02U, 0x00U, 0x00U };

    setup();
    CHECK(modbus_slave_get_address(&slave) == SLAVE_ADDR);

    /* A write to the address register is answered by the address the master
     * used; the new address applies from the next frame on. */
    feed(SLAVE_ADDR, wr, (int)sizeof wr);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu(wr, (int)sizeof wr);
    CHECK(modbus_slave_get_address(&slave) == 2U);
    clear_tx();

    /* The old address is no longer ours: dropped, no callback, no reply. */
    feed(SLAVE_ADDR, rd, (int)sizeof rd);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_HANDLED);
    CHECK(port.tx_len == 0);
    CHECK(store.calls == 1);

    /* The new address answers. */
    feed(2U, rd, (int)sizeof rd);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu_at(2U, rd_pdu, (int)sizeof rd_pdu);
    CHECK(store.calls == 2);
    clear_tx();

    /* The DIP offset is added on top, with no apply step. */
    store.address_offset = 1U;
    CHECK(modbus_slave_get_address(&slave) == 3U);
    feed(3U, rd, (int)sizeof rd);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);
    finish_tx();
    expect_adu_at(3U, rd_pdu, (int)sizeof rd_pdu);
    clear_tx();

    /* An out-of-range address silences the slave, even for the broadcast
     * address. */
    store.holding[7]     = 0U;
    store.address_offset = 0U;
    feed(0U, wr, (int)sizeof wr);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_HANDLED);
    CHECK(port.tx_len == 0);
}

static void test_start_mid_tx(void)
{
    const uint8_t req[] = { 0x03U, 0x00U, 0x00U, 0x00U, 0x01U };

    setup();
    feed(SLAVE_ADDR, req, (int)sizeof req);
    CHECK(modbus_slave_advance(&slave) == MODBUS_SLAVE_STEP_SENDING);

    CHECK(modbus_slave_start(&slave) == MODBUS_SLAVE_ERR_STATE);
    CHECK(port.rx_open == 0U); /* reception was not switched on under TX */

    finish_tx();
    CHECK(modbus_slave_start(&slave) == MODBUS_SLAVE_OK);
}

/* Frame codec: request shapes --------------------------------------------- */

/* Capture one ADU into the receive buffer and run the decoder on it. No port
 * is involved: the codec only reads bytes and fills the request view. */
static void check_shape(const uint8_t* pdu,
                        uint16_t pdu_len,
                        uint8_t kind,
                        uint16_t start,
                        uint16_t quantity,
                        modbus_slave_exception_t expected)
{
    modbus_slave_request_t req = { 0U };
    uint16_t n = 0U;
    uint16_t crc;
    uint16_t i;

    rx_buffer[n] = (uint8_t)SLAVE_ADDR;
    n++;
    for ( i = 0U; i < pdu_len; ++i ) {
        rx_buffer[n] = pdu[i];
        n++;
    }
    crc          = crc16(rx_buffer, (int)n);
    rx_buffer[n] = (uint8_t)(crc & 0xFFU);
    n++;
    rx_buffer[n] = (uint8_t)(crc >> 8);
    n++;
    slave.rx_pos = n;

    CHECK(modbus_slave_adu_capture(&slave, &req) == MODBUS_SLAVE_CAPTURE_UNICAST);
    CHECK(req.function == pdu[0]);
    CHECK(modbus_slave_adu_decode(&slave, &req) == expected);
    if ( expected == MODBUS_SLAVE_EX_NONE ) {
        CHECK(req.kind == kind);
        CHECK(req.start == start);
        CHECK(req.quantity == quantity);
    }
}

static void test_request_shapes(void)
{
    static const uint8_t bits_max[] = { 0x01U, 0x00U, 0x00U, 0x07U, 0xD0U };
    static const uint8_t bits_over[] = { 0x01U, 0x00U, 0x00U, 0x07U, 0xD1U };
    static const uint8_t regs_max[] = { 0x03U, 0x00U, 0x00U, 0x00U, 0x7DU };
    static const uint8_t regs_over[] = { 0x03U, 0x00U, 0x00U, 0x00U, 0x7EU };
    static const uint8_t regs_zero[] = { 0x03U, 0x00U, 0x00U, 0x00U, 0x00U };
    static const uint8_t coil_on[] = { 0x05U, 0x00U, 0x03U, 0xFFU, 0x00U };
    static const uint8_t coil_bad[] = { 0x05U, 0x00U, 0x03U, 0x12U, 0x34U };
    static const uint8_t reg_one[] = { 0x06U, 0x00U, 0x02U, 0xBEU, 0xEFU };
    static const uint8_t coils_many[] = { 0x0FU, 0x00U, 0x03U, 0x00U, 0x0AU,
                                          0x02U, 0xFFU, 0x03U };
    static const uint8_t coils_bad_bc[] = { 0x0FU, 0x00U, 0x03U, 0x00U, 0x0AU,
                                            0x03U, 0xFFU, 0x03U, 0x00U };
    static const uint8_t regs_many[] = { 0x10U, 0x00U, 0x00U, 0x00U, 0x02U,
                                         0x04U, 0xBEU, 0xEFU, 0x01U, 0x02U };
    static const uint8_t regs_bad_bc[] = { 0x10U, 0x00U, 0x00U, 0x00U, 0x02U,
                                           0x02U, 0xBEU, 0xEFU };
    static const uint8_t report[] = { 0x11U };
    static const uint8_t report_long[] = { 0x11U, 0x00U };
    static const uint8_t unsupported[] = { 0x08U, 0x00U, 0x00U, 0x12U, 0x34U };
    static const uint8_t short_pdu[] = { 0x03U, 0x00U, 0x00U };

    setup();

    check_shape(bits_max, 5U, MODBUS_SLAVE_REQ_READ_BITS, 0U, 2000U,
                MODBUS_SLAVE_EX_NONE);
    check_shape(bits_over, 5U, 0U, 0U, 0U,
                MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
    check_shape(regs_max, 5U, MODBUS_SLAVE_REQ_READ_REGS, 0U, 125U,
                MODBUS_SLAVE_EX_NONE);
    check_shape(regs_over, 5U, 0U, 0U, 0U,
                MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
    check_shape(regs_zero, 5U, 0U, 0U, 0U,
                MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
    check_shape(coil_on, 5U, MODBUS_SLAVE_REQ_WRITE_SINGLE_COIL, 3U, 1U,
                MODBUS_SLAVE_EX_NONE);
    check_shape(coil_bad, 5U, 0U, 0U, 0U,
                MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
    check_shape(reg_one, 5U, MODBUS_SLAVE_REQ_WRITE_SINGLE_REG, 2U, 1U,
                MODBUS_SLAVE_EX_NONE);
    check_shape(coils_many, 8U, MODBUS_SLAVE_REQ_WRITE_BITS, 3U, 10U,
                MODBUS_SLAVE_EX_NONE);
    check_shape(coils_bad_bc, 9U, 0U, 0U, 0U,
                MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
    check_shape(regs_many, 10U, MODBUS_SLAVE_REQ_WRITE_REGS, 0U, 2U,
                MODBUS_SLAVE_EX_NONE);
    check_shape(regs_bad_bc, 8U, 0U, 0U, 0U,
                MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
    check_shape(report, 1U, MODBUS_SLAVE_REQ_REPORT_ID, 0U, 0U,
                MODBUS_SLAVE_EX_NONE);
    check_shape(report_long, 2U, 0U, 0U, 0U,
                MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
    check_shape(unsupported, 5U, 0U, 0U, 0U,
                MODBUS_SLAVE_EX_ILLEGAL_FUNCTION);
    check_shape(short_pdu, 3U, 0U, 0U, 0U,
                MODBUS_SLAVE_EX_ILLEGAL_DATA_VALUE);
}

static void test_nested_critical(void)
{
    setup();
    CHECK(port.crit_depth == 0U);
    CHECK(port.irq_enabled == 1U);

    /* The application holds its own section across a core call: the core's
     * exit must not re-enable interrupts. */
    fake_enter(&port);
    CHECK(port.irq_enabled == 0U);
    CHECK(modbus_slave_start(&slave) == MODBUS_SLAVE_OK);
    CHECK(port.crit_depth == 1U);
    CHECK(port.irq_enabled == 0U);
    fake_exit(&port);
    CHECK(port.crit_depth == 0U);
    CHECK(port.irq_enabled == 1U);

    /* stop() balances its own pair. */
    CHECK(modbus_slave_stop(&slave) == MODBUS_SLAVE_OK);
    CHECK(port.crit_depth == 0U);
    CHECK(port.irq_enabled == 1U);
    CHECK(modbus_slave_start(&slave) == MODBUS_SLAVE_OK);
}

int main(void)
{
    test_start();
    test_read_holding();
    test_read_coils();
    test_read_discrete();
    test_read_input();
    test_write_single_coil();
    test_write_single_register();
    test_write_multiple_coils();
    test_write_multiple_coils_offset();
    test_write_multiple_registers();
    test_report_slave_id();
    test_broadcast_write();
    test_bad_crc();
    test_foreign_address();
    test_illegal_function();
    test_bad_quantity();
    test_out_of_range();
    test_tx_timeout();
    test_address_callback();
    test_start_mid_tx();
    test_nested_critical();
    test_request_shapes();

    printf("%d checks, %d failures\n", checks, failures);
    return (failures == 0) ? 0 : 1;
}
