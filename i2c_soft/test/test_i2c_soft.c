/* SPDX-License-Identifier: MIT */

#include <stdio.h>
#include <string.h>

#include "i2c_soft.h"

/** @brief Address the fake slave answers to. */
#define SLAVE_ADDR 0x50U
/** @brief One library half period, as the fake port sees it. */
#define HALF_US    I2C_SOFT_HALF_PERIOD_US

static int checks;
static int failures;

/** @brief Count one check and print a failure at its source line. */
#define CHECK(cond)                                                          \
    do {                                                                     \
        checks++;                                                            \
        if ( !(cond) ) {                                                     \
            failures++;                                                      \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                    \
    } while ( 0 )

/*----------------------------------------------------------------------------*/
/* Event log                                                                  */
/*----------------------------------------------------------------------------*/

/** @brief Event kinds recorded in ev_log. */
enum {
    EV_SDA_W = 0,
    EV_SCL_W,
    EV_SDA_R,
    EV_SCL_R,
    EV_DELAY
};

/** @brief Capacity of the event log. */
#define EV_MAX 4096U

/** @brief One recorded port operation. */
typedef struct {
    uint8_t  kind;  /**< EV_* kind. */
    uint32_t value; /**< Written level, sampled level or delay. */
} ev_t;

static ev_t   ev_log[EV_MAX];
static size_t ev_len;
static size_t ev_pos;
static int    ev_overflow; /* never reset: reported by main() */

static void record(uint8_t kind, uint32_t value)
{
    if ( ev_len >= (size_t)EV_MAX ) {
        ev_overflow++;
        return;
    }
    ev_log[ev_len].kind  = kind;
    ev_log[ev_len].value = value;
    ev_len++;
}

/*----------------------------------------------------------------------------*/
/* Fake port and fake slave                                                   */
/*----------------------------------------------------------------------------*/

/** @brief Where the fake slave is inside one addressed transfer. */
typedef enum {
    SL_IDLE = 0, /**< No START seen, or a STOP closed the transfer. */
    SL_ADDR,     /**< Shifting the address byte in. */
    SL_WR,       /**< Shifting written bytes in, acknowledge slot per byte. */
    SL_RD,       /**< Driving read bytes out, sampling the master's ACK. */
    SL_DONE      /**< Address NACKed: ignore the bus until STOP or START. */
} slave_phase_t;

/** @brief Fake bus, slave model and critical-section accounting. */
typedef struct {
    /* host-driven and resolved line levels */
    uint8_t host_sda;
    uint8_t host_scl;
    uint8_t sda_line;
    uint8_t scl_line;

    /* fault injection */
    uint8_t  sda_stuck;       /**< Slave holds SDA low regardless of host. */
    uint8_t  scl_stuck;       /**< SCL never rises. */
    uint32_t release_after;   /**< SCL edges until SDA releases; 0 = never. */
    uint32_t scl_stuck_after; /**< SCL rises before it sticks low; 0 = never. */
    uint8_t  nack_addr;       /**< Refuse to acknowledge the address. */
    uint8_t  nack_read_addr;  /**< Refuse an address with R/W = 1. */
    int      nack_wr;         /**< 0-based write byte to NACK; -1 = none. */
    uint8_t  slave_addr;      /**< Address the fake answers to. */
    uint8_t  rd_data[16];
    uint8_t  rd_len;
    uint8_t  rd_pos;

    /* protocol model */
    slave_phase_t phase;
    uint8_t       shift;
    uint8_t       bit_count;
    uint8_t       in_ack;
    uint8_t       sda_low; /**< Slave pulls SDA low. */
    uint8_t       rw;      /**< R/W bit of the address in flight. */
    uint8_t       rx_bytes[32];
    uint8_t       rx_count;
    uint8_t       addr_byte;
    uint8_t       addr_count;
    uint8_t       macks[32]; /**< The master's ACK (0) per read byte. */
    uint8_t       mack_count;
    int           starts;
    int           restarts;
    int           stops;
    uint32_t      scl_rises;

    /* critical-section accounting */
    uint8_t  irq_enabled;
    uint32_t crit_depth;
    uint32_t crit_count;
    uint32_t crit_underflow;
    uint32_t op_count;
    uint32_t op_min_depth;
    uint32_t op_max_depth;
    uint32_t delay_bad;
    uint32_t delay_d0;
    uint32_t delay_d1;
    uint32_t delay_dother;
} fake_t;

static fake_t     bus;
static i2c_soft_t dev;

static void resolve_levels(fake_t* f)
{
    uint8_t sda_low;
    uint8_t scl_low;

    sda_low = (uint8_t)((f->host_sda == 0U) || (f->sda_low != 0U)
                        || (f->sda_stuck != 0U));
    scl_low = (uint8_t)((f->host_scl == 0U) || (f->scl_stuck != 0U));
    f->sda_line = (uint8_t)((sda_low != 0U) ? 0U : 1U);
    f->scl_line = (uint8_t)((scl_low != 0U) ? 0U : 1U);
}

static void note_op(fake_t* f)
{
    f->op_count++;
    if ( f->crit_depth < f->op_min_depth ) {
        f->op_min_depth = f->crit_depth;
    }
    if ( f->crit_depth > f->op_max_depth ) {
        f->op_max_depth = f->crit_depth;
    }
}

/** @brief The level the master is holding on SDA at this clock edge. */
static uint8_t bit_in(const fake_t* f)
{
    return (uint8_t)((f->sda_line != 0U) ? 1U : 0U);
}

static uint8_t slave_next_read_byte(fake_t* f)
{
    uint8_t byte = 0xFFU; /* a released line reads as all ones */

    if ( f->rd_pos < f->rd_len ) {
        byte = f->rd_data[f->rd_pos];
        f->rd_pos++;
    }
    return byte;
}

static void slave_drive_read_bit(fake_t* f)
{
    if ( f->phase == SL_RD && f->bit_count < 8U ) {
        f->sda_low = (uint8_t)( ((f->shift >> (7U - f->bit_count)) & 1U)
                                == 0U );
    } else {
        f->sda_low = 0U;
    }
}

static void slave_on_start(fake_t* f)
{
    f->phase     = SL_ADDR;
    f->shift     = 0U;
    f->bit_count = 0U;
    f->in_ack    = 0U;
    f->rw        = 0U;
    f->sda_low   = 0U;
}

static void slave_on_stop(fake_t* f)
{
    f->phase     = SL_IDLE;
    f->shift     = 0U;
    f->bit_count = 0U;
    f->in_ack    = 0U;
    f->rw        = 0U;
    f->sda_low   = 0U;
}

static void slave_rise(fake_t* f)
{
    switch ( f->phase ) {
        case SL_ADDR:
            if ( f->in_ack == 0U ) {
                f->shift = (uint8_t)((f->shift << 1) | bit_in(f));
                f->bit_count++;
            }
            break;

        case SL_WR:
            if ( f->in_ack != 0U ) {
                break;
            }
            f->shift = (uint8_t)((f->shift << 1) | bit_in(f));
            f->bit_count++;
            if ( f->bit_count == 8U
                 && f->rx_count < (uint8_t)sizeof f->rx_bytes ) {
                f->rx_bytes[f->rx_count] = f->shift;
                f->rx_count++;
            }
            break;

        case SL_RD:
            if ( f->in_ack != 0U
                 && f->mack_count < (uint8_t)sizeof f->macks ) {
                f->macks[f->mack_count] = bit_in(f);
                f->mack_count++;
            }
            break;

        case SL_IDLE:
        case SL_DONE:
        default:
            break;
    }
}

static void slave_fall(fake_t* f)
{
    switch ( f->phase ) {
        case SL_ADDR:
            if ( f->in_ack != 0U ) {
                /* Acknowledge slot over: follow the R/W bit. */
                f->in_ack    = 0U;
                f->bit_count = 0U;
                if ( (f->rw & 1U) != 0U ) {
                    f->phase     = SL_RD;
                    f->shift     = slave_next_read_byte(f);
                    f->bit_count = 0U;
                    slave_drive_read_bit(f);
                } else {
                    f->phase   = SL_WR;
                    f->sda_low = 0U;
                }
            } else if ( f->bit_count == 8U ) {
                f->addr_byte = f->shift;
                f->addr_count++;
                f->rw        = (uint8_t)(f->shift & 1U);
                if ( (f->shift >> 1) == f->slave_addr
                     && f->nack_addr == 0U
                     && (f->nack_read_addr == 0U
                         || (f->shift & 1U) == 0U) ) {
                    f->in_ack  = 1U;
                    f->sda_low = 1U; /* acknowledge while SCL is low */
                } else {
                    f->phase   = SL_DONE;
                    f->sda_low = 0U;
                }
            }
            break;

        case SL_WR:
            if ( f->in_ack != 0U ) {
                f->in_ack    = 0U;
                f->bit_count = 0U;
                f->sda_low   = 0U; /* release for the next byte or STOP */
            } else if ( f->bit_count == 8U ) {
                f->in_ack = 1U;
                if ( f->nack_wr >= 0
                     && (int)(f->rx_count - 1U) == f->nack_wr ) {
                    f->sda_low = 0U; /* NACK: leave the line released */
                } else {
                    f->sda_low = 1U;
                }
            }
            break;

        case SL_RD:
            if ( f->in_ack != 0U ) {
                f->in_ack    = 0U;
                f->bit_count = 0U;
                if ( f->mack_count > 0U
                     && f->macks[f->mack_count - 1U] == 0U ) {
                    f->shift = slave_next_read_byte(f);
                    slave_drive_read_bit(f);
                } else {
                    f->sda_low = 0U; /* master NACKed: stay released */
                }
            } else if ( f->bit_count >= 7U ) {
                /* The eighth bit is out: open the slot for the master. */
                f->bit_count = 8U;
                f->in_ack    = 1U;
                f->sda_low   = 0U;
            } else {
                f->bit_count++;
                slave_drive_read_bit(f);
            }
            break;

        case SL_IDLE:
        case SL_DONE:
        default:
            break;
    }
}

/*----------------------------------------------------------------------------*/
/* Fake port operations                                                       */
/*----------------------------------------------------------------------------*/

static void fake_sda_write(void* ctx, uint8_t level)
{
    fake_t*  f = (fake_t*)ctx;
    uint8_t previous;

    note_op(f);
    previous    = f->host_sda;
    f->host_sda = (uint8_t)((level != 0U) ? 1U : 0U);

    if ( f->host_sda != previous ) {
        if ( f->scl_line != 0U ) { /* a condition edge needs SCL high */
            if ( f->host_sda == 0U ) {
                f->starts++;
                if ( f->phase != SL_IDLE ) {
                    f->restarts++;
                }
                slave_on_start(f);
            } else {
                f->stops++;
                slave_on_stop(f);
            }
        }
    }

    resolve_levels(f);
    record(EV_SDA_W, f->host_sda);
}

static void fake_scl_write(void* ctx, uint8_t level)
{
    fake_t*  f = (fake_t*)ctx;
    uint8_t previous;

    note_op(f);
    previous    = f->scl_line;
    f->host_scl = (uint8_t)((level != 0U) ? 1U : 0U);
    resolve_levels(f);

    if ( f->scl_line != previous ) {
        if ( f->scl_line != 0U ) {
            f->scl_rises++;
            if ( f->sda_stuck != 0U && f->release_after != 0U
                 && f->scl_rises >= f->release_after ) {
                f->sda_stuck = 0U;
                resolve_levels(f);
            }
            slave_rise(f);
            if ( f->scl_stuck_after != 0U
                 && f->scl_rises >= f->scl_stuck_after ) {
                f->scl_stuck = 1U;
                resolve_levels(f);
            }
        } else {
            slave_fall(f);
        }
    }

    record(EV_SCL_W, f->host_scl);
}

static uint8_t fake_sda_read(void* ctx)
{
    fake_t* f = (fake_t*)ctx;

    note_op(f);
    resolve_levels(f);
    record(EV_SDA_R, f->sda_line);
    return f->sda_line;
}

static uint8_t fake_scl_read(void* ctx)
{
    fake_t* f = (fake_t*)ctx;

    note_op(f);
    resolve_levels(f);
    record(EV_SCL_R, f->scl_line);
    return f->scl_line;
}

static void fake_delay(void* ctx, uint32_t us)
{
    fake_t* f = (fake_t*)ctx;

    note_op(f);
    if ( us != HALF_US ) {
        f->delay_bad++;
    }
    if ( f->crit_depth == 0U ) {
        f->delay_d0++;
    } else if ( f->crit_depth == 1U ) {
        f->delay_d1++;
    } else {
        f->delay_dother++;
    }
    record(EV_DELAY, us);
}

static void fake_enter(void* ctx)
{
    fake_t* f = (fake_t*)ctx;

    f->crit_count++;
    f->irq_enabled = 0U;
    f->crit_depth++;
}

static void fake_exit(void* ctx)
{
    fake_t* f = (fake_t*)ctx;

    if ( f->crit_depth == 0U ) {
        f->crit_underflow++;
        return;
    }
    f->crit_depth--;
    if ( f->crit_depth == 0U ) {
        f->irq_enabled = 1U;
    }
}

/*----------------------------------------------------------------------------*/
/* Test harness                                                               */
/*----------------------------------------------------------------------------*/

static void fake_reset(void)
{
    memset(&bus, 0, sizeof bus);

    bus.host_sda      = 1U;
    bus.host_scl      = 1U;
    bus.slave_addr    = SLAVE_ADDR;
    bus.nack_wr       = -1;
    bus.release_after = 0U;
    bus.phase         = SL_IDLE;
    bus.irq_enabled   = 1U;
    bus.op_min_depth  = 0xFFFFFFFFU;

    resolve_levels(&bus);

    ev_len = 0U;
    ev_pos = 0U;
}

static i2c_soft_config_t make_config(void)
{
    i2c_soft_config_t cfg;

    memset(&cfg, 0, sizeof cfg);

    cfg.port.ctx       = &bus;
    cfg.port.sda_write = fake_sda_write;
    cfg.port.scl_write = fake_scl_write;
    cfg.port.sda_read  = fake_sda_read;
    cfg.port.scl_read  = fake_scl_read;
    cfg.port.delay_us  = fake_delay;

    cfg.critical.ctx   = &bus;
    cfg.critical.enter = fake_enter;
    cfg.critical.exit  = fake_exit;

    return cfg;
}

/** @brief Fresh bus, constructed and initialized; the log starts at zero. */
static void setup(void)
{
    i2c_soft_config_t cfg;

    fake_reset();
    cfg = make_config();

    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(i2c_soft_init(&dev) == I2C_SOFT_OK);

    ev_len = 0U;
    ev_pos = 0U;
}

/*----------------------------------------------------------------------------*/
/* Event expectations                                                         */
/*----------------------------------------------------------------------------*/

static const char* ev_name(uint8_t kind)
{
    switch ( kind ) {
        case EV_SDA_W:
            return "sda_w";
        case EV_SCL_W:
            return "scl_w";
        case EV_SDA_R:
            return "sda_r";
        case EV_SCL_R:
            return "scl_r";
        case EV_DELAY:
            return "delay";
        default:
            return "?";
    }
}

static void expect_ev_at(int line, uint8_t kind, uint32_t value)
{
    checks++;
    if ( ev_pos >= ev_len ) {
        failures++;
        printf("FAIL %s:%d: expected %s(%u), the log ended\n", __FILE__, line,
               ev_name(kind), (unsigned)value);
        return;
    }
    if ( ev_log[ev_pos].kind != kind || ev_log[ev_pos].value != value ) {
        failures++;
        printf("FAIL %s:%d: expected %s(%u), got %s(%u) at event %u\n",
               __FILE__, line, ev_name(kind), (unsigned)value,
               ev_name(ev_log[ev_pos].kind),
               (unsigned)ev_log[ev_pos].value, (unsigned)ev_pos);
    }
    ev_pos++;
}

/** @brief Expect an event at the current source line. */
#define EXPECT(kind, value) expect_ev_at(__LINE__, (kind), (value))

static void expect_end(void)
{
    checks++;
    if ( ev_pos < ev_len ) {
        failures++;
        printf("FAIL %s:%d: %u event(s) left unexamined\n", __FILE__, __LINE__,
               (unsigned)(ev_len - ev_pos));
        while ( ev_pos < ev_len ) {
            printf("    leftover %s(%u)\n", ev_name(ev_log[ev_pos].kind),
                   (unsigned)ev_log[ev_pos].value);
            ev_pos++;
        }
    } else if ( ev_pos > ev_len ) {
        failures++;
        printf("FAIL %s:%d: the walk ran %u event(s) past the log\n",
               __FILE__, __LINE__, (unsigned)(ev_pos - ev_len));
        ev_pos = ev_len;
    }
}

/** @brief init()/bus_recover() release both lines and wait one half period. */
static void expect_release(uint8_t scl, uint8_t sda)
{
    EXPECT(EV_SDA_W, 1U);
    EXPECT(EV_SCL_W, 1U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SCL_R, scl);
    EXPECT(EV_SDA_R, sda);
}

/** @brief init() re-reads both lines after the release (or the recovery). */
static void expect_recheck(uint8_t scl, uint8_t sda)
{
    EXPECT(EV_SCL_R, scl);
    EXPECT(EV_SDA_R, sda);
}

static void expect_start(void)
{
    EXPECT(EV_SDA_W, 0U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SCL_W, 0U);
    EXPECT(EV_DELAY, HALF_US);
}

static void expect_restart(void)
{
    EXPECT(EV_SDA_W, 1U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SCL_W, 1U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SDA_W, 0U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SCL_W, 0U);
    EXPECT(EV_DELAY, HALF_US);
}

static void expect_stop(void)
{
    EXPECT(EV_SCL_W, 0U);
    EXPECT(EV_SDA_W, 0U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SCL_W, 1U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SDA_W, 1U);
    EXPECT(EV_DELAY, HALF_US);
}

static void expect_write_bit(uint8_t bit)
{
    EXPECT(EV_SDA_W, bit);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SCL_W, 1U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SCL_W, 0U);
}

/** @brief The ninth clock of a written byte: released line, then the sample. */
static void expect_ack_slot(uint8_t sampled)
{
    EXPECT(EV_SDA_W, 1U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SCL_W, 1U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SDA_R, sampled);
    EXPECT(EV_SCL_W, 0U);
}

static void expect_write_byte(uint8_t byte, uint8_t ack)
{
    int i;

    for ( i = 7; i >= 0; --i ) {
        expect_write_bit((uint8_t)((byte >> i) & 1U));
    }
    expect_ack_slot(ack);
}

/** @brief One clock of a byte the slave shifts out: master_ack drives SDA. */
static void expect_read_bit(uint8_t bit)
{
    EXPECT(EV_SDA_W, 1U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SCL_W, 1U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SDA_R, bit);
    EXPECT(EV_SCL_W, 0U);
}

static void expect_read_byte(uint8_t byte, uint8_t master_ack)
{
    int i;

    for ( i = 7; i >= 0; --i ) {
        expect_read_bit((uint8_t)((byte >> i) & 1U));
    }
    expect_write_bit(master_ack);
}

/** @brief One recovery clock: low, high, then the SDA sample. */
static void expect_pulse(uint8_t sda)
{
    EXPECT(EV_SCL_W, 0U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SCL_W, 1U);
    EXPECT(EV_DELAY, HALF_US);
    EXPECT(EV_SDA_R, sda);
}

/*----------------------------------------------------------------------------*/
/* 1. init on an idle bus                                                     */
/*----------------------------------------------------------------------------*/

static void test_init_idle(void)
{
    i2c_soft_config_t cfg;

    fake_reset();
    cfg = make_config();

    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(i2c_soft_init(&dev) == I2C_SOFT_OK);
    expect_release(1U, 1U);
    expect_recheck(1U, 1U);

    /* init may be called again: the same idle bus answers the same way. */
    CHECK(i2c_soft_init(&dev) == I2C_SOFT_OK);
    expect_release(1U, 1U);
    expect_recheck(1U, 1U);
    expect_end();

    CHECK(bus.sda_line == 1U);
    CHECK(bus.scl_line == 1U);
    CHECK(bus.scl_rises == 0U);
    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
    CHECK(bus.op_min_depth == 1U);
    CHECK(bus.op_max_depth == 1U);
}

/*----------------------------------------------------------------------------*/
/* 2. init recovers an SDA held low                                           */
/*----------------------------------------------------------------------------*/

static void test_init_recovers_sda(void)
{
    i2c_soft_config_t cfg;

    fake_reset();
    bus.sda_stuck     = 1U;
    bus.release_after = 3U;
    resolve_levels(&bus);
    cfg = make_config();

    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(i2c_soft_init(&dev) == I2C_SOFT_OK);

    expect_release(1U, 0U); /* init sees SDA low */
    expect_release(1U, 0U); /* recovery checks it too */
    expect_pulse(0U);
    expect_pulse(0U);
    expect_pulse(1U);       /* third clock: the slave lets go */
    expect_stop();
    expect_recheck(1U, 1U); /* recovery confirms both lines */
    expect_recheck(1U, 1U); /* init confirms them once more */
    expect_end();

    CHECK(bus.sda_line == 1U);
    CHECK(bus.scl_line == 1U);
    CHECK(bus.stops == 1);
    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
}

/*----------------------------------------------------------------------------*/
/* 3. init with an SDA that never lets go                                     */
/*----------------------------------------------------------------------------*/

static void test_init_sda_never_releases(void)
{
    i2c_soft_config_t cfg;
    int               i;

    fake_reset();
    bus.sda_stuck = 1U;
    resolve_levels(&bus);
    cfg = make_config();

    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(i2c_soft_init(&dev) == I2C_SOFT_ERR_BUS);

    expect_release(1U, 0U);
    expect_release(1U, 0U);
    for ( i = 0; i < 9; ++i ) {
        expect_pulse(0U);
    }
    expect_stop();
    expect_recheck(1U, 0U); /* still held: recovery gives up */
    expect_end();

    CHECK(bus.sda_line == 0U);
    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
}

/*----------------------------------------------------------------------------*/
/* 4. init with SCL held low                                                  */
/*----------------------------------------------------------------------------*/

static void test_init_scl_low(void)
{
    i2c_soft_config_t cfg;

    fake_reset();
    bus.scl_stuck = 1U;
    resolve_levels(&bus);
    cfg = make_config();

    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(i2c_soft_init(&dev) == I2C_SOFT_ERR_BUS);

    expect_release(0U, 1U);
    expect_end();

    CHECK(bus.scl_rises == 0U); /* not a single pulse */
    CHECK(bus.crit_depth == 0U);
}

/*----------------------------------------------------------------------------*/
/* 5. write one byte, waveform end to end                                     */
/*----------------------------------------------------------------------------*/

static void test_write_single(void)
{
    static const uint8_t data[1] = { 0xA5U };

    setup();
    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, data, sizeof data)
          == I2C_SOFT_OK);

    expect_start();
    expect_write_byte((uint8_t)(SLAVE_ADDR << 1), 0U); /* address+W, ACK */
    expect_write_byte(0xA5U, 0U);                      /* data, ACK */
    expect_stop();
    expect_end();

    CHECK(bus.addr_count == 1);
    CHECK(bus.addr_byte == (uint8_t)(SLAVE_ADDR << 1));
    CHECK(bus.rx_count == 1);
    CHECK(bus.rx_bytes[0] == 0xA5U);
    CHECK(bus.starts == 1);
    CHECK(bus.restarts == 0);
    CHECK(bus.stops == 1);
    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
    CHECK(bus.crit_underflow == 0U);
}

/*----------------------------------------------------------------------------*/
/* 6. write several bytes                                                     */
/*----------------------------------------------------------------------------*/

static void test_write_multi(void)
{
    static const uint8_t data[3] = { 0x12U, 0x34U, 0x56U };

    setup();
    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, data, sizeof data)
          == I2C_SOFT_OK);

    expect_start();
    expect_write_byte((uint8_t)(SLAVE_ADDR << 1), 0U);
    expect_write_byte(0x12U, 0U);
    expect_write_byte(0x34U, 0U);
    expect_write_byte(0x56U, 0U);
    expect_stop();
    expect_end();

    CHECK(bus.rx_count == 3);
    CHECK(memcmp(bus.rx_bytes, data, sizeof data) == 0);
    CHECK(bus.stops == 1);
    CHECK(bus.crit_depth == 0U);
}

/*----------------------------------------------------------------------------*/
/* 7. read several bytes: ACK on all but the last                             */
/*----------------------------------------------------------------------------*/

static void test_read_multi(void)
{
    static const uint8_t wanted[3] = { 0x12U, 0x34U, 0x56U };
    uint8_t             got[3];

    setup();
    bus.rd_data[0] = 0x12U;
    bus.rd_data[1] = 0x34U;
    bus.rd_data[2] = 0x56U;
    bus.rd_len     = 3U;

    CHECK(i2c_soft_read(&dev, SLAVE_ADDR, got, sizeof got) == I2C_SOFT_OK);

    expect_start();
    expect_write_byte((uint8_t)((SLAVE_ADDR << 1) | 1U), 0U); /* address+R */
    expect_read_byte(0x12U, 0U);                              /* ACK */
    expect_read_byte(0x34U, 0U);                              /* ACK */
    expect_read_byte(0x56U, 1U);                              /* NACK */
    expect_stop();
    expect_end();

    CHECK(memcmp(got, wanted, sizeof got) == 0);
    CHECK(bus.mack_count == 3);
    CHECK(bus.macks[0] == 0U);
    CHECK(bus.macks[1] == 0U);
    CHECK(bus.macks[2] == 1U);
    CHECK(bus.rd_pos == 3U);
    CHECK(bus.stops == 1);
    CHECK(bus.crit_depth == 0U);
}

/*----------------------------------------------------------------------------*/
/* 8. write_read: repeated START in the middle                                */
/*----------------------------------------------------------------------------*/

static void test_write_read(void)
{
    static const uint8_t wr[1]    = { 0x04U };
    static const uint8_t wanted[2] = { 0xABU, 0xCDU };
    uint8_t             rd[2];

    setup();
    bus.rd_data[0] = 0xABU;
    bus.rd_data[1] = 0xCDU;
    bus.rd_len     = 2U;

    CHECK(i2c_soft_write_read(&dev, SLAVE_ADDR, wr, sizeof wr, rd, sizeof rd)
          == I2C_SOFT_OK);

    expect_start();
    expect_write_byte((uint8_t)(SLAVE_ADDR << 1), 0U);
    expect_write_byte(0x04U, 0U);
    expect_restart();
    expect_write_byte((uint8_t)((SLAVE_ADDR << 1) | 1U), 0U);
    expect_read_byte(0xABU, 0U);
    expect_read_byte(0xCDU, 1U);
    expect_stop();
    expect_end();

    CHECK(memcmp(rd, wanted, sizeof rd) == 0);
    CHECK(bus.starts == 2);
    CHECK(bus.restarts == 1);
    CHECK(bus.stops == 1);
    CHECK(bus.rx_count == 1);
    CHECK(bus.mack_count == 2);
    CHECK(bus.crit_depth == 0U);
}

/* A NACK of the read address ends the transfer after the repeated START. */
static void test_write_read_read_addr_nack(void)
{
    static const uint8_t wr[1] = { 0x04U };
    uint8_t             rd[2] = { 0xAAU, 0xBBU };

    setup();
    bus.nack_read_addr = 1U;

    CHECK(i2c_soft_write_read(&dev, SLAVE_ADDR, wr, sizeof wr, rd, sizeof rd)
          == I2C_SOFT_ERR_NACK);

    expect_start();
    expect_write_byte((uint8_t)(SLAVE_ADDR << 1), 0U);
    expect_write_byte(0x04U, 0U);
    expect_restart();
    expect_write_byte((uint8_t)((SLAVE_ADDR << 1) | 1U), 1U);
    expect_stop();
    expect_end();

    CHECK(rd[0] == 0xAAU); /* the read phase never ran */
    CHECK(rd[1] == 0xBBU);
    CHECK(bus.restarts == 1);
    CHECK(bus.rd_pos == 0U);
    CHECK(bus.mack_count == 0U);
    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
}

/*----------------------------------------------------------------------------*/
/* 9. zero lengths never touch the bus                                        */
/*----------------------------------------------------------------------------*/

static void test_zero_length(void)
{
    static const uint8_t wr[1] = { 0x01U };
    uint8_t             rd[1];
    uint32_t            op_count;
    uint32_t            crit_count;

    setup();
    op_count   = bus.op_count;
    crit_count = bus.crit_count;

    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, wr, 0U) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_read(&dev, SLAVE_ADDR, rd, 0U) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_write_read(&dev, SLAVE_ADDR, wr, 0U, rd, sizeof rd)
          == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_write_read(&dev, SLAVE_ADDR, wr, sizeof wr, rd, 0U)
          == I2C_SOFT_ERR_ARG);

    CHECK(bus.op_count == op_count);
    CHECK(bus.crit_count == crit_count);
    CHECK(ev_len == 0U);
}

/*----------------------------------------------------------------------------*/
/* 10. probe answers with ACK or NACK, always with a STOP                     */
/*----------------------------------------------------------------------------*/

static void test_probe(void)
{
    setup();
    CHECK(i2c_soft_probe(&dev, SLAVE_ADDR) == I2C_SOFT_OK);

    expect_start();
    expect_write_byte((uint8_t)(SLAVE_ADDR << 1), 0U);
    expect_stop();
    expect_end();
    CHECK(bus.stops == 1);
    CHECK(bus.addr_count == 1);

    setup();
    bus.nack_addr = 1U;
    CHECK(i2c_soft_probe(&dev, SLAVE_ADDR) == I2C_SOFT_ERR_NACK);

    expect_start();
    expect_write_byte((uint8_t)(SLAVE_ADDR << 1), 1U);
    expect_stop();
    expect_end();
    CHECK(bus.stops == 1);
    CHECK(bus.crit_depth == 0U);
}

/*----------------------------------------------------------------------------*/
/* 11. a NACKed address stops the transfer before the data phase              */
/*----------------------------------------------------------------------------*/

static void test_address_nack(void)
{
    static const uint8_t wr[2] = { 0x11U, 0x22U };
    uint8_t             rd[2] = { 0xAAU, 0xBBU };

    setup();
    bus.nack_addr = 1U;

    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, wr, sizeof wr)
          == I2C_SOFT_ERR_NACK);

    expect_start();
    expect_write_byte((uint8_t)(SLAVE_ADDR << 1), 1U);
    expect_stop();
    expect_end();

    CHECK(bus.addr_count == 1);
    CHECK(bus.rx_count == 0); /* the data phase never started */
    CHECK(bus.stops == 1);
    CHECK(bus.restarts == 0);

    setup();
    bus.nack_addr = 1U;

    CHECK(i2c_soft_read(&dev, SLAVE_ADDR, rd, sizeof rd)
          == I2C_SOFT_ERR_NACK);

    expect_start();
    expect_write_byte((uint8_t)((SLAVE_ADDR << 1) | 1U), 1U);
    expect_stop();
    expect_end();

    CHECK(bus.rd_pos == 0);
    CHECK(bus.mack_count == 0);
    CHECK(bus.stops == 1);
    CHECK(bus.crit_depth == 0U);
    CHECK(rd[0] == 0xAAU); /* a NACKed read leaves the buffer alone */
    CHECK(rd[1] == 0xBBU);
}

/*----------------------------------------------------------------------------*/
/* 12. a NACKed write byte stops the transfer                                 */
/*----------------------------------------------------------------------------*/

static void test_write_nack(void)
{
    static const uint8_t wr[3] = { 0x11U, 0x22U, 0x33U };
    uint8_t             rd[1] = { 0x5AU };

    setup();
    bus.nack_wr = 1;

    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, wr, sizeof wr)
          == I2C_SOFT_ERR_NACK);

    expect_start();
    expect_write_byte((uint8_t)(SLAVE_ADDR << 1), 0U);
    expect_write_byte(0x11U, 0U);
    expect_write_byte(0x22U, 1U); /* second byte NACKed */
    expect_stop();
    expect_end();

    CHECK(bus.rx_count == 2); /* the third byte never left */
    CHECK(bus.stops == 1);
    CHECK(bus.restarts == 0);

    /* the same NACK ends write_read before its repeated START */
    setup();
    bus.nack_wr = 0;

    CHECK(i2c_soft_write_read(&dev, SLAVE_ADDR, wr, 2U, rd, sizeof rd)
          == I2C_SOFT_ERR_NACK);

    expect_start();
    expect_write_byte((uint8_t)(SLAVE_ADDR << 1), 0U);
    expect_write_byte(0x11U, 1U);
    expect_stop();
    expect_end();

    CHECK(bus.restarts == 0);
    CHECK(bus.stops == 1);
    CHECK(bus.rd_pos == 0);
    CHECK(bus.mack_count == 0);
    CHECK(rd[0] == 0x5AU); /* the read phase never touched the buffer */
    CHECK(bus.crit_depth == 0U);
}

/*----------------------------------------------------------------------------*/
/* 13. reserved addresses are rejected without a bus action                   */
/*----------------------------------------------------------------------------*/

static void test_reserved_addresses(void)
{
    static const uint8_t wr[1] = { 0x01U };
    uint8_t             rd[1];
    uint32_t            op_count;
    uint32_t            crit_count;

    setup();
    op_count   = bus.op_count;
    crit_count = bus.crit_count;

    CHECK(i2c_soft_write(&dev, 0x00U, wr, sizeof wr) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_write(&dev, 0x78U, wr, sizeof wr) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_write(&dev, 0x7FU, wr, sizeof wr) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_write(&dev, 0x80U, wr, sizeof wr) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_read(&dev, 0x00U, rd, sizeof rd) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_read(&dev, 0x78U, rd, sizeof rd) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_read(&dev, 0x7FU, rd, sizeof rd) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_read(&dev, 0x80U, rd, sizeof rd) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_probe(&dev, 0x00U) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_probe(&dev, 0x78U) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_probe(&dev, 0x7FU) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_probe(&dev, 0x80U) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_write_read(&dev, 0x78U, wr, sizeof wr, rd, sizeof rd)
          == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_write_read(&dev, 0x7FU, wr, sizeof wr, rd, sizeof rd)
          == I2C_SOFT_ERR_ARG);

    CHECK(bus.op_count == op_count);
    CHECK(bus.crit_count == crit_count);
    CHECK(ev_len == 0U);
}

/* The valid ends of the range reach the bus and are acknowledged. */
static void test_address_limits(void)
{
    static const uint8_t data[1] = { 0x00U };

    setup();
    bus.slave_addr = 0x01U;
    CHECK(i2c_soft_write(&dev, 0x01U, data, sizeof data) == I2C_SOFT_OK);
    CHECK(bus.addr_byte == 0x02U);
    CHECK(bus.addr_count == 1);
    CHECK(bus.stops == 1);

    setup();
    bus.slave_addr = 0x77U;
    CHECK(i2c_soft_write(&dev, 0x77U, data, sizeof data) == I2C_SOFT_OK);
    CHECK(bus.addr_byte == 0xEEU);
    CHECK(bus.addr_count == 1);
    CHECK(bus.stops == 1);
}

/*----------------------------------------------------------------------------*/
/* 14. NULL arguments are rejected without a bus action                       */
/*----------------------------------------------------------------------------*/

static void test_null_arguments(void)
{
    static const uint8_t wr[2] = { 0x01U, 0x02U };
    uint8_t             rd[2];
    uint32_t            op_count;
    uint32_t            crit_count;

    setup();
    op_count   = bus.op_count;
    crit_count = bus.crit_count;

    CHECK(i2c_soft_init(NULL) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_bus_recover(NULL) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_write(NULL, SLAVE_ADDR, wr, 1U) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_read(NULL, SLAVE_ADDR, rd, 1U) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_write_read(NULL, SLAVE_ADDR, wr, 1U, rd, 1U)
          == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_probe(NULL, SLAVE_ADDR) == I2C_SOFT_ERR_ARG);

    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, NULL, 1U) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_read(&dev, SLAVE_ADDR, NULL, 1U) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_write_read(&dev, SLAVE_ADDR, NULL, 1U, rd, 1U)
          == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_write_read(&dev, SLAVE_ADDR, wr, 1U, NULL, 1U)
          == I2C_SOFT_ERR_ARG);

    /* not one port operation, not one critical section */
    CHECK(bus.op_count == op_count);
    CHECK(bus.crit_count == crit_count);
    CHECK(ev_len == 0U);
}

/*----------------------------------------------------------------------------*/
/* 15. the constructor validates every callback                               */
/*----------------------------------------------------------------------------*/

static void test_ctor_validation(void)
{
    i2c_soft_config_t cfg;

    fake_reset();
    cfg = make_config();

    CHECK(i2c_soft_ctor(NULL, &cfg) == I2C_SOFT_ERR_ARG);
    CHECK(i2c_soft_ctor(&dev, NULL) == I2C_SOFT_ERR_ARG);

    cfg.port.sda_write = NULL;
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_ERR_ARG);
    cfg = make_config();

    cfg.port.scl_write = NULL;
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_ERR_ARG);
    cfg = make_config();

    cfg.port.sda_read = NULL;
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_ERR_ARG);
    cfg = make_config();

    cfg.port.scl_read = NULL;
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_ERR_ARG);
    cfg = make_config();

    cfg.port.delay_us = NULL;
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_ERR_ARG);
    cfg = make_config();

    cfg.critical.enter = NULL;
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_ERR_ARG);
    cfg = make_config();

    cfg.critical.exit = NULL;
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_ERR_ARG);

    /* a rejected construction never reached the port */
    CHECK(bus.op_count == 0U);
    CHECK(bus.crit_count == 0U);

    cfg = make_config();
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(bus.op_count == 0U); /* pure RAM construction */
    CHECK(bus.crit_count == 0U);
}

/*----------------------------------------------------------------------------*/
/* 16. every delay is a half period taken inside one critical section         */
/*----------------------------------------------------------------------------*/

static void test_delay_values(void)
{
    static const uint8_t data[1] = { 0xA5U };

    setup();
    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, data, sizeof data)
          == I2C_SOFT_OK);

    CHECK(bus.delay_bad == 0U);
    CHECK(bus.delay_d0 == 0U);
    CHECK(bus.delay_dother == 0U);
    CHECK(bus.delay_d1 > 0U);
    CHECK(bus.op_min_depth == 1U);
    CHECK(bus.op_max_depth == 1U);
    CHECK(bus.crit_depth == 0U);
}

/*----------------------------------------------------------------------------*/
/* 17. critical sections balance on every path and nest with the caller's     */
/*----------------------------------------------------------------------------*/

static void test_critical_balance(void)
{
    static const uint8_t data[1] = { 0xA5U };

    setup();
    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, data, sizeof data)
          == I2C_SOFT_OK);
    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
    CHECK(bus.crit_underflow == 0U);

    setup();
    bus.nack_addr = 1U;
    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, data, sizeof data)
          == I2C_SOFT_ERR_NACK);
    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
    CHECK(bus.crit_underflow == 0U);

    /* the caller holds its own critical section around the transaction */
    setup();
    bus.op_min_depth = 0xFFFFFFFFU;
    bus.op_max_depth = 0U;
    bus.delay_d0     = 0U;
    bus.delay_d1     = 0U;
    bus.delay_dother = 0U;

    fake_enter(&bus);
    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, data, sizeof data)
          == I2C_SOFT_OK);
    CHECK(bus.crit_depth == 1U);
    CHECK(bus.irq_enabled == 0U);
    CHECK(bus.op_min_depth == 2U);
    CHECK(bus.op_max_depth == 2U);
    CHECK(bus.delay_d1 == 0U);
    CHECK(bus.delay_dother > 0U);
    fake_exit(&bus);

    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
    CHECK(bus.crit_underflow == 0U);

    /* ...and when the transfer NACKs. */
    setup();
    bus.op_min_depth = 0xFFFFFFFFU;
    bus.op_max_depth = 0U;
    bus.delay_d0     = 0U;
    bus.delay_d1     = 0U;
    bus.delay_dother = 0U;
    bus.nack_addr    = 1U;

    fake_enter(&bus);
    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, data, sizeof data)
          == I2C_SOFT_ERR_NACK);
    CHECK(bus.crit_depth == 1U);
    CHECK(bus.irq_enabled == 0U);
    CHECK(bus.op_min_depth == 2U);
    CHECK(bus.op_max_depth == 2U);
    CHECK(bus.delay_d1 == 0U);
    CHECK(bus.delay_dother > 0U);
    fake_exit(&bus);

    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
    CHECK(bus.crit_underflow == 0U);
}

/** @brief Every delay ran at critical depth 1 and every pair balanced. */
static void expect_clean_critical(void)
{
    CHECK(bus.delay_bad == 0U);
    CHECK(bus.delay_d0 == 0U);
    CHECK(bus.delay_dother == 0U);
    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
    CHECK(bus.crit_underflow == 0U);
    CHECK(bus.op_min_depth == 1U);
    CHECK(bus.op_max_depth == 1U);
}

/* The per-bit invariant holds on every API, not only on write. */
static void test_all_paths_critical(void)
{
    static const uint8_t wr[1]   = { 0x04U };
    static const uint8_t data[1] = { 0xA5U };
    uint8_t             rd[2];

    setup();
    CHECK(i2c_soft_read(&dev, SLAVE_ADDR, rd, sizeof rd) == I2C_SOFT_OK);
    expect_clean_critical();

    setup();
    CHECK(i2c_soft_write_read(&dev, SLAVE_ADDR, wr, sizeof wr, rd, sizeof rd)
          == I2C_SOFT_OK);
    expect_clean_critical();

    setup();
    CHECK(i2c_soft_probe(&dev, SLAVE_ADDR) == I2C_SOFT_OK);
    expect_clean_critical();

    setup();
    CHECK(i2c_soft_bus_recover(&dev) == I2C_SOFT_OK);
    expect_clean_critical();

    setup();
    bus.nack_addr = 1U;
    CHECK(i2c_soft_write(&dev, SLAVE_ADDR, data, sizeof data)
          == I2C_SOFT_ERR_NACK);
    expect_clean_critical();
}

/*----------------------------------------------------------------------------*/
/* 18. bus recovery in all four states                                        */
/*----------------------------------------------------------------------------*/

static void test_bus_recover(void)
{
    i2c_soft_config_t cfg;
    int               i;

    /* SDA already high: recovery sends a STOP and rechecks both lines. */
    fake_reset();
    cfg = make_config();
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(i2c_soft_bus_recover(&dev) == I2C_SOFT_OK);

    expect_release(1U, 1U);
    expect_stop();
    expect_recheck(1U, 1U);
    expect_end();
    CHECK(bus.sda_line == 1U);
    CHECK(bus.stops == 1);

    /* SDA released on the third pulse. */
    fake_reset();
    bus.sda_stuck     = 1U;
    bus.release_after = 3U;
    resolve_levels(&bus);
    cfg = make_config();
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(i2c_soft_bus_recover(&dev) == I2C_SOFT_OK);

    expect_release(1U, 0U);
    expect_pulse(0U);
    expect_pulse(0U);
    expect_pulse(1U);
    expect_stop();
    expect_recheck(1U, 1U);
    expect_end();
    CHECK(bus.sda_line == 1U);

    /* SDA never released: nine pulses, then a failure. */
    fake_reset();
    bus.sda_stuck = 1U;
    resolve_levels(&bus);
    cfg = make_config();
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(i2c_soft_bus_recover(&dev) == I2C_SOFT_ERR_BUS);

    expect_release(1U, 0U);
    for ( i = 0; i < 9; ++i ) {
        expect_pulse(0U);
    }
    expect_stop();
    expect_recheck(1U, 0U);
    expect_end();
    CHECK(bus.sda_line == 0U);

    /* SCL held low: reported, and not a single pulse. */
    fake_reset();
    bus.scl_stuck = 1U;
    resolve_levels(&bus);
    cfg = make_config();
    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(i2c_soft_bus_recover(&dev) == I2C_SOFT_ERR_BUS);

    expect_release(0U, 1U);
    expect_end();
    CHECK(bus.scl_rises == 0U);

    CHECK(bus.crit_depth == 0U);
    CHECK(bus.crit_underflow == 0U);
}

/* SCL falls after the entry check: the final STOP check must report BUS. */
static void test_bus_recover_scl_falls(void)
{
    i2c_soft_config_t cfg;

    fake_reset();
    bus.scl_stuck_after = 1U;
    cfg = make_config();

    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(i2c_soft_bus_recover(&dev) == I2C_SOFT_ERR_BUS);

    expect_release(1U, 1U);
    expect_stop();          /* SCL rises, then the slave holds it low */
    expect_recheck(0U, 1U);
    expect_end();

    CHECK(bus.scl_line == 0U);
    CHECK(bus.stops == 0);  /* the STOP never completed */
    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
}

/* SCL falls during the pulse loop: the pulse path must also report BUS. */
static void test_bus_recover_scl_falls_on_pulse(void)
{
    i2c_soft_config_t cfg;
    int               i;

    fake_reset();
    bus.sda_stuck       = 1U;
    bus.scl_stuck_after = 1U;
    resolve_levels(&bus);
    cfg = make_config();

    CHECK(i2c_soft_ctor(&dev, &cfg) == I2C_SOFT_OK);
    CHECK(i2c_soft_bus_recover(&dev) == I2C_SOFT_ERR_BUS);

    expect_release(1U, 0U);
    for ( i = 0; i < 9; ++i ) {
        expect_pulse(0U);
    }
    expect_stop();
    expect_recheck(0U, 0U);
    expect_end();

    CHECK(bus.scl_line == 0U);
    CHECK(bus.sda_line == 0U);
    CHECK(bus.stops == 0);  /* SCL was held low through the STOP */
    CHECK(bus.crit_depth == 0U);
    CHECK(bus.irq_enabled == 1U);
}

int main(void)
{
    test_init_idle();
    test_init_recovers_sda();
    test_init_sda_never_releases();
    test_init_scl_low();
    test_write_single();
    test_write_multi();
    test_read_multi();
    test_write_read();
    test_write_read_read_addr_nack();
    test_zero_length();
    test_probe();
    test_address_nack();
    test_write_nack();
    test_reserved_addresses();
    test_address_limits();
    test_null_arguments();
    test_ctor_validation();
    test_delay_values();
    test_critical_balance();
    test_all_paths_critical();
    test_bus_recover();
    test_bus_recover_scl_falls();
    test_bus_recover_scl_falls_on_pulse();

    CHECK(ev_overflow == 0);

    printf("%d checks, %d failures\n", checks, failures);
    return (failures == 0) ? 0 : 1;
}
