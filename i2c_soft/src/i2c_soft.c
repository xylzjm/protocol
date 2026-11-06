/* SPDX-License-Identifier: MIT */

#include "i2c_soft.h"

/*----------------------------------------------------------------------------*/
/* Validation                                                                 */
/*----------------------------------------------------------------------------*/

/** @brief Every GPIO and critical operation must be present. */
static uint8_t config_valid(const i2c_soft_config_t* config)
{
    if ( config->port.sda_write == NULL || config->port.scl_write == NULL
         || config->port.sda_read == NULL || config->port.scl_read == NULL
         || config->port.delay_us == NULL ) {
        return 0U;
    }
    if ( config->critical.enter == NULL || config->critical.exit == NULL ) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief The 7-bit address space this library accepts.
 *
 * 0x00 is the general call and 0x78..0x7F carry the 10-bit prefix and other
 * reservations, so neither can name a plain 7-bit slave.
 */
static uint8_t addr_valid(uint8_t addr7)
{
    return (uint8_t)((addr7 >= 0x01U) && (addr7 <= 0x77U));
}

/*----------------------------------------------------------------------------*/
/* Waveform                                                                   */
/*----------------------------------------------------------------------------*/

/* Each bit is one critical section; delay_us() must work with IRQs masked. */

/** @brief One half SCL period, spent inside the caller's critical section. */
static void half_period(i2c_soft_t* me)
{
    me->port.delay_us(me->port.ctx, I2C_SOFT_HALF_PERIOD_US);
}

/**
 * @brief START: SDA falls while SCL is high.
 *
 * The bus must be idle-high, which i2c_soft_init() and every STOP leave
 * behind; the group does not re-release the lines.
 */
static void start_cond(i2c_soft_t* me)
{
    me->critical.enter(me->critical.ctx);
    me->port.sda_write(me->port.ctx, 0U);
    half_period(me);
    me->port.scl_write(me->port.ctx, 0U);
    half_period(me);
    me->critical.exit(me->critical.ctx);
}

/** @brief Repeated START from the SCL-low state a write leaves behind. */
static void restart_cond(i2c_soft_t* me)
{
    me->critical.enter(me->critical.ctx);
    me->port.sda_write(me->port.ctx, 1U);
    half_period(me);
    me->port.scl_write(me->port.ctx, 1U);
    half_period(me);
    me->port.sda_write(me->port.ctx, 0U);
    half_period(me);
    me->port.scl_write(me->port.ctx, 0U);
    half_period(me);
    me->critical.exit(me->critical.ctx);
}

/**
 * @brief STOP: SDA rises while SCL is high.
 *
 * SCL is pulled low first, so this is a STOP whatever the bus state; an SDA
 * fall with SCL high would be read as a START instead.
 */
static void stop_cond(i2c_soft_t* me)
{
    me->critical.enter(me->critical.ctx);
    me->port.scl_write(me->port.ctx, 0U);
    me->port.sda_write(me->port.ctx, 0U);
    half_period(me);
    me->port.scl_write(me->port.ctx, 1U);
    half_period(me);
    me->port.sda_write(me->port.ctx, 1U);
    half_period(me);
    me->critical.exit(me->critical.ctx);
}

/** @brief One write slot: set SDA, then clock it out. */
static void write_bit(i2c_soft_t* me, uint8_t bit)
{
    me->critical.enter(me->critical.ctx);
    me->port.sda_write(me->port.ctx, bit);
    half_period(me);
    me->port.scl_write(me->port.ctx, 1U);
    half_period(me);
    me->port.scl_write(me->port.ctx, 0U);
    me->critical.exit(me->critical.ctx);
}

/** @brief One read or acknowledge slot: release SDA, clock, sample, close. */
static uint8_t read_bit(i2c_soft_t* me)
{
    uint8_t level;

    me->critical.enter(me->critical.ctx);
    me->port.sda_write(me->port.ctx, 1U);
    half_period(me);
    me->port.scl_write(me->port.ctx, 1U);
    half_period(me);
    level = (uint8_t)((me->port.sda_read(me->port.ctx) != 0U) ? 1U : 0U);
    me->port.scl_write(me->port.ctx, 0U);
    me->critical.exit(me->critical.ctx);
    return level;
}

/** @brief Shift one byte out MSB-first and sample the acknowledge slot. */
static i2c_soft_status_t write_byte(i2c_soft_t* me, uint8_t byte)
{
    int i;

    for ( i = 7; i >= 0; --i ) {
        write_bit(me, (uint8_t)((byte >> i) & 1U));
    }
    if ( read_bit(me) != 0U ) {
        return I2C_SOFT_ERR_NACK;
    }
    return I2C_SOFT_OK;
}

/** @brief Shift one byte in MSB-first. */
static uint8_t read_byte(i2c_soft_t* me)
{
    uint8_t byte = 0U;
    int     i;

    for ( i = 0; i < 8; ++i ) {
        byte = (uint8_t)((byte << 1) | read_bit(me));
    }
    return byte;
}

/*----------------------------------------------------------------------------*/
/* Bus recovery                                                               */
/*----------------------------------------------------------------------------*/

/** @brief Both lines read high: the bus is idle. */
static uint8_t bus_idle(i2c_soft_t* me)
{
    uint8_t scl_high;
    uint8_t sda_high;

    me->critical.enter(me->critical.ctx);
    scl_high = (uint8_t)(me->port.scl_read(me->port.ctx) != 0U);
    sda_high = (uint8_t)(me->port.sda_read(me->port.ctx) != 0U);
    me->critical.exit(me->critical.ctx);

    return (uint8_t)((scl_high != 0U) && (sda_high != 0U));
}

i2c_soft_status_t i2c_soft_bus_recover(i2c_soft_t* me)
{
    uint8_t index;
    uint8_t scl_low;
    uint8_t sda_high;

    if ( me == NULL ) {
        return I2C_SOFT_ERR_ARG;
    }

    me->critical.enter(me->critical.ctx);
    me->port.sda_write(me->port.ctx, 1U);
    me->port.scl_write(me->port.ctx, 1U);
    half_period(me);
    scl_low  = (uint8_t)(me->port.scl_read(me->port.ctx) == 0U);
    sda_high = (uint8_t)(me->port.sda_read(me->port.ctx) != 0U);
    me->critical.exit(me->critical.ctx);

    if ( scl_low != 0U ) {
        /* Report only: a held SCL gets no edge, and no push-pull fight. */
        return I2C_SOFT_ERR_BUS;
    }
    if ( sda_high != 0U ) {
        stop_cond(me);
        return (bus_idle(me) != 0U) ? I2C_SOFT_OK : I2C_SOFT_ERR_BUS;
    }

    /* Nine clocks at most: a slave that let go mid-way stops the count. */
    for ( index = 0U; index < 9U; ++index ) {
        me->critical.enter(me->critical.ctx);
        me->port.scl_write(me->port.ctx, 0U);
        half_period(me);
        me->port.scl_write(me->port.ctx, 1U);
        half_period(me);
        sda_high = (uint8_t)(me->port.sda_read(me->port.ctx) != 0U);
        me->critical.exit(me->critical.ctx);
        if ( sda_high != 0U ) {
            break;
        }
    }

    stop_cond(me);
    return (bus_idle(me) != 0U) ? I2C_SOFT_OK : I2C_SOFT_ERR_BUS;
}

/*----------------------------------------------------------------------------*/
/* Construction and initialization                                            */
/*----------------------------------------------------------------------------*/

i2c_soft_status_t i2c_soft_ctor(i2c_soft_t* me,
                                const i2c_soft_config_t* config)
{
    if ( me == NULL || config == NULL || config_valid(config) == 0U ) {
        return I2C_SOFT_ERR_ARG;
    }

    me->port     = config->port;
    me->critical = config->critical;
    return I2C_SOFT_OK;
}

i2c_soft_status_t i2c_soft_init(i2c_soft_t* me)
{
    i2c_soft_status_t status;
    uint8_t           scl_low;
    uint8_t           sda_low;

    if ( me == NULL ) {
        return I2C_SOFT_ERR_ARG;
    }

    me->critical.enter(me->critical.ctx);
    me->port.sda_write(me->port.ctx, 1U);
    me->port.scl_write(me->port.ctx, 1U);
    half_period(me);
    scl_low = (uint8_t)(me->port.scl_read(me->port.ctx) == 0U);
    sda_low = (uint8_t)(me->port.sda_read(me->port.ctx) == 0U);
    me->critical.exit(me->critical.ctx);

    if ( scl_low != 0U ) {
        /* A clock held down cannot be freed from software: report it. */
        return I2C_SOFT_ERR_BUS;
    }
    if ( sda_low != 0U ) {
        status = i2c_soft_bus_recover(me);
        if ( status != I2C_SOFT_OK ) {
            return status;
        }
    }

    if ( bus_idle(me) == 0U ) {
        return I2C_SOFT_ERR_BUS;
    }
    return I2C_SOFT_OK;
}

/*----------------------------------------------------------------------------*/
/* Transactions                                                               */
/*----------------------------------------------------------------------------*/

i2c_soft_status_t i2c_soft_write(i2c_soft_t* me,
                                 uint8_t addr7,
                                 const uint8_t* data,
                                 size_t len)
{
    i2c_soft_status_t status;
    size_t            index;

    if ( me == NULL || data == NULL || len == 0U
         || addr_valid(addr7) == 0U ) {
        return I2C_SOFT_ERR_ARG;
    }

    start_cond(me);
    status = write_byte(me, (uint8_t)(addr7 << 1));
    if ( status != I2C_SOFT_OK ) {
        stop_cond(me);
        return I2C_SOFT_ERR_NACK;
    }

    for ( index = 0U; index < len; ++index ) {
        status = write_byte(me, data[index]);
        if ( status != I2C_SOFT_OK ) {
            stop_cond(me);
            return I2C_SOFT_ERR_NACK;
        }
    }

    stop_cond(me);
    return I2C_SOFT_OK;
}

i2c_soft_status_t i2c_soft_read(i2c_soft_t* me,
                                uint8_t addr7,
                                uint8_t* data,
                                size_t len)
{
    i2c_soft_status_t status;
    size_t            index;

    if ( me == NULL || data == NULL || len == 0U
         || addr_valid(addr7) == 0U ) {
        return I2C_SOFT_ERR_ARG;
    }

    start_cond(me);
    status = write_byte(me, (uint8_t)((addr7 << 1) | 1U));
    if ( status != I2C_SOFT_OK ) {
        stop_cond(me);
        return I2C_SOFT_ERR_NACK;
    }

    for ( index = 0U; index < len; ++index ) {
        data[index] = read_byte(me);
        /* Acknowledge all but the last byte, which ends the read. */
        write_bit(me, (uint8_t)(((index + 1U) < len) ? 0U : 1U));
    }

    stop_cond(me);
    return I2C_SOFT_OK;
}

i2c_soft_status_t i2c_soft_write_read(i2c_soft_t* me,
                                      uint8_t addr7,
                                      const uint8_t* wr,
                                      size_t wr_len,
                                      uint8_t* rd,
                                      size_t rd_len)
{
    i2c_soft_status_t status;
    size_t            index;

    if ( me == NULL || wr == NULL || rd == NULL || wr_len == 0U
         || rd_len == 0U || addr_valid(addr7) == 0U ) {
        return I2C_SOFT_ERR_ARG;
    }

    start_cond(me);
    status = write_byte(me, (uint8_t)(addr7 << 1));
    if ( status != I2C_SOFT_OK ) {
        stop_cond(me);
        return I2C_SOFT_ERR_NACK;
    }

    for ( index = 0U; index < wr_len; ++index ) {
        status = write_byte(me, wr[index]);
        if ( status != I2C_SOFT_OK ) {
            stop_cond(me);
            return I2C_SOFT_ERR_NACK;
        }
    }

    restart_cond(me);
    status = write_byte(me, (uint8_t)((addr7 << 1) | 1U));
    if ( status != I2C_SOFT_OK ) {
        stop_cond(me);
        return I2C_SOFT_ERR_NACK;
    }

    for ( index = 0U; index < rd_len; ++index ) {
        rd[index] = read_byte(me);
        write_bit(me, (uint8_t)(((index + 1U) < rd_len) ? 0U : 1U));
    }

    stop_cond(me);
    return I2C_SOFT_OK;
}

i2c_soft_status_t i2c_soft_probe(i2c_soft_t* me,
                                 uint8_t addr7)
{
    i2c_soft_status_t status;

    if ( me == NULL || addr_valid(addr7) == 0U ) {
        return I2C_SOFT_ERR_ARG;
    }

    start_cond(me);
    status = write_byte(me, (uint8_t)(addr7 << 1));
    stop_cond(me);
    return status;
}
