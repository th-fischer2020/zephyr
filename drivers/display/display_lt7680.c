/*
 * Copyright (c) 2025 JUMO GmbH & Co. KG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "display_lt7680.h"

LOG_MODULE_REGISTER(display_lt7680, CONFIG_DISPLAY_LOG_LEVEL);

#define DT_DRV_COMPAT levetop_lt7680

struct lt7680_config {
	struct spi_dt_spec          spi_spec;
	struct gpio_dt_spec         reset_gpio;

	uint16_t                width;
	uint16_t                height;
	uint16_t                reset_delay_ms;

	uint32_t                crystal_freq;

	uint8_t                 vscan_direction;
	uint8_t                 memory_write_direction;
	uint16_t                hsync_pulse_width;
	uint16_t                hsync_back_porch;
	uint16_t                hsync_front_porch;
	uint16_t                vsync_pulse_width;
	uint16_t                vsync_back_porch;
	uint16_t                vsync_front_porch;
	bool                    hsync_active_low;
	bool                    vsync_active_low;
	bool                    de_active_low;
	bool                    pclk_falling_edge;
};

struct lt7680_data {
	enum display_pixel_format   current_pixel_format;
	bool                        display_on;
	bool                        blanking;
	enum display_orientation    orientation;
};

/**
 * @brief  Write a single LT768 register.
 *
 * Requires TWO separate SPI transactions (CS toggled between them):
 *   TX1: [0x00, reg]   — select register address
 *   TX2: [0x80, val]   — write data
 */
static int lt7680_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct lt7680_config *cfg = dev->config;

	// Transaction 1: select register
	uint8_t sel[2] = { LT7680_SPI_SEL_REG, reg };
	struct spi_buf sel_buf = { .buf = sel, .len = 2u };
	const struct spi_buf_set sel_set = { .buffers = &sel_buf, .count = 1u };

	int ret = spi_write_dt(&cfg->spi_spec, &sel_set);
	if (ret < 0) return ret;

	// Transaction 2: write data
	uint8_t wr[2] = { LT7680_SPI_DATA_WRITE, val };
	struct spi_buf wr_buf = { .buf = wr, .len = 2u };
	const struct spi_buf_set wr_set = { .buffers = &wr_buf, .count = 1u };

	ret = spi_write_dt(&cfg->spi_spec, &wr_set);

	return ret;
}

/**
 * @brief  Read the LT768 status byte.
 *
 * Single 2-byte transaction: TX [0x40, 0xFF], RX [discard, status].
 * The second RX byte contains the 8-bit status.
 *
 * Status byte bits:
 *   [7] MEM_WR_FIFO_FULL   [6] MEM_WR_FIFO_EMPTY
 *   [3] 2D_BUSY            [2] SDRAM_READY
 *   [1] POWER_SAVING       [0] INTERRUPT
 */
static int lt7680_read_status(const struct device *dev, uint8_t *status)
{
	const struct lt7680_config *cfg = dev->config;
	uint8_t tx[2] = { LT7680_SPI_STATUS_READ, 0xFFu };
	uint8_t rx[2] = { 0u, 0u };
	struct spi_buf tx_buf = { .buf = tx, .len = 2u };
	struct spi_buf rx_buf = { .buf = rx, .len = 2u };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1u };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1u };

	int ret = spi_transceive_dt(&cfg->spi_spec, &tx_set, &rx_set);
	if (ret == 0)
	{
		*status = rx[1];
	}

	return ret;
}

#ifndef CONFIG_LT7680_TRANSMIT_DATA_AS_BURST
static int lt7680_wait_fifo_not_full(const struct device *dev)
{
	uint8_t status;
	int ret;

	for (size_t i = 0; i < 200; i++) {
		ret = lt7680_read_status(dev, &status);
		if (ret < 0) return ret;
		if (!(status & LT7680_STATUS_MEM_WR_FIFO_FULL)) {
			return 0;
		}
		k_msleep(1);
	}
	LOG_ERR("LT7680: timeout waiting for MEM_WR_FIFO de-assert");
	return -ETIMEDOUT;
}
#endif

static int lt7680_wait_not_power_saving(const struct device *dev)
{
	uint8_t status;
	int ret;

	for (size_t i = 0; i < 200; i++) {
		ret = lt7680_read_status(dev, &status);
		if (ret < 0) return ret;
		if (!(status & LT7680_STATUS_POWER_SAVING)) {
			return 0;
		}
		k_msleep(1);
	}
	LOG_ERR("LT7680: timeout waiting for power-saving de-assert");
	return -ETIMEDOUT;
}

static int lt7680_wait_sdram_ready(const struct device *dev)
{
	uint8_t status;
	int ret;

	for (size_t i = 0; i < 200; i++) {
		ret = lt7680_read_status(dev, &status);
		if (ret < 0) return ret;

		if (status & LT7680_STATUS_SDRAM_READY) {
			return 0;
		}
		k_msleep(1);
	}

	return -ETIMEDOUT;
}

/**
 * @brief  Read a single LT768 register.
 *
 * Requires TWO separate SPI transactions:
 *   TX1: [0x00, reg]        — select register address
 *   TX2: [0xC0, 0xFF]       — read data; second RX byte = register value
 */
static int lt7680_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct lt7680_config *cfg = dev->config;

	// Transaction 1: select register
	uint8_t sel[2] = { LT7680_SPI_SEL_REG, reg };
	struct spi_buf sel_buf = { .buf = sel, .len = 2u };
	const struct spi_buf_set sel_set = { .buffers = &sel_buf, .count = 1u };

	int ret = spi_write_dt(&cfg->spi_spec, &sel_set);
	if (ret < 0) return ret;

	// Transaction 2: read data
	uint8_t tx[2] = { LT7680_SPI_DATA_READ, 0xFFu };
	uint8_t rx[2] = { 0u, 0u };
	struct spi_buf tx_buf = { .buf = tx, .len = 2u };
	struct spi_buf rx_buf = { .buf = rx, .len = 2u };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1u };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1u };

	ret = spi_transceive_dt(&cfg->spi_spec, &tx_set, &rx_set);
	if (ret == 0)
	{
		*val = rx[1];
	}

	return ret;
}

/**
 * @brief  Write two consecutive LT768 registers (16-bit value, LSB first).
 */
static int lt7680_write_reg16(const struct device *dev, uint8_t reg, uint16_t val)
{
	int ret = lt7680_write_reg(dev, reg, (uint8_t)(val & 0xFFu));
	if (ret < 0) return ret;

	ret = lt7680_write_reg(dev, reg + 1u, (uint8_t)(val >> 8u));

	return ret;
}

/**
 * @brief  Write four consecutive LT768 registers (32-bit value, LSB first).
 */
static int lt7680_write_reg32(const struct device *dev, uint8_t reg, uint32_t val)
{
	int ret;

	ret = lt7680_write_reg(dev, reg,        (uint8_t)( val         & 0xFFu));
	if (ret < 0) return ret;

	ret = lt7680_write_reg(dev, reg + 1u,   (uint8_t)((val >>  8u) & 0xFFu));
	if (ret < 0) return ret;

	ret = lt7680_write_reg(dev, reg + 2u,   (uint8_t)((val >> 16u) & 0xFFu));
	if (ret < 0) return ret;

	ret = lt7680_write_reg(dev, reg + 3u,  (uint8_t)((val >> 24u) & 0xFFu));

	return ret;
}

/**
 * @brief  Set active window + move memory-write cursor to (x, y).
 *
 * Active window upper-left: regs [56h-59h]
 * Active window size:        regs [5Ah-5Dh]
 * Write cursor XY:           regs [5Fh-62h]
 */
static int lt7680_set_window(const struct device *dev,
							 uint16_t x, uint16_t y,
							 uint16_t w, uint16_t h)
{
	int ret;

	// Active window origin
	ret = lt7680_write_reg16(dev, LT7680_REG_AWUL_X0, x);
	if (ret < 0) return ret;
	ret = lt7680_write_reg16(dev, LT7680_REG_AWUL_Y0, y);
	if (ret < 0) return ret;

	// Active window size
	ret = lt7680_write_reg16(dev, LT7680_REG_AW_WTH0, w);
	if (ret < 0) return ret;
	ret = lt7680_write_reg16(dev, LT7680_REG_AW_HT0,  h);
	if (ret < 0) return ret;

	// Move write cursor to (x, y)
	ret = lt7680_write_reg16(dev, LT7680_REG_CURH0, x);
	if (ret < 0) return ret;

	ret = lt7680_write_reg16(dev, LT7680_REG_CURV0, y);

	return ret;
}

/**
 * @brief  Push raw pixel bytes to the LT768 frame-buffer.
 *
 * SPI TX: [0x80 (data-write mode), pixel_data...] — single burst transaction.
 * The LT768 auto-increments the write address within the active window.
 */
static int lt7680_push_pixels(const struct device *dev,
							  const uint8_t *data, size_t len)
{
	const struct lt7680_config *cfg = dev->config;

	// Transaction 1: select register
	uint8_t sel[2] = { LT7680_SPI_SEL_REG, LT7680_REG_MRWDP };
	struct spi_buf sel_buf = { .buf = sel, .len = 2u };
	const struct spi_buf_set sel_set = { .buffers = &sel_buf, .count = 1u };

	int ret = spi_write_dt(&cfg->spi_spec, &sel_set);
	if (ret < 0) return ret;

#if defined(CONFIG_LT7680_TRANSMIT_DATA_AS_BURST)
	// Transaction 2: write pixel bytes as burst
	uint8_t cmd = LT7680_SPI_DATA_WRITE;
	struct spi_buf tx_bufs[2] = { { .buf = &cmd, .len = 1u }, { .buf = (void *)data, .len = len } };
	const struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 2u };

	ret = spi_write_dt(&cfg->spi_spec, &tx_set);
	if (ret < 0) return ret;
#else
	// Transaction 2: write pixel pytes as single bytes with fifo-Check
	for (size_t i = 0; i < len; i++) {
		/* Wait for MEM_WR_FIFO to have space for at least one more byte */
		ret = lt7680_wait_fifo_not_full(dev);
		if (ret < 0) return ret;

		/* Transaction 2: write pixel byte */
		uint8_t tx[2] = { LT7680_SPI_DATA_WRITE, data[i] };
		struct spi_buf tx_buf = { .buf = tx, .len = 2u };
		const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1u };

		ret = spi_write_dt(&cfg->spi_spec, &tx_set);
		if (ret < 0) return ret;
	}
#endif

	return 0;
}

/**
 * @brief  Write a rectangular region of pixels.
 */
static int lt7680_write(const struct device *dev,
						const uint16_t x, const uint16_t y,
						const struct display_buffer_descriptor *desc,
						const void *buf)
{
	const struct lt7680_config *cfg  = dev->config;
	struct lt7680_data         *data = dev->data;
	int ret;

	if ((buf == NULL) || (desc == NULL)) {
		return -EINVAL;
	}

	if (((x + desc->width)  > cfg->width) ||
		((y + desc->height) > cfg->height)) {
		LOG_ERR("Write (%u,%u)+(%ux%u) exceeds display (%ux%u)",
				x, y,
				desc->width, desc->height,
				cfg->width, cfg->height);
		return -EINVAL;
	}

	if (data->blanking) {
		// discard while blanked
		return 0;
	}

	ret = lt7680_set_window(dev, x, y, desc->width, desc->height);
	if (ret < 0) {
		LOG_ERR("set_window failed: %d", ret);
		return ret;
	}

	size_t px_size  = (data->current_pixel_format == PIXEL_FORMAT_RGB_888) ? 3u : 2u;
	size_t byte_len = (size_t)desc->width * (size_t)desc->height * px_size;

	ret = lt7680_push_pixels(dev, (const uint8_t *)buf, byte_len);
	if (ret < 0) {
		LOG_ERR("pixel push failed: %d", ret);
	}

	return ret;
}

/**
 * @brief  Read a rectangular region of pixels from the frame-buffer.
 *
 * @note   Read-back requires the SPI bus to support half-duplex / MISO.
 *         Verify your MIPI DBI host controller supports command_read.
 */
static int lt7680_read(const struct device *dev,
					   const uint16_t x, const uint16_t y,
					   const struct display_buffer_descriptor *desc,
					   void *buf)
{
	const struct lt7680_config *cfg  = dev->config;
	struct lt7680_data         *data = dev->data;
	int ret;

	if ((buf == NULL) || (desc == NULL)) {
		return -EINVAL;
	}

	if (((x + desc->width)  > cfg->width) ||
		((y + desc->height) > cfg->height)) {
		return -EINVAL;
	}

	ret = lt7680_set_window(dev, x, y, desc->width, desc->height);
	if (ret < 0) {
		LOG_ERR("set_window failed: %d", ret);
		return ret;
	}

	size_t px_size  = (data->current_pixel_format == PIXEL_FORMAT_RGB_888) ? 3u : 2u;
	size_t byte_len = (size_t)desc->width * (size_t)desc->height * px_size;

	// Transaction 1: select MRWDP register — identical to lt7680_push_pixels
	uint8_t sel[2] = { LT7680_SPI_SEL_REG, LT7680_REG_MRWDP };
	struct spi_buf sel_buf = { .buf = sel, .len = 2u };
	const struct spi_buf_set sel_set = { .buffers = &sel_buf, .count = 1u };

	ret = spi_write_dt(&cfg->spi_spec, &sel_set);
	if (ret < 0) return ret;

#if defined(CONFIG_LT7680_READ_DATA_AS_BURST)
	// Transaction 2: read buffer as burst.
	uint8_t txCmd[2] = {LT7680_SPI_DATA_READ, 0xFFu};
	uint8_t rxDummy[2] = {0u, 0u};
	struct spi_buf tx_bufs[2] = { { .buf = txCmd, .len = 2u },   { .buf = buf, .len = byte_len } };
	struct spi_buf rx_bufs[2] = { { .buf = rxDummy, .len = 2u }, { .buf = buf, .len = byte_len } };
	const struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 2u };
	const struct spi_buf_set rx_set = { .buffers = rx_bufs, .count = 2u };

	ret = spi_transceive_dt(&cfg->spi_spec, &tx_set, &rx_set);
	if (ret < 0) return ret;
#else
	// Transaction 2…N: read one byte per 2-byte transceive — identical to lt7680_read_reg
	uint8_t *out = (uint8_t *)buf;
	for (size_t i = 0; i < byte_len; i++) {
		uint8_t tx[2] = { LT7680_SPI_DATA_READ, 0xFFu };
		uint8_t rx[2] = { 0u, 0u };
		struct spi_buf tx_buf = { .buf = tx, .len = 2u };
		struct spi_buf rx_buf = { .buf = rx, .len = 2u };
		const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1u };
		const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1u };

		ret = spi_transceive_dt(&cfg->spi_spec, &tx_set, &rx_set);
		if (ret < 0) return ret;

		out[i] = rx[1];
	}
#endif

	return 0;
}

static void *lt7680_get_framebuffer(const struct device *dev)
{
	return NULL;
}

static int lt7680_set_brightness(const struct device *dev,
				  const uint8_t brightness)
{
	return -ENOTSUP;
}

static int lt7680_set_contrast(const struct device *dev,
				const uint8_t contrast)
{
	return -ENOTSUP;
}

/**
 * @brief  Assert display blanking – clears Display_ON bit in DPCR (reg 0x12).
 */
static int lt7680_blanking_on(const struct device *dev)
{
	struct lt7680_data *data = dev->data;
	uint8_t val;
	int ret;

	ret = lt7680_read_reg(dev, LT7680_REG_DPCR, &val);
	if (ret < 0) return ret;

	val &= (uint8_t)~LT7680_DPCR_DISPLAY_ON;
	ret = lt7680_write_reg(dev, LT7680_REG_DPCR, val);

	if (ret == 0) {
		data->blanking = true;
	}

	return ret;
}

/**
 * @brief  De-assert display blanking – sets Display_ON bit in DPCR (reg 0x12).
 */
static int lt7680_blanking_off(const struct device *dev)
{
	struct lt7680_data *data = dev->data;
	uint8_t val;
	int ret;

	ret = lt7680_read_reg(dev, LT7680_REG_DPCR, &val);
	if (ret < 0) return ret;

	val |= LT7680_DPCR_DISPLAY_ON;
	ret = lt7680_write_reg(dev, LT7680_REG_DPCR, val);

	if (ret == 0) {
		data->blanking = false;
	}

	return ret;
}

/**
 * @brief  Report display capabilities to the Zephyr display subsystem.
 */
static void lt7680_get_capabilities(const struct device *dev,
									struct display_capabilities *caps)
{
	const struct lt7680_config *cfg  = dev->config;
	const struct lt7680_data   *data = dev->data;

	memset(caps, 0, sizeof(*caps));

	caps->x_resolution            = cfg->width;
	caps->y_resolution            = cfg->height;
	caps->supported_pixel_formats = PIXEL_FORMAT_RGB_565 | PIXEL_FORMAT_RGB_888;
	caps->current_pixel_format    = data->current_pixel_format;
	caps->current_orientation     = DISPLAY_ORIENTATION_NORMAL;
}

/**
 * @brief  Switch pixel format.
 *
 * Programs the colour-depth bits in MPWCTR (reg 0x5E) on the LT768.
 */
static int lt7680_set_pixel_format(const struct device *dev,
								   const enum display_pixel_format fmt)
{
	struct lt7680_data *data = dev->data;
	uint8_t mode_bpp;
	uint8_t val;
	int ret;

	switch (fmt) {
	case PIXEL_FORMAT_RGB_565:
		mode_bpp = LT7680_MPWCTR_16BPP;
		break;
	case PIXEL_FORMAT_RGB_888:
		mode_bpp = LT7680_MPWCTR_24BPP;
		break;
	default:
		LOG_ERR("Unsupported pixel format: %d", fmt);
		return -ENOTSUP;
	}

	ret = lt7680_read_reg(dev, LT7680_REG_MPWCTR, &val);
	if (ret < 0) return ret;

	val = (val & 0xFCu) | mode_bpp;
	ret = lt7680_write_reg(dev, LT7680_REG_MPWCTR, val);

	if (ret == 0) {
		data->current_pixel_format = fmt;
	}

	return ret;
}

/**
 * @brief  Set display orientation (NORMAL only).
 */
static int lt7680_set_orientation(const struct device *dev,
								  const enum display_orientation orientation)
{
	if (orientation != DISPLAY_ORIENTATION_NORMAL) {
		LOG_WRN("LT7680: only DISPLAY_ORIENTATION_NORMAL is supported");
		return -ENOTSUP;
	}
	return 0;
}

/**
 * @brief  Toggle the optional hardware reset GPIO.
 *
 * When no reset GPIO is configured, a power-on stabilisation delay is used.
 */
static void lt7680_hw_reset(const struct device *dev)
{
	const struct lt7680_config *cfg = dev->config;

	if (cfg->reset_gpio.port) {
		gpio_pin_set_dt(&cfg->reset_gpio, 0);
		k_msleep(cfg->reset_delay_ms);

		gpio_pin_set_dt(&cfg->reset_gpio, 1);
		k_msleep(cfg->reset_delay_ms);

		gpio_pin_set_dt(&cfg->reset_gpio, 0);
		k_msleep(cfg->reset_delay_ms);
	}
}

static uint32_t lt7680_calculate_sclk_mhz(const struct device *dev)
{
	const struct lt7680_config *cfg = dev->config;

	uint32_t htotal = (uint32_t)cfg->width
					+ cfg->hsync_back_porch
					+ cfg->hsync_front_porch
					+ cfg->hsync_pulse_width;
	uint32_t vtotal = (uint32_t)cfg->height
					+ cfg->vsync_back_porch
					+ cfg->vsync_front_porch
					+ cfg->vsync_pulse_width;

	uint32_t sclk_mhz = 0;
	uint32_t sclk_hz = htotal * vtotal * 60u;
	uint32_t round_tmp = (sclk_hz % 1000000) / 100000;

	if (round_tmp > 5) {
		sclk_mhz = sclk_hz / 1000000 + 1;
	}
	else {
		sclk_mhz = sclk_hz / 1000000;
	}

	return sclk_mhz;
}

/**
 * @brief  Configure the three on-chip PLLs (SCLK, MCLK, CCLK).
 *
 * Follows the algorithm from LT768_Lib.c::LT768_PLL_Initial():
 *   SCLK = pixel clock derived from panel timing × 60 Hz
 *   MCLK = CCLK = 3 × SCLK  (capped at 100 MHz)
 *   SCLK capped at 65 MHz
 *
 * Each PLL register pair:
 *   [OD_reg]: bits[7:6]=OD, bits[4:1]=R, bit[0]=N[8]
 *   [N_reg]:  N[7:0]
 * After programming all three PLLs, write 0x80 to reg 0x00 to re-lock.
 */
static int lt7680_pll_init(const struct device *dev)
{
	const struct lt7680_config *cfg = dev->config;
	uint32_t xi_mhz = cfg->crystal_freq / 1000000u;
	int ret;

	if (xi_mhz != 10)
	{
		LOG_ERR("Unsupported crystal frequency: %u kHz (only 10 MHz supported)",
				cfg->crystal_freq);
		return -ENOTSUP;
	}

	uint32_t sclk_mhz = lt7680_calculate_sclk_mhz(dev);

	// MCLK = CCLK = 3 * SCLK, capped at 100 MHz
	uint32_t mclk_mhz = sclk_mhz * 3u;
	if (mclk_mhz > 100u) {
		mclk_mhz = 100u;
	}
	uint32_t cclk_mhz = mclk_mhz;

	if (sclk_mhz > 65u) {
		sclk_mhz = 65u;
	}

	LOG_DBG("LT768 PLL: XI=%u MHz, SCLK=%u MHz, MCLK=%u MHz, CCLK=%u MHz",
			xi_mhz, sclk_mhz, mclk_mhz, cclk_mhz);

	uint8_t  od_s, r_s, od_m, r_m, od_c, r_c;
	uint16_t n_s,  n_m,  n_c;

	od_s = 3;
	r_s =  5;
	n_s =  2 * sclk_mhz;

	od_m = 2;
	r_m =  5;
	n_m =  mclk_mhz;

	od_c = 2;
	r_c =  5;
	n_c =  cclk_mhz;

	// Program SCLK PLL 1 (reg 0x05): bits[7:6]=OD, bits[4:1]=R, bit[0]=N[8]
	ret = lt7680_write_reg(dev, LT7680_REG_SCLK_PLL1,
		(uint8_t)((od_s << 6u) | (r_s << 1u) | ((n_s >> 8u) & 0x01u)));
	if (ret < 0) return ret;

	// Program MCLK PLL 1 (reg 0x07): bits[7:6]=OD, bits[4:1]=R, bit[0]=N[8]
	ret = lt7680_write_reg(dev, LT7680_REG_MCLK_PLL1,
		(uint8_t)((od_m << 6u) | (r_m << 1u) | ((n_m >> 8u) & 0x01u)));
	if (ret < 0) return ret;

	// Program CCLK PLL 1 (reg 0x09): bits[7:6]=OD, bits[4:1]=R, bit[0]=N[8]
	ret = lt7680_write_reg(dev, LT7680_REG_CCLK_PLL1,
		(uint8_t)((od_c << 6u) | (r_c << 1u) | ((n_c >> 8u) & 0x01u)));
	if (ret < 0) return ret;

	// Programm SCLK PLL N: reg 0x06 = N[7:0]
	ret = lt7680_write_reg(dev, LT7680_REG_SCLK_PLL_N,
		(uint8_t)(n_s & 0xFFu));
	if (ret < 0) return ret;

	// Program MCLK PLL N: reg 0x08 = N[7:0]
	ret = lt7680_write_reg(dev, LT7680_REG_MCLK_PLL_N,
		(uint8_t)(n_m & 0xFFu));
	if (ret < 0) return ret;

	// Program CCLK PLL N: reg 0x0A = N[7:0]
	ret = lt7680_write_reg(dev, LT7680_REG_CCLK_PLL_N,
		(uint8_t)(n_c & 0xFFu));
	if (ret < 0) return ret;

	// Trigger PLL re-lock: write 0x80 to reg 0x00
	ret = lt7680_write_reg(dev, LT7680_REG_SWRESET, LT7680_SWRESET_PLL_TRIG);
	if (ret < 0) return ret;

	k_msleep(1);  /* wait for PLLs to lock */
	return 0;
}

/**
 * @brief  Initialise the on-chip SDRAM.
 *
 * SDRAM refresh interval formula (from LT768_Lib.c):
 *   sdram_itv = (64_000_000 / 8192) / (1000 / MCLK_MHz) - 2
 *             = MCLK_kHz * 64 / 8192 - 2
 */
static int lt7680_sdram_init(const struct device *dev)
{
	uint16_t sdram_itv;
	int ret;

	uint32_t sclk_mhz = lt7680_calculate_sclk_mhz(dev);

	// MCLK = CCLK = 3 * SCLK, capped at 100 MHz
	uint32_t mclk_mhz = sclk_mhz * 3u;
	if (mclk_mhz > 100u) {
		mclk_mhz = 100u;
	}

	if (sclk_mhz > 65u) {
		sclk_mhz = 65u;
	}

	sdram_itv = (uint16_t)(((sclk_mhz * 1000u * 1000000u) / (8192u * 64u)) - 2u);

	ret = lt7680_write_reg(dev, LT7680_REG_SDRC, LT7680_SDRC_DEFAULT);
	if (ret < 0) return ret;

	ret = lt7680_write_reg(dev, LT7680_REG_SDRCAS, LT7680_SDRCAS_CAS3);
	if (ret < 0) return ret;

	ret = lt7680_write_reg(dev, LT7680_REG_SDRITV0, (uint8_t)(sdram_itv & 0xFFu));
	if (ret < 0) return ret;

	ret = lt7680_write_reg(dev, LT7680_REG_SDRITV1, (uint8_t)(sdram_itv >> 8u));
	if (ret < 0) return ret;

	ret = lt7680_write_reg(dev, LT7680_REG_SDRINI, 0x01u);
	if (ret < 0) return ret;

	// Wait for SDRAM ready (status bit[2] = SDRAM_READY)
	ret = lt7680_wait_sdram_ready(dev);

	if (ret < 0) {
		LOG_ERR("LT768: SDRAM init timeout");
	}

	return ret;
}

/**
 * @brief  Configure the panel control registers from the DT configuration.
 *
 * Follows Set_LCD_Panel() in LT768_Lib.c:
 *   [01h] CCR  : TFT output width, host bus width
 *   [02h] MCR  : RGB format + memory write direction
 *   [03h] ICR  : graphic mode + SDRAM
 *   [12h] DPCR : PCLK edge, display on/off, scan direction, PDATA order
 *   [13h] HDCR : HSYNC/VSYNC/DE polarity
 *   [14h,15h,1Ah,1Bh] : display width/height
 *   [16h,17h] : horizontal back porch
 *   [18h]     : horizontal front porch
 *   [19h]     : HSYNC pulse width
 *   [1Ch,1Dh] : vertical back porch
 *   [1Eh]     : vertical front porch
 *   [1Fh]     : VSYNC pulse width
 *
 * All horizontal values are in units of 8 pixels; vertical in lines.
 *
 * @note  If hsync_pulse_width == 0 (not set in DT), detailed timing setup
 *        is skipped and the LT768 retains its power-on defaults.
 */
static int lt7680_panel_init(const struct device *dev)
{
	const struct lt7680_config *cfg  = dev->config;
	const struct lt7680_data   *data = dev->data;
	int ret;
	uint8_t reg_val = 0;

	ret = lt7680_read_reg(dev, LT7680_REG_CCR, &reg_val);
	if (ret < 0) return ret;

	reg_val &= ~(0x1f);
	reg_val = reg_val | LT7680_CCR_TFT_16BIT | LT7680_CCR_HOST_8BIT;
	reg_val |= (LT7680_CCR_TFT_16BIT | LT7680_CCR_HOST_8BIT);

	// CCR [01h]: 16-bit TFT output, 8-bit host bus (SPI mode)
	ret = lt7680_write_reg(dev, LT7680_REG_CCR, reg_val);
	if (ret < 0) return ret;

	// MCR [02h]: 16-bit bus, 16 bpp by default; LR-TB write direction
	uint8_t mcr = LT7680_MCR_RGB16_16BPP | cfg->memory_write_direction;
	if (data->current_pixel_format == PIXEL_FORMAT_RGB_888) {
		mcr = LT7680_MCR_RGB16_24BPP_M1 | cfg->memory_write_direction;
	}
	ret = lt7680_write_reg(dev, LT7680_REG_MCR, mcr);
	if (ret < 0) return ret;

	// ICR [03h]: graphic mode, SDRAM selected
	ret = lt7680_write_reg(dev, LT7680_REG_ICR,
						   LT7680_ICR_GRAPHIC_MODE | LT7680_ICR_MEM_SDRAM);
	if (ret < 0) return ret;

	// DPCR [12h]: PCLK edge, display OFF (enabled later), scan dir, RGB
	uint8_t dpcr = LT7680_DPCR_DISPLAY_OFF | cfg->vscan_direction
				 | LT7680_DPCR_PDATA_RGB;
	if (cfg->pclk_falling_edge) {
		dpcr |= LT7680_DPCR_PCLK_FALLING;
	}
	ret = lt7680_write_reg(dev, LT7680_REG_DPCR, dpcr);
	if (ret < 0) return ret;

	// HDCR [13h]: sync polarities, DE high active
	uint8_t hdcr = LT7680_HDCR_DE_HIGH_ACT;
	hdcr |= cfg->hsync_active_low ? LT7680_HDCR_HSYNC_LOW_ACT
								  : LT7680_HDCR_HSYNC_HIGH_ACT;
	hdcr |= cfg->vsync_active_low ? LT7680_HDCR_VSYNC_LOW_ACT
								  : LT7680_HDCR_VSYNC_HIGH_ACT;
	hdcr |= cfg->de_active_low ? LT7680_HDCR_DE_LOW_ACT
							   : LT7680_HDCR_DE_HIGH_ACT;
	ret = lt7680_write_reg(dev, LT7680_REG_HDCR, hdcr);
	if (ret < 0) return ret;

	// Skip detailed timing if not provided in DT
	if (cfg->hsync_pulse_width == 0u || cfg->vsync_pulse_width == 0u) {
		LOG_DBG("LT768: panel timing not configured, skipping");
		return 0;
	}

	// --- Horizontal display width [14h, 15h]
	// HDWR = (width / 8) - 1, HDWFTR = width % 8
	//
	ret = lt7680_write_reg(dev, LT7680_REG_HDWR,
						   (uint8_t)(cfg->width / 8u - 1u));
	if (ret < 0) return ret;
	ret = lt7680_write_reg(dev, LT7680_REG_HDWFTR,
						   (uint8_t)(cfg->width % 8u));
	if (ret < 0) return ret;

	// --- Vertical display height [1Ah, 1Bh]
	// VDHR = height - 1
	//
	uint16_t vdhr = (uint16_t)(cfg->height - 1u);
	ret = lt7680_write_reg(dev, LT7680_REG_VDHR0, (uint8_t)(vdhr & 0xFFu));
	if (ret < 0) return ret;
	ret = lt7680_write_reg(dev, LT7680_REG_VDHR1, (uint8_t)(vdhr >> 8u));
	if (ret < 0) return ret;

	// --- Horizontal non-display / back porch [16h, 17h] ---
	// HNDR = (HBPD / 8) - 1, HNDFTR = HBPD % 8
	if (cfg->hsync_back_porch >= 8u) {
		ret = lt7680_write_reg(dev, LT7680_REG_HNDR,
							   (uint8_t)(cfg->hsync_back_porch / 8u - 1u));
		if (ret < 0) return ret;
		ret = lt7680_write_reg(dev, LT7680_REG_HNDFTR,
							   (uint8_t)(cfg->hsync_back_porch % 8u));
	} else {
		ret = lt7680_write_reg(dev, LT7680_REG_HNDR, 0x00u);
		if (ret < 0) return ret;
		ret = lt7680_write_reg(dev, LT7680_REG_HNDFTR,
							   (uint8_t)(cfg->hsync_back_porch));
	}
	if (ret < 0) return ret;

	// --- HSYNC start position / front porch [18h] ---
	// HSTR = (HFPD / 8) - 1
	if (cfg->hsync_front_porch >= 8u) {
		ret = lt7680_write_reg(dev, LT7680_REG_HSTR,
							   (uint8_t)(cfg->hsync_front_porch / 8u - 1u));
	} else {
		ret = lt7680_write_reg(dev, LT7680_REG_HSTR, 0x00u);
	}
	if (ret < 0) return ret;

	// --- HSYNC pulse width [19h] ---
	// HPW = (HSPW / 8) - 1
	if (cfg->hsync_pulse_width >= 8u) {
		ret = lt7680_write_reg(dev, LT7680_REG_HPW,
							   (uint8_t)(cfg->hsync_pulse_width / 8u - 1u));
	} else {
		ret = lt7680_write_reg(dev, LT7680_REG_HPW, 0x00u);
	}
	if (ret < 0) return ret;


	// --- Vertical non-display / back porch [1Ch, 1Dh] ---
	// VNDR = VBPD - 1
	uint16_t vndr = (uint16_t)(cfg->vsync_back_porch - 1u);
	ret = lt7680_write_reg(dev, LT7680_REG_VNDR0, (uint8_t)(vndr & 0xFFu));
	if (ret < 0) return ret;
	ret = lt7680_write_reg(dev, LT7680_REG_VNDR1, (uint8_t)(vndr >> 8u));
	if (ret < 0) return ret;

	// --- VSYNC start position / front porch [1Eh] ---
	// VSTR = VFPD - 1
	ret = lt7680_write_reg(dev, LT7680_REG_VSTR,
						   (uint8_t)(cfg->vsync_front_porch - 1u));
	if (ret < 0) return ret;

	// --- VSYNC pulse width [1Fh] ---
	// VPWR = VSPW - 1
	ret = lt7680_write_reg(dev, LT7680_REG_VPWR,
						   (uint8_t)(cfg->vsync_pulse_width - 1u));
	return ret;
}

/**
 * @brief  Configure the main image layer and canvas for full-screen use.
 *
 * LT768 layer setup sequence:
 *   [20h-23h] Main Image Start Address = 0 (SDRAM base)
 *   [24h-25h] Main Image Width = display width
 *   [26h-29h] Main Window upper-left = (0, 0)
 *   [50h-53h] Canvas Start Address = 0  (same frame-buffer)
 *   [54h-55h] Canvas Image Width = display width
 *   [56h-5Dh] Active Window = (0,0)+full size  (via lt7680_set_window)
 *   [5Eh]     MPWCTR: block/XY mode, colour depth
 */
static int lt7680_layer_init(const struct device *dev)
{
	const struct lt7680_config *cfg  = dev->config;
	const struct lt7680_data   *data = dev->data;
	int ret;

	// --- Main image: origin at SDRAM address 0 ---
	ret = lt7680_write_reg32(dev, LT7680_REG_MISA0, 0u);
	if (ret < 0) return ret;

	// --- Main image width ---
	ret = lt7680_write_reg16(dev, LT7680_REG_MIW0, cfg->width);
	if (ret < 0) return ret;

	// --- Main window upper-left at (0, 0) ---
	ret = lt7680_write_reg16(dev, LT7680_REG_MWULX0, 0u);
	if (ret < 0) return ret;
	ret = lt7680_write_reg16(dev, LT7680_REG_MWULY0, 0u);
	if (ret < 0) return ret;

	// --- Canvas: same frame-buffer, same width ---
	ret = lt7680_write_reg32(dev, LT7680_REG_CVSSA0, 0u);
	if (ret < 0) return ret;
	ret = lt7680_write_reg16(dev, LT7680_REG_CVS_IMWTH0, cfg->width);
	if (ret < 0) return ret;

	// --- Active window = full display ---
	ret = lt7680_set_window(dev, 0u, 0u, cfg->width, cfg->height);
	if (ret < 0) return ret;

	// --- MPWCTR [5Eh]: block/XY mode, colour depth ---
	uint8_t mpw = LT7680_MPWCTR_XY_MODE;
	mpw |= (data->current_pixel_format == PIXEL_FORMAT_RGB_888)
		   ? LT7680_MPWCTR_24BPP
		   : LT7680_MPWCTR_16BPP;
	return lt7680_write_reg(dev, LT7680_REG_MPWCTR, mpw);
}

/**
 * @brief  Zephyr device initialisation.
 *
 * Sequence (following LT768_Lib.c::LT768_Init):
 *   1. Validate MIPI DBI host / GPIO.
 *   2. Hardware reset (GPIO, optional).
 *   3. Wait for power-saving de-assert.
 *   4. PLL initialisation (SCLK, MCLK, CCLK).
 *   5. SDRAM initialisation.
 *   6. Panel control registers (CCR, MCR, ICR, DPCR, HDCR, timing).
 *   7. Frame-buffer / layer setup (main image, canvas, active window).
 *   8. Display ON.
 */
static int lt7680_init(const struct device *dev)
{
	const struct lt7680_config *cfg  = dev->config;
	struct lt7680_data         *data = dev->data;
	int ret;

	if (!spi_is_ready_dt(&cfg->spi_spec)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	if (cfg->reset_gpio.port) {
		if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Reset GPIO config failed: %d", ret);
			return ret;
		}
	}

	lt7680_hw_reset(dev);

	ret = lt7680_wait_not_power_saving(dev);
	if (ret < 0) {
		LOG_ERR("Power-saving wait failed: %d", ret);
		return ret;
	}

	ret = lt7680_pll_init(dev);
	if (ret < 0) {
		LOG_ERR("PLL init failed: %d", ret);
		return ret;
	}

	ret = lt7680_sdram_init(dev);
	if (ret < 0) {
		LOG_ERR("SDRAM init failed: %d", ret);
		return ret;
	}

	ret = lt7680_panel_init(dev);
	if (ret < 0) {
		LOG_ERR("Panel init failed: %d", ret);
		return ret;
	}

	ret = lt7680_layer_init(dev);
	if (ret < 0) {
		LOG_ERR("Layer init failed: %d", ret);
		return ret;
	}

	// Display ON: set DPCR[6] = Display_ON
	uint8_t dpcr;
	ret = lt7680_read_reg(dev, LT7680_REG_DPCR, &dpcr);
	if (ret < 0) {
		LOG_ERR("DPCR read failed: %d", ret);
		return ret;
	}
	dpcr |= LT7680_DPCR_DISPLAY_ON;
	ret = lt7680_write_reg(dev, LT7680_REG_DPCR, dpcr);
	if (ret < 0) {
		LOG_ERR("Display enable failed: %d", ret);
		return ret;
	}

	data->display_on = true;
	data->blanking   = false;

	LOG_INF("LT768 %ux%u initialised (SPI)", cfg->width, cfg->height);
	return 0;
}

static DEVICE_API(display, lt7680_api) = {
	.blanking_on = lt7680_blanking_on,
	.blanking_off = lt7680_blanking_off,
	.write = lt7680_write,
	.read = lt7680_read,
	.get_framebuffer = lt7680_get_framebuffer,
	.set_brightness = lt7680_set_brightness,
	.set_contrast = lt7680_set_contrast,
	.get_capabilities = lt7680_get_capabilities,
	.set_pixel_format = lt7680_set_pixel_format,
	.set_orientation = lt7680_set_orientation,
};

#define LT7680_INIT(inst)                                                           \
	static struct lt7680_data lt7680_data_##inst = {                                \
		.current_pixel_format = PIXEL_FORMAT_RGB_565,                               \
		.display_on           = false,                                              \
		.blanking             = false,                                              \
		.orientation          = DISPLAY_ORIENTATION_NORMAL,                         \
	};                                                                              \
																					\
	static const struct lt7680_config lt7680_config_##inst = {                      \
		.spi_spec    = SPI_DT_SPEC_GET(                                             \
			DT_INST(inst, levetop_lt7680), SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8)),  \
		.reset_gpio         = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),     \
		.width              = DT_INST_PROP(inst, width),                            \
		.height             = DT_INST_PROP(inst, height),                           \
		.reset_delay_ms     = DT_INST_PROP(inst, reset_delay_ms),                   \
		.crystal_freq   = DT_INST_PROP_OR(inst, crystal_frequency, 10000000),       \
		.vscan_direction    = DT_INST_PROP_OR(inst, vscan_direction, LT7680_DPCR_VSCAN_T_TO_B), \
		.memory_write_direction = DT_INST_PROP_OR(inst, memory_write_direction, LT7680_MCR_MWRITE_LR_TB), \
		.hsync_pulse_width  = DT_INST_PROP_OR(inst, hsync_pulse_width,  0),         \
		.hsync_back_porch   = DT_INST_PROP_OR(inst, hsync_back_porch,   0),         \
		.hsync_front_porch  = DT_INST_PROP_OR(inst, hsync_front_porch,  0),         \
		.vsync_pulse_width  = DT_INST_PROP_OR(inst, vsync_pulse_width,  0),         \
		.vsync_back_porch   = DT_INST_PROP_OR(inst, vsync_back_porch,   0),         \
		.vsync_front_porch  = DT_INST_PROP_OR(inst, vsync_front_porch,  0),         \
		.hsync_active_low   = DT_INST_PROP(inst, hsync_active_low),                 \
		.vsync_active_low   = DT_INST_PROP(inst, vsync_active_low),                 \
		.de_active_low      = DT_INST_PROP(inst, de_active_low),                    \
		.pclk_falling_edge  = DT_INST_PROP(inst, pclk_falling_edge),                \
	};                                                                              \
																					\
	DEVICE_DT_INST_DEFINE(inst,                                                     \
						  lt7680_init,                                              \
						  NULL,                                                     \
						  &lt7680_data_##inst,                                      \
						  &lt7680_config_##inst,                                    \
						  POST_KERNEL,                                              \
						  CONFIG_DISPLAY_INIT_PRIORITY,                             \
						  &lt7680_api);

DT_INST_FOREACH_STATUS_OKAY(LT7680_INIT)
