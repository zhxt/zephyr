/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bestechnic_bes2700imp_devkit_panel

#include <zephyr/display/mipi_display.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>

#include "bes2700imp_lcdc.h"
#include BES2700IMP_PANEL_PROFILE

#define PANEL_ID 0x560130U
#define PANEL_PROFILE_BYTES 424U
#define PANEL_PROFILE_COMMANDS 105U
#define PANEL_PROFILE_FNV 0x3ceca8ffU
#define PANEL_VDDI_GPIO 0x4008a000U
#define PANEL_VDDI_MUX  0x40086090U
#define PANEL_VDDI BIT(12)
#define PANEL_VCI_MUX 0x4008600cU
#define PANEL_VCI BIT(23)
#define GPIO_SET     0x00U
#define GPIO_DIR_SET 0x04U
#define GPIO_CLEAR   0x0cU
#define GPIO_LEVEL   0x50U

BUILD_ASSERT(sizeof(q66cn_560130_init[0]) == sizeof(uint8_t) &&
	     sizeof(q66cn_560130_init) == PANEL_PROFILE_BYTES,
	     "BES2700iMP panel profile must contain 424 bytes");

struct bes2700imp_panel_config {
	struct bes2700imp_lcdc bus;
	uint16_t width;
	uint16_t height;
	uint16_t row_offset;
};

struct bes2700imp_panel_data {
	struct k_mutex lock;
	bool buffer_fault;
};

static void set_gpio_outputs(uintptr_t gpio, uintptr_t mux, uint32_t fields,
			     uint32_t pins, uint32_t high)
{
	sys_write32(high, gpio + GPIO_SET);
	sys_write32(pins & ~high, gpio + GPIO_CLEAR);
	sys_write32(pins, gpio + GPIO_DIR_SET);
	sys_clear_bits(mux, fields);
}

static int panel_power_on(const struct bes2700imp_lcdc *bus)
{
	uint32_t pins = bus->clk | bus->cs | (0xfU << bus->d0);

	set_gpio_outputs(bus->reset_gpio, bus->reset_mux, GENMASK(3, 0), bus->reset, 0);
	set_gpio_outputs(PANEL_VDDI_GPIO, PANEL_VDDI_MUX, GENMASK(19, 16), PANEL_VDDI, PANEL_VDDI);
	k_msleep(1);
	set_gpio_outputs(bus->gpio, bus->mux, GENMASK(23, 0), pins, bus->cs);
	set_gpio_outputs(bus->reset_gpio, PANEL_VCI_MUX, GENMASK(31, 28), PANEL_VCI, PANEL_VCI);
	k_msleep(16);
	sys_write32(bus->reset, bus->reset_gpio + GPIO_SET);
	k_msleep(12);
	sys_write32(bus->reset, bus->reset_gpio + GPIO_CLEAR);
	k_msleep(1);
	sys_write32(bus->reset, bus->reset_gpio + GPIO_SET);
	k_msleep(12);
	sys_write32(bus->reset, bus->reset_gpio + GPIO_CLEAR);
	k_msleep(100);
	sys_write32(bus->reset, bus->reset_gpio + GPIO_SET);
	k_msleep(500);

	return (sys_read32(PANEL_VDDI_GPIO + GPIO_LEVEL) & PANEL_VDDI) != 0U &&
	       (sys_read32(bus->reset_gpio + GPIO_LEVEL) & (PANEL_VCI | bus->reset)) ==
		       (PANEL_VCI | bus->reset) &&
	       (sys_read32(bus->gpio + GPIO_LEVEL) & bus->cs) != 0U ? 0 : -EIO;
}

static bool panel_profile_valid(void)
{
	uint32_t digest = 2166136261U;

	for (size_t i = 0; i < PANEL_PROFILE_BYTES; i++) {
		digest = (digest ^ q66cn_560130_init[i]) * 16777619U;
	}
	return digest == PANEL_PROFILE_FNV;
}

static int apply_profile(const struct bes2700imp_lcdc *bus)
{
	const size_t size = sizeof(q66cn_560130_init);
	unsigned int count = 0;
	size_t offset = 0;
	uint8_t power, format;
	int ret;

	while (offset < size) {
		const uint8_t *record = &q66cn_560130_init[offset];

		if (size - offset < 3U || record[2] > 4U || record[2] > size - offset - 3U) {
			return -EINVAL;
		}
		ret = bes2700imp_lcdc_command(bus, record[0], record + 3U, record[2]);
		if (ret != 0) {
			return ret;
		}
		if (record[1] != 0U) {
			k_msleep(record[1]);
		}
		offset += 3U + record[2];
		count++;
	}
	if (count != PANEL_PROFILE_COMMANDS) {
		return -EINVAL;
	}

	k_msleep(200);
	ret = bes2700imp_lcdc_read(bus, MIPI_DCS_GET_POWER_MODE, &power);
	if (ret == 0) {
		ret = bes2700imp_lcdc_read(bus, MIPI_DCS_GET_PIXEL_FORMAT, &format);
	}
	if (ret != 0) {
		return ret;
	}
	printk("BES2700IMP_PANEL PROFILE commands=%u power=%02x format=%02x\n",
	       count, power, format);
	return power == 0x9cU && format == 0x55U ? 0 : -EIO;
}

static int panel_set_window(const struct bes2700imp_lcdc *bus, uint8_t cmd, uint16_t start,
			   uint16_t count)
{
	uint8_t data[4];

	sys_put_be16(start, data);
	sys_put_be16(start + count - 1U, &data[2]);
	return bes2700imp_lcdc_command(bus, cmd, data, sizeof(data));
}

static int panel_write(const struct device *dev, uint16_t x, uint16_t y,
		       const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct bes2700imp_panel_config *cfg = dev->config;
	struct bes2700imp_panel_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	if (buf == NULL || desc == NULL || desc->width == 0U || desc->height == 0U ||
	    x >= cfg->width || y >= cfg->height || desc->width > cfg->width - x ||
	    desc->height > cfg->height - y || desc->pitch < desc->width ||
	    desc->buf_size / 2U < (size_t)(desc->height - 1U) * desc->pitch + desc->width) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->buffer_fault) {
		k_mutex_unlock(&data->lock);
		return -EIO;
	}
	ret = panel_set_window(&cfg->bus, MIPI_DCS_SET_COLUMN_ADDRESS, x, desc->width);
	if (ret == 0) {
		ret = panel_set_window(&cfg->bus, MIPI_DCS_SET_PAGE_ADDRESS,
				       y + cfg->row_offset, desc->height);
	}
	if (ret == 0) {
		ret = bes2700imp_lcdc_write(&cfg->bus, desc, buf);
		data->buffer_fault = ret == -EFAULT;
	}
	if (ret != 0) {
		printk("BES2700IMP_PANEL ERROR transfer x=%u y=%u rc=%d\n", x, y, ret);
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int panel_blank(const struct device *dev, uint8_t command)
{
	const struct bes2700imp_panel_config *cfg = dev->config;
	struct bes2700imp_panel_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = bes2700imp_lcdc_command(&cfg->bus, command, NULL, 0);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int panel_blanking_on(const struct device *dev)
{
	return panel_blank(dev, MIPI_DCS_SET_DISPLAY_OFF);
}

static int panel_blanking_off(const struct device *dev)
{
	return panel_blank(dev, MIPI_DCS_SET_DISPLAY_ON);
}

static void panel_capabilities(const struct device *dev, struct display_capabilities *caps)
{
	const struct bes2700imp_panel_config *cfg = dev->config;

	*caps = (struct display_capabilities){
		.x_resolution = cfg->width,
		.y_resolution = cfg->height,
		.supported_pixel_formats = PIXEL_FORMAT_RGB_565,
		.current_pixel_format = PIXEL_FORMAT_RGB_565,
		.current_orientation = DISPLAY_ORIENTATION_NORMAL,
	};
}

static int panel_format(const struct device *dev, enum display_pixel_format format)
{
	ARG_UNUSED(dev);
	return format == PIXEL_FORMAT_RGB_565 ? 0 : -ENOTSUP;
}

static int panel_init(const struct device *dev)
{
	const struct bes2700imp_panel_config *cfg = dev->config;
	struct bes2700imp_panel_data *data = dev->data;
	uint32_t id = 0;
	int ret;

	if (!panel_profile_valid()) {
		printk("BES2700IMP_PANEL ERROR invalid initialization profile\n");
		return -EINVAL;
	}
	k_mutex_init(&data->lock);
	ret = panel_power_on(&cfg->bus);
	if (ret == 0) {
		ret = bes2700imp_lcdc_prepare(&cfg->bus, cfg->width, cfg->height);
	}
	if (ret != 0) {
		return ret;
	}
	for (uint8_t cmd = 0xdaU; cmd <= 0xdcU; cmd++) {
		uint8_t value;

		ret = bes2700imp_lcdc_read(&cfg->bus, cmd, &value);
		if (ret != 0) {
			return ret;
		}
		id = (id << 8) | value;
	}
	printk("BES2700IMP_PANEL ID value=%06x\n", id);
	if (id != PANEL_ID) {
		return -ENODEV;
	}
	ret = apply_profile(&cfg->bus);
	if (ret != 0) {
		return ret;
	}
	return bes2700imp_lcdc_command(&cfg->bus, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
}

static DEVICE_API(display, panel_api) = {
	.blanking_on = panel_blanking_on,
	.blanking_off = panel_blanking_off,
	.write = panel_write,
	.get_capabilities = panel_capabilities,
	.set_pixel_format = panel_format,
};

#define PANEL_SRAM DT_CHOSEN(zephyr_sram)
#define PANEL_BUF(inst) DT_INST_REG_ADDR_BY_NAME(inst, lcdc_buffer)
#define PANEL_BUF_SIZE(inst) DT_INST_REG_SIZE_BY_NAME(inst, lcdc_buffer)

#define PANEL_DEFINE(inst)                                                                   \
	BUILD_ASSERT(PANEL_BUF_SIZE(inst) >= BES2700IMP_LCDC_GUARD_SIZE +                    \
		     2U * DT_INST_PROP(inst, width) * DT_INST_PROP(inst, height));           \
	BUILD_ASSERT(PANEL_BUF(inst) + PANEL_BUF_SIZE(inst) <= DT_REG_ADDR(PANEL_SRAM) ||    \
		     DT_REG_ADDR(PANEL_SRAM) + DT_REG_SIZE(PANEL_SRAM) <= PANEL_BUF(inst));  \
	static const struct bes2700imp_panel_config panel_config_##inst = {                  \
		.bus = {                                                                     \
			.gpio = DT_INST_REG_ADDR_BY_NAME(inst, gpio),                        \
			.mux = DT_INST_REG_ADDR_BY_NAME(inst, mux),                          \
			.reset_gpio = DT_INST_REG_ADDR_BY_NAME(inst, reset_gpio),            \
			.reset_mux = DT_INST_REG_ADDR_BY_NAME(inst, reset_mux),              \
			.lcdc = DT_INST_REG_ADDR_BY_NAME(inst, lcdc),                        \
			.lcdc_buffer = PANEL_BUF(inst),                                      \
			.lcdc_buffer_size = PANEL_BUF_SIZE(inst),                            \
			.clk = BIT(DT_INST_PROP(inst, clock_pin)),                           \
			.cs = BIT(DT_INST_PROP(inst, cs_pin)),                               \
			.reset = BIT(DT_INST_PROP(inst, reset_pin)),                         \
			.d0 = DT_INST_PROP(inst, data_pin_start),                            \
		},                                                                           \
		.width = DT_INST_PROP(inst, width),                                          \
		.height = DT_INST_PROP(inst, height),                                        \
		.row_offset = DT_INST_PROP(inst, row_offset),                                \
	};                                                                                   \
	static struct bes2700imp_panel_data panel_data_##inst;                               \
	DEVICE_DT_INST_DEFINE(inst, panel_init, NULL, &panel_data_##inst, &panel_config_##inst, \
			      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY, &panel_api);

DT_INST_FOREACH_STATUS_OKAY(PANEL_DEFINE)
