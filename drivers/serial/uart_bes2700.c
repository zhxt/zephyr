/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bestechnic_bes2700_bth_uart

#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>

/* The PL011-style register bank includes BES oversampling extensions. */
#define UART_DR       0x00U
#define UART_ECR      0x04U
#define UART_FR       0x18U
#define UART_IBRD     0x24U
#define UART_FBRD     0x28U
#define UART_LCR      0x2cU
#define UART_CR       0x30U
#define UART_IFLS     0x34U
#define UART_IMSC     0x38U
#define UART_ICR      0x44U
#define UART_DMACR    0x48U
#define UART_OVSAMP   0x4cU
#define UART_OVSAMPST 0x50U
#define UART_RXEXT    0x54U

#define UART_FR_BUSY BIT(3)
#define UART_FR_RXFE BIT(4)
#define UART_FR_TXFF BIT(5)
#define UART_ERRORS  GENMASK(3, 0)

struct bes2700_uart_config {
	uintptr_t base;
	uint32_t clock;
	uint32_t baud;
};

struct bes2700_uart_data {
	uint32_t rx_errors;
};

static uint32_t bes2700_uart_read(const struct device *dev, uint32_t reg)
{
	const struct bes2700_uart_config *config = dev->config;

	return sys_read32(config->base + reg);
}

static void bes2700_uart_write(const struct device *dev, uint32_t reg, uint32_t value)
{
	const struct bes2700_uart_config *config = dev->config;

	sys_write32(value, config->base + reg);
}

static int bes2700_uart_poll_in(const struct device *dev, unsigned char *ch)
{
	struct bes2700_uart_data *data = dev->data;
	unsigned int key = irq_lock();
	uint32_t value;

	if ((bes2700_uart_read(dev, UART_FR) & UART_FR_RXFE) != 0U) {
		irq_unlock(key);
		return -1;
	}

	value = bes2700_uart_read(dev, UART_DR);
	data->rx_errors |= (value >> 8) & UART_ERRORS;
	*ch = (unsigned char)value;
	irq_unlock(key);
	return 0;
}

static void bes2700_uart_poll_out(const struct device *dev, unsigned char ch)
{
	while ((bes2700_uart_read(dev, UART_FR) & UART_FR_TXFF) != 0U) {
	}
	bes2700_uart_write(dev, UART_DR, ch);
}

static int bes2700_uart_err_check(const struct device *dev)
{
	struct bes2700_uart_data *data = dev->data;
	unsigned int key = irq_lock();
	uint32_t status = bes2700_uart_read(dev, UART_ECR) & UART_ERRORS;
	uint32_t errors = status | data->rx_errors;

	data->rx_errors = 0U;
	if (status != 0U) {
		bes2700_uart_write(dev, UART_ECR, 0U);
	}
	irq_unlock(key);

	return (((errors & BIT(0)) != 0U) ? UART_ERROR_FRAMING : 0) |
	       (((errors & BIT(1)) != 0U) ? UART_ERROR_PARITY : 0) |
	       (((errors & BIT(2)) != 0U) ? UART_BREAK : 0) |
	       (((errors & BIT(3)) != 0U) ? UART_ERROR_OVERRUN : 0);
}

static int bes2700_uart_init(const struct device *dev)
{
	const struct bes2700_uart_config *config = dev->config;
	uint64_t divisor;
	uint32_t budget = 1000000U;

	if ((config->baud == 0U) || (config->clock == 0U)) {
		return -EINVAL;
	}
	divisor = ((uint64_t)config->clock * 4U + config->baud / 2U) / config->baud;
	if ((divisor < 64U) || ((divisor >> 6) > UINT16_MAX)) {
		return -EINVAL;
	}

	/* The loader must drain its output before giving this driver ownership. */
	while ((bes2700_uart_read(dev, UART_FR) & UART_FR_BUSY) != 0U) {
		if (--budget == 0U) {
			return -ETIMEDOUT;
		}
	}
	bes2700_uart_write(dev, UART_IMSC, 0U);
	bes2700_uart_write(dev, UART_CR, 0U);
	bes2700_uart_write(dev, UART_LCR, 0U);
	bes2700_uart_write(dev, UART_DMACR, 0U);
	bes2700_uart_write(dev, UART_ECR, 0U);
	bes2700_uart_write(dev, UART_ICR, 0x7ffU);
	bes2700_uart_write(dev, UART_OVSAMP, 15U);
	bes2700_uart_write(dev, UART_OVSAMPST, 8U);
	bes2700_uart_write(dev, UART_IBRD, (uint32_t)(divisor >> 6));
	bes2700_uart_write(dev, UART_FBRD, (uint32_t)(divisor & 63U));
	bes2700_uart_write(dev, UART_RXEXT, 3U);
	bes2700_uart_write(dev, UART_IFLS, 0U);
	bes2700_uart_write(dev, UART_LCR, (3U << 5) | BIT(4));
	bes2700_uart_write(dev, UART_CR, BIT(0) | BIT(8) | BIT(9));
	return 0;
}

static DEVICE_API(uart, bes2700_uart_api) = {
	.poll_in = bes2700_uart_poll_in,
	.poll_out = bes2700_uart_poll_out,
	.err_check = bes2700_uart_err_check,
};

#define BES2700_UART_DEFINE(inst)                                                                  \
	BUILD_ASSERT(!DT_INST_PROP(inst, hw_flow_control), "Flow control is not supported");       \
	static struct bes2700_uart_data bes2700_uart_data_##inst;                                  \
	static const struct bes2700_uart_config bes2700_uart_config_##inst = {                     \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.clock = DT_INST_PROP(inst, clock_frequency),                                      \
		.baud = DT_INST_PROP(inst, current_speed),                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, bes2700_uart_init, NULL, &bes2700_uart_data_##inst,            \
			      &bes2700_uart_config_##inst, PRE_KERNEL_1,                           \
			      CONFIG_SERIAL_INIT_PRIORITY, &bes2700_uart_api);

DT_INST_FOREACH_STATUS_OKAY(BES2700_UART_DEFINE)
