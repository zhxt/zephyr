/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bestechnic_bes2700imp_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

#define GPIO_OUT_SET         0x00U
#define GPIO_DIR_SET         0x04U
#define GPIO_OUT_CLEAR       0x0cU
#define GPIO_DIR_CLEAR       0x10U
#define GPIO_IRQ_DISABLE     0x18U
#define GPIO_IRQ_UNMASK      0x1cU
#define GPIO_IRQ_FALLING     0x24U
#define GPIO_IRQ_NO_DEBOUNCE 0x28U
#define GPIO_IRQ_ENABLE      0x30U
#define GPIO_IRQ_MASK        0x34U
#define GPIO_IRQ_EDGE        0x38U
#define GPIO_IRQ_CLEAR       0x4cU
#define GPIO_IN              0x50U
#define PSC_IRQ_ENABLE       0x00U
#define PSC_IRQ_STATUS       0x0cU

struct bes2700imp_gpio_config {
	struct gpio_driver_config common;
	uintptr_t gpio;
	uintptr_t psc;
	gpio_port_pins_t irq_mask;
	void (*connect_irq)(void);
};

struct bes2700imp_gpio_data {
	struct gpio_driver_data common;
	struct k_spinlock lock;
	sys_slist_t callbacks;
};

static int bes2700imp_gpio_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct bes2700imp_gpio_config *cfg = dev->config;
	struct bes2700imp_gpio_data *data = dev->data;
	const uint32_t bit = BIT(pin);
	const gpio_flags_t supported =
		GPIO_INPUT | GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH | GPIO_OUTPUT_INIT_LOW;
	k_spinlock_key_t key;

	if ((cfg->common.port_pin_mask & bit) == 0U) {
		return -EINVAL;
	}
	if ((flags & ~(supported | GPIO_ACTIVE_LOW)) != 0U ||
	    (flags & (GPIO_INPUT | GPIO_OUTPUT)) == (GPIO_INPUT | GPIO_OUTPUT)) {
		return -ENOTSUP;
	}
	key = k_spin_lock(&data->lock);
	if ((flags & GPIO_OUTPUT) != 0U) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			sys_write32(bit, cfg->gpio + GPIO_OUT_SET);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			sys_write32(bit, cfg->gpio + GPIO_OUT_CLEAR);
		}
		sys_write32(bit, cfg->gpio + GPIO_DIR_SET);
	} else {
		sys_write32(bit, cfg->gpio + GPIO_DIR_CLEAR);
	}
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int bes2700imp_gpio_get(const struct device *dev, gpio_port_value_t *value)
{
	const struct bes2700imp_gpio_config *cfg = dev->config;

	*value = sys_read32(cfg->gpio + GPIO_IN) & cfg->common.port_pin_mask;
	return 0;
}

static int bes2700imp_gpio_set_masked(const struct device *dev, gpio_port_pins_t mask,
				   gpio_port_value_t value)
{
	const struct bes2700imp_gpio_config *cfg = dev->config;
	struct bes2700imp_gpio_data *data = dev->data;
	k_spinlock_key_t key;

	if ((mask & ~cfg->common.port_pin_mask) != 0U) {
		return -EINVAL;
	}
	key = k_spin_lock(&data->lock);
	sys_write32(mask & value, cfg->gpio + GPIO_OUT_SET);
	sys_write32(mask & ~value, cfg->gpio + GPIO_OUT_CLEAR);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int bes2700imp_gpio_set(const struct device *dev, gpio_port_pins_t pins)
{
	return bes2700imp_gpio_set_masked(dev, pins, pins);
}

static int bes2700imp_gpio_clear(const struct device *dev, gpio_port_pins_t pins)
{
	return bes2700imp_gpio_set_masked(dev, pins, 0);
}

static int bes2700imp_gpio_toggle(const struct device *dev, gpio_port_pins_t pins)
{
	const struct bes2700imp_gpio_config *cfg = dev->config;
	struct bes2700imp_gpio_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t output;

	if ((pins & ~cfg->common.port_pin_mask) != 0U) {
		return -EINVAL;
	}
	key = k_spin_lock(&data->lock);
	output = sys_read32(cfg->gpio + GPIO_OUT_SET);
	sys_write32(pins & ~output, cfg->gpio + GPIO_OUT_SET);
	sys_write32(pins & output, cfg->gpio + GPIO_OUT_CLEAR);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int bes2700imp_gpio_irq_configure(const struct device *dev, gpio_pin_t pin,
				       enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct bes2700imp_gpio_config *cfg = dev->config;
	struct bes2700imp_gpio_data *data = dev->data;
	const uint32_t bit = BIT(pin);
	k_spinlock_key_t key;
	uint32_t routes;

	if ((cfg->irq_mask & bit) == 0U ||
	    (mode != GPIO_INT_MODE_DISABLED &&
	     (mode != GPIO_INT_MODE_EDGE || trig != GPIO_INT_TRIG_LOW))) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);
	routes = sys_read32(cfg->psc + PSC_IRQ_ENABLE);
	sys_write32(bit, cfg->gpio + GPIO_IRQ_DISABLE);
	sys_write32(bit, cfg->gpio + GPIO_IRQ_MASK);
	if (mode == GPIO_INT_MODE_DISABLED) {
		sys_write32(routes & ~bit, cfg->psc + PSC_IRQ_ENABLE);
	} else {
		sys_write32(bit, cfg->gpio + GPIO_IRQ_EDGE);
		sys_write32(bit, cfg->gpio + GPIO_IRQ_FALLING);
		sys_write32(bit, cfg->gpio + GPIO_IRQ_NO_DEBOUNCE);
		k_busy_wait(1);
		sys_write32(bit, cfg->gpio + GPIO_IRQ_CLEAR);
		sys_write32(routes | bit, cfg->psc + PSC_IRQ_ENABLE);
		sys_write32(bit, cfg->gpio + GPIO_IRQ_UNMASK);
		sys_write32(bit, cfg->gpio + GPIO_IRQ_ENABLE);
	}
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int bes2700imp_gpio_manage_callback(const struct device *dev,
					 struct gpio_callback *callback, bool set)
{
	struct bes2700imp_gpio_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int ret = gpio_manage_callback(&data->callbacks, callback, set);

	k_spin_unlock(&data->lock, key);
	return ret;
}

static void bes2700imp_gpio_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct bes2700imp_gpio_config *cfg = dev->config;
	struct bes2700imp_gpio_data *data = dev->data;
	uint32_t pending =
		sys_read32(cfg->psc + PSC_IRQ_STATUS) & sys_read32(cfg->psc + PSC_IRQ_ENABLE) &
		cfg->irq_mask;

	if (pending != 0U) {
		sys_write32(pending, cfg->gpio + GPIO_IRQ_CLEAR);
		barrier_dsync_fence_full();
		gpio_fire_callbacks(&data->callbacks, dev, pending);
	}
}

static int bes2700imp_gpio_init(const struct device *dev)
{
	const struct bes2700imp_gpio_config *cfg = dev->config;

	sys_write32(cfg->irq_mask, cfg->gpio + GPIO_IRQ_DISABLE);
	sys_write32(cfg->irq_mask, cfg->gpio + GPIO_IRQ_MASK);
	sys_write32(cfg->irq_mask, cfg->gpio + GPIO_IRQ_CLEAR);
	cfg->connect_irq();
	return 0;
}

static DEVICE_API(gpio, bes2700imp_gpio_api) = {
	.pin_configure = bes2700imp_gpio_configure,
	.port_get_raw = bes2700imp_gpio_get,
	.port_set_masked_raw = bes2700imp_gpio_set_masked,
	.port_set_bits_raw = bes2700imp_gpio_set,
	.port_clear_bits_raw = bes2700imp_gpio_clear,
	.port_toggle_bits = bes2700imp_gpio_toggle,
	.pin_interrupt_configure = bes2700imp_gpio_irq_configure,
	.manage_callback = bes2700imp_gpio_manage_callback,
};

#define BES2700IMP_GPIO_DEFINE(n)                                                            \
	static void bes2700imp_gpio_irq_##n(void)                                            \
	{                                                                                    \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), bes2700imp_gpio_isr,  \
			    DEVICE_DT_INST_GET(n), 0);                                       \
		k_irq_clear_pending(DT_INST_IRQN(n));                                        \
		irq_enable(DT_INST_IRQN(n));                                                 \
	}                                                                                    \
	static const struct bes2700imp_gpio_config bes2700imp_gpio_cfg_##n = {               \
		.common = { .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n) },           \
		.gpio = DT_INST_REG_ADDR_BY_NAME(n, gpio),                                   \
		.psc = DT_INST_REG_ADDR_BY_NAME(n, psc),                                     \
		.irq_mask = DT_INST_PROP(n, interrupt_pin_mask),                             \
		.connect_irq = bes2700imp_gpio_irq_##n,                                      \
	};                                                                                   \
	static struct bes2700imp_gpio_data bes2700imp_gpio_data_##n;                         \
	DEVICE_DT_INST_DEFINE(n, bes2700imp_gpio_init, NULL, &bes2700imp_gpio_data_##n,      \
			      &bes2700imp_gpio_cfg_##n, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, \
			      &bes2700imp_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(BES2700IMP_GPIO_DEFINE)
