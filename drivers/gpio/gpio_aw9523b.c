/*
 * Copyright (c) 2024 Zhang Xingtao <zhxt@live.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/toolchain.h>
#include <zephyr/logging/log.h>

#include "gpio_aw9523b.h"

#define DT_DRV_COMPAT awinic_aw9523b_gpio

LOG_MODULE_REGISTER(gpio_aw9523b, CONFIG_GPIO_LOG_LEVEL);

struct gpio_aw9523b_config {
	struct gpio_driver_config common;
	struct i2c_dt_spec i2c;
	uint32_t ngpios;
};

struct gpio_aw9523b_data {
	struct gpio_driver_data common;
	struct k_mutex mutex;
	sys_slist_t cb_list_gpio;
};

static int gpio_aw9523b_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	int ret;
	return ret;
}

static int gpio_aw9523b_port_get_raw(const struct device *dev, uint32_t *value)
{
	int ret;
        const struct gpio_aw9523b_config *cfg = dev->config;
        struct gpio_aw9523b_data *data = dev->data;

        if (k_is_in_isr()) {
            return -EWOULDBLOCK;
        }

	return ret;
}

static int gpio_aw9523b_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					   gpio_port_value_t value)
{
	int ret;
	return ret;
}

static int gpio_aw9523b_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return gpio_aw9523b_port_set_masked_raw(dev, pins, pins);
}

static int gpio_aw9523b_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return gpio_aw9523b_port_set_masked_raw(dev, pins, 0);
}

static int gpio_aw9523b_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	int ret;
	return ret;
}

static int gpio_aw9523b_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					       enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pin);
	ARG_UNUSED(mode);
	ARG_UNUSED(trig);

	return -ENOTSUP;
}

static int gpio_aw9523b_manage_callback(const struct device *dev, struct gpio_callback *callback,
				       bool set)
{
	struct gpio_aw9523b_data *const data = dev->data;

	return gpio_manage_callback(&data->cb_list_gpio, callback, set);
}

static const struct gpio_driver_api gpio_aw9523b_api = {
	.pin_configure = gpio_aw9523b_configure,
	.port_get_raw = gpio_aw9523b_port_get_raw,
	.port_set_masked_raw = gpio_aw9523b_port_set_masked_raw,
	.port_set_bits_raw = gpio_aw9523b_port_set_bits_raw,
	.port_clear_bits_raw = gpio_aw9523b_port_clear_bits_raw,
	.port_toggle_bits = gpio_aw9523b_port_toggle_bits,
	.pin_interrupt_configure = gpio_aw9523b_pin_interrupt_configure,
        .manage_callback = gpio_aw9523b_manage_callback,

};

static int gpio_aw9523b_init(const struct device *dev)
{
	const struct gpio_aw9523b_config *config = dev->config;
	struct gpio_aw9523b_data *data = dev->data;

	LOG_DBG("aw9523b init.");

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("device not ready, config->i2c.addr: 0x%02x", config->i2c.addr);
		return -ENODEV;
	}

	LOG_DBG("aw9523b init done.");
	return k_mutex_init(&data->mutex);
}


#define GPIO_AW9523B_DEFINE(inst)                                                                  \
	static const struct gpio_aw9523b_config gpio_aw9523b_config##inst = {                      \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),            \
			},                                                                         \
		.i2c = I2C_DT_SPEC_GET(DT_INST_PARENT(inst)),                                      \
		.ngpios = DT_INST_PROP(inst, ngpios),                                              \
	};                                                                                         \
                                                                                                   \
	static struct gpio_aw9523b_data gpio_aw9523b_data##inst;                                   \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, gpio_aw9523b_init, NULL, &gpio_aw9523b_data##inst,             \
			      &gpio_aw9523b_config##inst, POST_KERNEL,                             \
			      CONFIG_GPIO_INIT_PRIORITY, &gpio_aw9523b_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_AW9523B_DEFINE)
