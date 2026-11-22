/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zinitix_ztw623

#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/dt-bindings/input/ztw623-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_touch.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(ztw623, CONFIG_INPUT_LOG_LEVEL);

#define ZTW623_CMD_SOFT_RESET     0x0000
#define ZTW623_CMD_CLEAR_INT      0x0003
#define ZTW623_REG_POINT          0x0080
#define ZTW623_REG_IDENTITY       0x09d2
#define ZTW623_CMD_IUM_UNLOCK     0x0ff8
#define ZTW623_REG_CHECKSUM       0x012c
#define ZTW623_REG_HW_ID          0x0014
#define ZTW623_REG_VENDOR_COMMAND 0xc000
#define ZTW623_REG_PROGRAM_START  0xc001
#define ZTW623_REG_NVM_INIT       0xc002
#define ZTW623_CMD_VENDOR_CLEAR   0xc004
#define ZTW623_REG_CHIP_CODE      0xcc00
#define ZTW623_CHIP_CODE          0xe623
#define ZTW623_FW_CHECKSUM_OK     0x55aa
#define ZTW623_POINT_SIZE         10
#define ZTW623_PALM               BIT(4)
#define ZTW623_CONTACT            BIT(0)
#define ZTW623_DOWN               BIT(1)
#define ZTW623_MOVE               BIT(2)
#define ZTW623_UP                 BIT(3)
#define ZTW623_FALLBACK_MS        100
#define ZTW623_STUCK_MS           250

struct ztw623_config {
	struct input_touchscreen_common_config common;
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec irq;
	struct gpio_dt_spec reset;
};

INPUT_TOUCH_STRUCT_CHECK(struct ztw623_config);

struct ztw623_data {
	struct gpio_callback callback;
	struct k_sem ready;
	struct k_thread thread;

	K_KERNEL_STACK_MEMBER(stack, CONFIG_INPUT_ZTW623_THREAD_STACK_SIZE);
	bool pressed;
	bool await_down;
	bool reset_used;
};

static int ztw623_read(const struct ztw623_config *cfg, uint16_t reg, void *value, size_t len)
{
	uint8_t address[2];

	sys_put_le16(reg, address);
	return i2c_write_read_dt(&cfg->i2c, address, sizeof(address), value, len);
}

static int ztw623_read_u16(const struct ztw623_config *cfg, uint16_t reg, uint16_t *value)
{
	uint8_t bytes[2];
	int ret = ztw623_read(cfg, reg, bytes, sizeof(bytes));

	if (ret == 0) {
		*value = sys_get_le16(bytes);
	}

	return ret;
}

static int ztw623_write(const struct ztw623_config *cfg, uint16_t reg, uint16_t value,
			bool command)
{
	uint8_t bytes[4];
	int ret;

	sys_put_le16(reg, bytes);
	sys_put_le16(value, &bytes[2]);
	ret = i2c_write_dt(&cfg->i2c, bytes, command ? 2 : sizeof(bytes));
	if (ret == 0 && command) {
		k_busy_wait(10);
	}

	return ret;
}

static int ztw623_start(const struct device *dev)
{
	const struct ztw623_config *cfg = dev->config;
	const struct {
		uint16_t reg;
		uint8_t delay_ms;
		bool command;
	} sequence[] = {
		{ZTW623_REG_NVM_INIT, 2, false},
		{ZTW623_REG_PROGRAM_START, 50, false},
		{ZTW623_CMD_SOFT_RESET, 10, true},
	};
	uint8_t identity[12];
	uint16_t chip, hw_id, checksum;
	int ret;

	k_msleep(20);
	ret = gpio_pin_set_dt(&cfg->reset, 1);
	if (ret != 0) {
		return ret;
	}
	k_msleep(100);
	ret = gpio_pin_set_dt(&cfg->reset, 0);
	if (ret != 0) {
		return ret;
	}
	k_msleep(50);

	ret = ztw623_write(cfg, ZTW623_REG_VENDOR_COMMAND, 1, false);
	if (ret == 0) {
		ret = ztw623_read(cfg, ZTW623_REG_IDENTITY, identity, sizeof(identity));
	}
	if (ret != 0) {
		return ret;
	}
	if (sys_get_le16(&identity[10]) != ZTW623_CHIP_CODE) {
		return -ENODEV;
	}

	ret = ztw623_write(cfg, ZTW623_CMD_IUM_UNLOCK, 0, true);
	if (ret != 0) {
		return ret;
	}
	k_msleep(50);
	ret = ztw623_write(cfg, ZTW623_REG_VENDOR_COMMAND, 1, false);
	if (ret != 0) {
		return ret;
	}
	k_msleep(5);
	ret = ztw623_read_u16(cfg, ZTW623_REG_CHIP_CODE, &chip);
	if (ret != 0) {
		return ret;
	}
	if (chip != ZTW623_CHIP_CODE) {
		return -ENODEV;
	}
	k_msleep(1);
	for (int attempt = 0; attempt < 3; attempt++) {
		ret = ztw623_write(cfg, ZTW623_CMD_VENDOR_CLEAR, 0, true);
		if (ret == 0) {
			break;
		}
		k_msleep(2);
	}
	if (ret != 0) {
		return ret;
	}
	for (size_t i = 0; i < ARRAY_SIZE(sequence); i++) {
		ret = ztw623_write(cfg, sequence[i].reg, 1, sequence[i].command);
		if (ret != 0) {
			return ret;
		}
		k_msleep(sequence[i].delay_ms);
	}

	ret = ztw623_read_u16(cfg, ZTW623_REG_HW_ID, &hw_id);
	if (ret == 0) {
		ret = ztw623_read_u16(cfg, ZTW623_REG_CHECKSUM, &checksum);
	}
	if (ret != 0) {
		return ret;
	}
	if (hw_id != sys_get_le16(&identity[8]) || checksum != ZTW623_FW_CHECKSUM_OK) {
		return -ENODEV;
	}
	LOG_INF("chip=%04x hw=%04x", chip, hw_id);
	return ztw623_write(cfg, ZTW623_CMD_CLEAR_INT, 0, true);
}

static int ztw623_release(const struct device *dev)
{
	struct ztw623_data *data = dev->data;
	int ret = 0;

	if (data->pressed) {
		ret = input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
		if (ret == 0) {
			data->pressed = false;
		}
	}

	return ret;
}

static int ztw623_cancel(const struct device *dev)
{
	struct ztw623_data *data = dev->data;
	int ret;
	int release;

	if (!data->pressed) {
		return 0;
	}
	ret = input_report(dev, INPUT_EV_DEVICE, INPUT_ZTW623_CANCEL, 1, false, K_FOREVER);
	release = ztw623_release(dev);
	return ret != 0 ? ret : release;
}

static int ztw623_report(const struct device *dev, const uint8_t *point)
{
	const struct ztw623_config *cfg = dev->config;
	struct ztw623_data *data = dev->data;
	uint8_t flags = point[9];
	bool down = (flags & (ZTW623_CONTACT | ZTW623_DOWN)) == (ZTW623_CONTACT | ZTW623_DOWN);
	uint16_t x, y;
	int ret;

	if ((sys_get_le16(point) & ZTW623_PALM) != 0U) {
		data->await_down = true;
		return ztw623_cancel(dev);
	}
	if (data->await_down && !down) {
		return 0;
	}
	if ((flags & ZTW623_CONTACT) == 0U) {
		return (flags & ZTW623_UP) != 0U ? ztw623_release(dev) : 0;
	}
	if (!down && (flags & ZTW623_MOVE) != 0U && !data->pressed) {
		return -EPROTO;
	}
	if (data->pressed && (down || (flags & ZTW623_MOVE) == 0U)) {
		return 0;
	}

	x = sys_get_le16(&point[4]);
	y = sys_get_le16(&point[6]);
	if ((cfg->common.screen_width != 0U && x >= cfg->common.screen_width) ||
	    (cfg->common.screen_height != 0U && y >= cfg->common.screen_height)) {
		return -ERANGE;
	}
	input_touchscreen_report_pos(dev, x, y, K_FOREVER);
	ret = input_report_key(dev, INPUT_BTN_TOUCH, 1, true, K_FOREVER);
	if (ret == 0) {
		data->pressed = true;
		data->await_down = false;
		data->reset_used = false;
	}
	return ret;
}

static int ztw623_recover(const struct device *dev, int error)
{
	const struct ztw623_config *cfg = dev->config;
	struct ztw623_data *data = dev->data;
	int ret = ztw623_cancel(dev);

	data->await_down = true;
	LOG_WRN("touch cancelled: %d", error);
	if (ret != 0 || data->reset_used) {
		return ret != 0 ? ret : error;
	}

	data->reset_used = true;
	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_DISABLE);
	if (ret != 0) {
		return ret;
	}
	k_sem_reset(&data->ready);
	ret = ztw623_start(dev);
	if (ret != 0) {
		return ret;
	}
	return gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
}

static void ztw623_thread(void *arg, void *unused1, void *unused2)
{
	const struct device *dev = arg;
	const struct ztw623_config *cfg = dev->config;
	struct ztw623_data *data = dev->data;
	uint8_t last[ZTW623_POINT_SIZE] = {0};
	bool draining = false;
	int64_t progress = 0;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	for (;;) {
		int active = gpio_pin_get_dt(&cfg->irq);
		bool notified = k_sem_take(&data->ready, active == 0 ? K_MSEC(ZTW623_FALLBACK_MS) :
					  K_NO_WAIT) == 0;
		uint8_t point[ZTW623_POINT_SIZE];
		int ret;

		active = gpio_pin_get_dt(&cfg->irq);
		if (active == 0) {
			draining = false;
			continue;
		}
		ret = active < 0 ? active :
				       ztw623_read(cfg, ZTW623_REG_POINT, point, sizeof(point));
		if (active > 0 && ret != 0) {
			k_msleep(5);
			ret = ztw623_read(cfg, ZTW623_REG_POINT, point, sizeof(point));
		}
		if (ret == 0) {
			ret = ztw623_write(cfg, ZTW623_CMD_CLEAR_INT, 0, true);
		}
		if (ret == 0) {
			int64_t now = k_uptime_get();

			LOG_DBG("sample source=%s", notified ? "irq" : "level");
			if (!draining || memcmp(last, point, sizeof(point)) != 0) {
				progress = now;
			}
			memcpy(last, point, sizeof(last));
			active = gpio_pin_get_dt(&cfg->irq);
			draining = active > 0;
			if (active < 0) {
				ret = active;
			} else if (draining && now - progress >= ZTW623_STUCK_MS) {
				ret = -ETIMEDOUT;
			} else {
				ret = ztw623_report(dev, point);
			}
		}
		if (ret != 0) {
			ret = ztw623_recover(dev, ret);
			if (ret != 0) {
				int disable = gpio_pin_interrupt_configure_dt(
					&cfg->irq, GPIO_INT_DISABLE);

				LOG_ERR("touch stopped: %d, IRQ disable: %d", ret, disable);
				return;
			}
			draining = false;
		}
		if (draining) {
			k_msleep(1);
		}
	}
}

static void ztw623_gpio_callback(const struct device *port, struct gpio_callback *callback,
				 gpio_port_pins_t pins)
{
	struct ztw623_data *data = CONTAINER_OF(callback, struct ztw623_data, callback);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	k_sem_give(&data->ready);
}

static int ztw623_init(const struct device *dev)
{
	const struct ztw623_config *cfg = dev->config;
	struct ztw623_data *data = dev->data;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c) || !gpio_is_ready_dt(&cfg->irq) ||
	    !gpio_is_ready_dt(&cfg->reset)) {
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
	if (ret == 0) {
		ret = gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT);
	}
	if (ret == 0) {
		ret = ztw623_start(dev);
	}
	if (ret != 0) {
		return ret;
	}

	k_sem_init(&data->ready, 0, 1);
	gpio_init_callback(&data->callback, ztw623_gpio_callback, BIT(cfg->irq.pin));
	ret = gpio_add_callback_dt(&cfg->irq, &data->callback);
	if (ret != 0) {
		return ret;
	}
	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		(void)gpio_remove_callback_dt(&cfg->irq, &data->callback);
		return ret;
	}
	k_thread_create(&data->thread, data->stack, K_KERNEL_STACK_SIZEOF(data->stack),
			ztw623_thread, (void *)dev, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0,
			K_NO_WAIT);

	return 0;
}

#define ZTW623_DEFINE(inst)                                                                  \
	static const struct ztw623_config ztw623_cfg_##inst = {                              \
		.common = INPUT_TOUCH_DT_INST_COMMON_CONFIG_INIT(inst),                      \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                           \
		.irq = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                               \
		.reset = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                           \
	};                                                                                   \
	static struct ztw623_data ztw623_data_##inst;                                        \
	DEVICE_DT_INST_DEFINE(inst, ztw623_init, NULL, &ztw623_data_##inst, &ztw623_cfg_##inst, \
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ZTW623_DEFINE)
