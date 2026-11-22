/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/dt-bindings/input/ztw623-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#define TOUCH DT_NODELABEL(touch)
#define FLAGS_DOWN 0x03
#define FLAGS_MOVE 0x05
#define FLAGS_UP   0x08

static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(TOUCH, irq_gpios);
static uint8_t point[10];
static bool boot_clear;
static atomic_t read_errors;
static atomic_t ack_error;
static atomic_t stuck_low;

K_MUTEX_DEFINE(model_lock);
K_SEM_DEFINE(restarted, 0, 1);
K_SEM_DEFINE(acknowledged, 0, 1);

struct touch_event {
	int32_t x;
	int32_t y;
	int32_t pressed;
	bool cancelled;
};

K_MSGQ_DEFINE(events, sizeof(struct touch_event), 4, sizeof(uint32_t));

static int model_read(uint16_t reg, struct i2c_msg *message)
{
	uint16_t value;

	if ((message->flags & I2C_MSG_READ) == 0) {
		return -EINVAL;
	}
	switch (reg) {
	case 0x0080:
		if (message->len != sizeof(point)) {
			return -EINVAL;
		}
		if (atomic_get(&read_errors) != 0) {
			atomic_dec(&read_errors);
			return -EIO;
		}
		k_mutex_lock(&model_lock, K_FOREVER);
		memcpy(message->buf, point, sizeof(point));
		k_mutex_unlock(&model_lock);
		return 0;
	case 0x09d2:
		if (message->len != 12) {
			return -EINVAL;
		}
		memset(message->buf, 0, message->len);
		sys_put_le16(0x5601, &message->buf[8]);
		sys_put_le16(0xe623, &message->buf[10]);
		return 0;
	case 0x0014:
		value = 0x5601;
		break;
	case 0x012c:
		value = 0x55aa;
		break;
	case 0xcc00:
		value = 0xe623;
		break;
	default:
		return -EIO;
	}
	if (message->len != 2) {
		return -EINVAL;
	}
	sys_put_le16(value, message->buf);
	return 0;
}

static int model_transfer(const struct emul *target, struct i2c_msg *msgs, int count, int address)
{
	uint16_t reg;

	ARG_UNUSED(target);
	if (address != 0x20 || count < 1 || msgs[0].len < 2 ||
	    (msgs[0].flags & I2C_MSG_READ) != 0) {
		return -EINVAL;
	}
	reg = sys_get_le16(msgs[0].buf);
	if (count == 2 && msgs[0].len == 2) {
		return model_read(reg, &msgs[1]);
	}
	if (count != 1) {
		return -EINVAL;
	}
	switch (reg) {
	case 0x0000:
		boot_clear = true;
		atomic_clear(&stuck_low);
		return 0;
	case 0x0003:
		if (atomic_set(&ack_error, 0) != 0) {
			return -EIO;
		}
		if (atomic_get(&stuck_low) == 0) {
			int ret = gpio_emul_input_set_dt(&irq, 1);

			if (ret != 0) {
				return ret;
			}
		}
		if (boot_clear) {
			boot_clear = false;
			k_sem_give(&restarted);
		} else {
			k_sem_give(&acknowledged);
		}
		return 0;
	case 0x0ff8:
	case 0xc000:
	case 0xc001:
	case 0xc002:
	case 0xc004:
		return 0;
	default:
		return -EIO;
	}
}

static int model_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);
	return 0;
}

static const struct i2c_emul_api i2c_api = {
	.transfer = model_transfer,
};

EMUL_DT_DEFINE(TOUCH, model_init, NULL, NULL, &i2c_api, NULL);

static void record(struct input_event *event, void *user_data)
{
	static struct touch_event position;

	ARG_UNUSED(user_data);
	if (event->type == INPUT_EV_ABS && event->code == INPUT_ABS_X) {
		position.x = event->value;
	} else if (event->type == INPUT_EV_ABS && event->code == INPUT_ABS_Y) {
		position.y = event->value;
	} else if (event->type == INPUT_EV_DEVICE && event->code == INPUT_ZTW623_CANCEL) {
		position.cancelled = true;
	} else if (event->type == INPUT_EV_KEY && event->code == INPUT_BTN_TOUCH && event->sync) {
		position.pressed = event->value;
		zassert_ok(k_msgq_put(&events, &position, K_NO_WAIT));
		position.cancelled = false;
	}
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(TOUCH), record, NULL);

static void send_point(uint8_t flags, uint16_t x, uint16_t y)
{
	k_mutex_lock(&model_lock, K_FOREVER);
	sys_put_le16(x, &point[4]);
	sys_put_le16(y, &point[6]);
	point[9] = flags;
	k_mutex_unlock(&model_lock);
	zassert_ok(gpio_emul_input_set_dt(&irq, 0));
}

static void expect_point(int32_t pressed, int32_t x, int32_t y, bool cancelled)
{
	struct touch_event event;

	zassert_ok(k_msgq_get(&events, &event, K_SECONDS(1)));
	zassert_equal(event.pressed, pressed);
	zassert_equal(event.x, x);
	zassert_equal(event.y, y);
	zassert_equal(event.cancelled, cancelled);
}

ZTEST(ztw623, test_recovery_releases_and_waits_for_new_press)
{
	zassert_true(device_is_ready(DEVICE_DT_GET(TOUCH)));
	zassert_ok(k_sem_take(&restarted, K_NO_WAIT));
	for (unsigned int fault = 0; fault < 3; fault++) {
		send_point(FLAGS_DOWN, 12, 34);
		expect_point(1, 12, 34, false);

		k_sem_reset(&acknowledged);
		if (fault == 0) {
			atomic_set(&read_errors, 2);
		} else if (fault == 1) {
			atomic_set(&ack_error, 1);
		} else {
			atomic_set(&stuck_low, 1);
		}
		send_point(FLAGS_MOVE, 12, 34);
		if (fault == 2) {
			const int64_t deadline = k_uptime_get() + 1000;
			struct touch_event event;

			expect_point(1, 12, 34, false);
			/* The repeated sample has no movement or new edge. Consume its
			 * updates until the guard expires and the driver releases.
			 */
			do {
				const int64_t remaining = deadline - k_uptime_get();

				zassert_true(remaining > 0, "stuck INT did not release the touch");
				zassert_ok(k_msgq_get(&events, &event, K_MSEC(remaining)));
			} while (event.pressed != 0);
			zassert_true(event.cancelled);
		} else {
			expect_point(0, 12, 34, true);
		}
		zassert_ok(k_sem_take(&restarted, K_SECONDS(1)));

		k_sem_reset(&acknowledged);
		send_point(FLAGS_MOVE, 150, 400);
		zassert_ok(k_sem_take(&acknowledged, K_SECONDS(1)));
		struct touch_event event;

		zassert_not_equal(k_msgq_get(&events, &event, K_MSEC(40)), 0);
		send_point(FLAGS_DOWN, 185, 500);
		expect_point(1, 185, 500, false);
		send_point(FLAGS_UP, 185, 500);
		expect_point(0, 185, 500, false);
	}
}

ZTEST_SUITE(ztw623, NULL, NULL, NULL, NULL, NULL);
