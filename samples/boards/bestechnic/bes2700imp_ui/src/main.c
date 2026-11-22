/* SPDX-License-Identifier: Apache-2.0 */

#include <lvgl.h>
#include <lvgl_zephyr.h>
#include <stdlib.h>
#include <zephyr/drivers/display.h>
#include <zephyr/dt-bindings/input/ztw623-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define DISPLAY_NODE DT_CHOSEN(zephyr_display)
#define TOUCH_NODE   DT_NODELABEL(touch)
#define WIDTH        DT_PROP(DISPLAY_NODE, width)
#define HEIGHT       DT_PROP(DISPLAY_NODE, height)
#define TILE_ROWS    26
#define TARGET_SIZE  32
#define MARKER_SIZE  8
#define DRAG_PX      12
#define IDLE_MS      5000

BUILD_ASSERT(HEIGHT % TILE_ROWS == 0);
BUILD_ASSERT(CONFIG_LV_Z_VDB_SIZE * HEIGHT == 100 * TILE_ROWS);

static const struct device *const display = DEVICE_DT_GET(DISPLAY_NODE);
static const int target_x[] = {28, 106, 184};
static const int target_y[] = {48, 260, 472};
static lv_obj_t *targets[ARRAY_SIZE(target_x) * ARRAY_SIZE(target_y)];
static lv_obj_t *marker;
static int first_frame_error;
static bool first_frame_complete;

struct touch_frame {
	lv_point_t point;
	uint32_t uptime;
	bool pressed;
	bool cancelled;
};

struct interaction {
	lv_display_t *surface;
	int64_t idle_deadline;
	uint32_t wake_start;
	uint32_t last_wake;
	lv_point_t down;
	unsigned int distance;
	bool blanked;
	bool swallow;
	bool pressed;
};

K_MSGQ_DEFINE(touch_frames, sizeof(struct touch_frame), 64, sizeof(uint32_t));

static void touch_event(struct input_event *event, void *user)
{
	static struct touch_frame frame;

	ARG_UNUSED(user);
	if (event->type == INPUT_EV_ABS) {
		if (event->code == INPUT_ABS_X) {
			frame.point.x = event->value;
		} else if (event->code == INPUT_ABS_Y) {
			frame.point.y = event->value;
		}
	} else if (event->type == INPUT_EV_DEVICE && event->code == INPUT_ZTW623_CANCEL) {
		frame.cancelled = true;
	} else if (event->type == INPUT_EV_KEY && event->code == INPUT_BTN_TOUCH && event->sync) {
		frame.pressed = event->value != 0;
		frame.uptime = k_uptime_get_32();
		if (k_msgq_put(&touch_frames, &frame, K_NO_WAIT) != 0) {
			/* Drop stale frames and cancel with the latest contact state. */
			k_msgq_purge(&touch_frames);
			frame.cancelled = true;
			/* This input callback is the sole producer. */
			(void)k_msgq_put(&touch_frames, &frame, K_NO_WAIT);
		}
		frame.cancelled = false;
	}
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(TOUCH_NODE), touch_event, NULL);

static void round_dirty_area(lv_event_t *event)
{
	lv_area_t *area = lv_event_get_invalidated_area(event);

	/* Keep partial updates on the verified full-width LCDC tiles. */
	area->x1 = 0;
	area->x2 = WIDTH - 1;
	area->y1 = area->y1 / TILE_ROWS * TILE_ROWS;
	area->y2 = (area->y2 / TILE_ROWS + 1) * TILE_ROWS - 1;
}

static lv_obj_t *square(lv_obj_t *screen, int x, int y, int size, uint32_t rgb)
{
	lv_obj_t *obj = lv_obj_create(screen);

	lv_obj_remove_style_all(obj);
	lv_obj_set_scrollable(obj, false);
	lv_obj_set_size(obj, size, size);
	lv_obj_set_pos(obj, x, y);
	lv_obj_set_style_bg_color(obj, lv_color_hex(rgb), 0);
	lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
	return obj;
}

static void flush_first_frame(lv_display_t *surface, const lv_area_t *area, uint8_t *pixels)
{
	if (first_frame_error == 0) {
		uint32_t stride = lv_display_get_buf_active(surface)->header.stride;
		uint16_t height = lv_area_get_height(area);
		const struct display_buffer_descriptor desc = {
			.buf_size = stride * height,
			.width = lv_area_get_width(area),
			.height = height,
			.pitch = stride / sizeof(uint16_t),
			.frame_incomplete = !lv_display_flush_is_last(surface),
		};

		first_frame_error = display_write(display, area->x1, area->y1, &desc, pixels);
		first_frame_complete = first_frame_error == 0 && !desc.frame_incomplete;
	}
	lv_display_flush_ready(surface);
}

static int draw_targets(lv_display_t *surface)
{
	lv_obj_t *screen = lv_display_get_screen_active(surface);
	lv_display_flush_cb_t flush = lv_display_get_flush_cb(surface);
	unsigned int count = 0;

	lv_display_add_event_cb(surface, round_dirty_area, LV_EVENT_INVALIDATE_AREA, NULL);
	lv_obj_remove_style_all(screen);
	lv_obj_set_scrollable(screen, false);
	lv_obj_set_style_bg_color(screen, lv_color_hex(0x0a1220), 0);
	lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
	for (unsigned int row = 0; row < ARRAY_SIZE(target_y); row++) {
		for (unsigned int col = 0; col < ARRAY_SIZE(target_x); col++) {
			targets[count++] = square(screen, target_x[col] - TARGET_SIZE / 2,
						  target_y[row] - TARGET_SIZE / 2,
						  TARGET_SIZE, 0x646dda);
		}
	}
	marker = square(screen, 0, 0, MARKER_SIZE, 0xffffff);
	lv_obj_set_hidden(marker, true);

	first_frame_error = 0;
	first_frame_complete = false;
	lv_display_set_flush_cb(surface, flush_first_frame);
	lv_refr_now(surface);
	lv_display_set_flush_cb(surface, flush);

	if (first_frame_error != 0) {
		return first_frame_error;
	}
	return first_frame_complete ? 0 : -EIO;
}

static void mark_target(const struct interaction *ui, const struct touch_frame *frame)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(targets); i++) {
		int x = target_x[i % ARRAY_SIZE(target_x)];
		int y = target_y[i / ARRAY_SIZE(target_x)];

		if (abs(ui->down.x - x) < TARGET_SIZE / 2 &&
		    abs(ui->down.y - y) < TARGET_SIZE / 2) {
			uint32_t wake_gap =
				ui->last_wake != 0U ? frame->uptime - ui->last_wake : 0U;

			lv_obj_set_style_bg_color(targets[i], lv_color_hex(0x28caec), 0);
			printk("BES2700IMP_UI HIT target=%u wake_gap_ms=%u\n", i + 1, wake_gap);
			return;
		}
	}
}

static int handle_touch(struct interaction *ui, const struct touch_frame *frame)
{
	if (frame->cancelled) {
		ui->pressed = false;
		ui->swallow = frame->pressed;
		ui->wake_start = 0;
		ui->last_wake = 0;
		ui->idle_deadline = k_uptime_get() + IDLE_MS;
		lv_obj_set_hidden(marker, true);
		lv_refr_now(ui->surface);
		printk("BES2700IMP_UI CANCEL\n");
		return 0;
	}
	if (ui->swallow) {
		if (!frame->pressed) {
			ui->swallow = false;
			ui->idle_deadline = k_uptime_get() + IDLE_MS;
			if (ui->wake_start != 0U) {
				printk("BES2700IMP_UI WAKE_RELEASE held_ms=%u\n",
				       frame->uptime - ui->wake_start);
				ui->last_wake = frame->uptime;
				ui->wake_start = 0;
			}
		}
		return 0;
	}
	if (ui->blanked) {
		if (frame->pressed) {
			int ret = display_blanking_off(display);

			if (ret != 0) {
				return ret;
			}
			ui->blanked = false;
			ui->swallow = true;
			ui->wake_start = frame->uptime;
			printk("BES2700IMP_UI WAKE\n");
		}
		return 0;
	}
	if (frame->pressed) {
		if (!ui->pressed) {
			ui->down = frame->point;
			ui->distance = 0;
		}
		ui->distance = MAX(ui->distance, (unsigned int)abs(frame->point.x - ui->down.x));
		ui->distance = MAX(ui->distance, (unsigned int)abs(frame->point.y - ui->down.y));
		lv_obj_set_pos(marker,
			       CLAMP(frame->point.x - MARKER_SIZE / 2, 0, WIDTH - MARKER_SIZE),
			       CLAMP(frame->point.y - MARKER_SIZE / 2, 0, HEIGHT - MARKER_SIZE));
		lv_obj_set_hidden(marker, false);
	} else if (ui->pressed) {
		if (ui->distance < DRAG_PX) {
			mark_target(ui, frame);
		} else {
			printk("BES2700IMP_UI DRAG distance=%u\n", ui->distance);
		}
		ui->idle_deadline = k_uptime_get() + IDLE_MS;
		lv_obj_set_hidden(marker, true);
	}
	ui->pressed = frame->pressed;
	lv_refr_now(ui->surface);
	return 0;
}

static int run_interaction(lv_display_t *surface)
{
	struct interaction ui = { .surface = surface, .idle_deadline = k_uptime_get() + IDLE_MS };
	struct touch_frame frame;
	int ret;

	for (;;) {
		k_timeout_t timeout = K_FOREVER;

		if (!ui.blanked && !ui.swallow && !ui.pressed) {
			timeout = K_MSEC(MAX(ui.idle_deadline - k_uptime_get(), 0));
		}
		ret = k_msgq_get(&touch_frames, &frame, timeout);
		if (ret == 0) {
			ret = handle_touch(&ui, &frame);
		} else if (ret == -ENOMSG && !K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
			/* A purge may wake this thread before the cancel frame arrives. */
			continue;
		} else {
			ret = display_blanking_on(display);
			if (ret == 0) {
				ui.blanked = true;
				printk("BES2700IMP_UI BLANK\n");
			}
		}
		if (ret != 0) {
			return ret;
		}
	}
}

int main(void)
{
	int ret;

	if (!device_is_ready(display) || !device_is_ready(DEVICE_DT_GET(TOUCH_NODE))) {
		ret = -ENODEV;
	} else {
		ret = lvgl_init();
	}
	if (ret == 0) {
		lv_display_t *surface = lv_display_get_default();

		ret = draw_targets(surface);
		if (ret == 0) {
			ret = display_blanking_off(display);
		}
		if (ret == 0) {
			printk("BES2700IMP_UI READY width=%u height=%u targets=%u\n",
			       WIDTH, HEIGHT, (unsigned int)ARRAY_SIZE(targets));
			ret = run_interaction(surface);
		}
	}
	printk("BES2700IMP_UI ERROR rc=%d\n", ret);
	return ret;
}
