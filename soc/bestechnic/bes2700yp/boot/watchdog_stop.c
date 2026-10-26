/* SPDX-License-Identifier: Apache-2.0 */

#include "watchdog_stop.h"
#ifdef BOOT_WDT_STOP_HOST
#include "tests/watchdog_stop_host.h"
#else
#include "arch.h"
#include "boot_contract.h"
#define WDT_READ(address) (*(__IM uint32_t *)(uintptr_t)(address))
#define WDT_WRITE(address, value) \
	(*(__IO uint32_t *)(uintptr_t)(address) = (value))
#endif

/* CMSDK DDI0479D 4.5.1 defines the register protocol. The physical address
 * and accessible startup phase are local BES experimental integration facts.
 * No clock, reset, LOAD, INTCLR or PMU registers are changed here.
 */
#define WDT_CONTROL 0x40082008U
#define WDT_LOCK 0x40082c00U
#define WDT_CONTROL_MASK 3U
#define WDT_UNLOCK 0x1acce551U
#define WDT_RELOCK 1U

static int last_result = -11;
static uint32_t unlock_readback;
static uint32_t control_readback;
static uint32_t lock_readback;
static uint32_t masks_restored;

__attribute__((noinline))
int boot_wdt_cmsdk_stop(uint32_t id)
{
	uint32_t primask;
	uint32_t basepri;
	uint32_t control;
	uint32_t lock;
	int result = -5;

	last_result = -22;
	if (id != 0U) {
		return last_result;
	}
	primask = __get_PRIMASK();
	basepri = __get_BASEPRI();
	__disable_irq();
	__DSB();
	control = WDT_READ(WDT_CONTROL);
	lock = WDT_READ(WDT_LOCK);
	if ((control & ~WDT_CONTROL_MASK) != 0U || (lock & ~1U) != 0U) {
		result = -95;
		goto restore;
	}
	WDT_WRITE(WDT_LOCK, WDT_UNLOCK);
	__DSB();
	unlock_readback = WDT_READ(WDT_LOCK);
	if (unlock_readback == 0U) {
		WDT_WRITE(WDT_CONTROL, 0U);
		__DSB();
		control_readback = WDT_READ(WDT_CONTROL);
		if (control_readback == 0U) {
			result = 0;
		}
	}
	/* DDI0479D permits any non-unlock value to disable register writes. */
	WDT_WRITE(WDT_LOCK, WDT_RELOCK);
	__DSB();
	lock_readback = WDT_READ(WDT_LOCK);
	if (lock_readback != 1U) {
		result = -5;
	}
restore:
	__set_PRIMASK(primask);
	__ISB();
	masks_restored = (__get_PRIMASK() == primask &&
			  __get_BASEPRI() == basepri);
	last_result = masks_restored ? result : -5;
	return last_result;
}

/* A fixed-instance ABI bridge for the unchanged local source wrapper. */
int hal_wdt_stop(uint32_t id)
{
	return boot_wdt_cmsdk_stop(id);
}

int boot_wdt_stop_status(void)
{
	return last_result;
}

void boot_wdt_stop_dump(void)
{
	bth_puts("zephyr_bth watchdog_stop version=1 provider=cmsdk-stop");
	bth_field(" unlock=", unlock_readback);
	bth_field(" ctrl=", control_readback);
	bth_field(" lock=", lock_readback);
	bth_field(" restore=", masks_restored);
	bth_end();
}
