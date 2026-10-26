/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BES_BOOT_WATCHDOG_STOP_H_
#define BES_BOOT_WATCHDOG_STOP_H_

#include <stdint.h>

int boot_wdt_cmsdk_stop(uint32_t id);
int hal_wdt_stop(uint32_t id);
int boot_wdt_stop_status(void);
void boot_wdt_stop_dump(void);

#endif /* BES_BOOT_WATCHDOG_STOP_H_ */
