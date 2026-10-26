/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BES_BOOT_CMU_RELEASE_NATIVE_H_
#define BES_BOOT_CMU_RELEASE_NATIVE_H_

#include <stdint.h>

__attribute__((noreturn)) void hal_cmu_boot_start_bth_cpu(uint32_t stack, uint32_t entry);
__attribute__((noreturn)) void hal_cmu_boot_m55_start_bth(void);
__attribute__((noreturn)) void boot_loader_entry_hook(void);
void hal_cmu_bth_ram_cfg(void);
void boot_cmu_release_sync(void);
__attribute__((noreturn)) void boot_cmu_release_wait(void);
extern uint32_t __StackTop;

#endif /* BES_BOOT_CMU_RELEASE_NATIVE_H_ */
