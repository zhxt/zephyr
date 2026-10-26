/* SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors */
/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BES2700YP_BOOT_VECTORS_H_
#define BES2700YP_BOOT_VECTORS_H_

#include <stdint.h>

/* Existing loader layout only; not a discovery of the SoC interrupt count. */
#define BOOT_CORE_VECTORS           16U
#define BOOT_VECTOR_COUNT           80U
#define BOOT_VECTOR_ALIGNMENT       1024U
#define BOOT_VECTOR_SELFTEST_CHECKS 6U
#define BOOT_VECTOR_UART_SLOT       (BOOT_CORE_VECTORS + 17U)
#define BOOT_VECTOR_PHASE_COUNT     3U

struct boot_vectors_report {
	uint32_t fail_mask;
	uint32_t vtor;
	uint32_t first_slot;
	uintptr_t expected;
	uintptr_t actual;
};

struct boot_vectors_state {
	struct boot_vectors_report points[BOOT_VECTOR_PHASE_COUNT];
	uintptr_t uart_handler;
};

extern uint32_t __StackTop;
extern volatile uintptr_t __vector_table_start[];

void NVIC_InitVectors(void);
int NVIC_GetCurrentActiveIRQ(void);
void Reset_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SecureFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

unsigned int boot_vectors_check(void);
void boot_vectors_capture(struct boot_vectors_report *report, uintptr_t uart_handler);
void boot_vectors_before_uart(struct boot_vectors_state *state);
void boot_vectors_after_uart(struct boot_vectors_state *state);
int boot_vectors_selftest(struct boot_vectors_state *state);

#endif
