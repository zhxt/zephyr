/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include "image_config.h"

/* Local BEST1503 UART0 wiring, checked against the working pre-BootInit console. */
#define CMU 0x40000000U
#define IOMUX 0x40086000U
#define UART 0x4000b000U
#define UART_PCLK (1U << 10)
#define UART_OCLK (1U << 12)
#define UART_ENABLED (1U << 0)
#define UART_TRANSMIT (1U << 8)
#define UART_RECEIVE (1U << 9)
#define UART_BUSY (1U << 3)
#define UART_TX_FULL (1U << 5)
#define UART_POLL_LIMIT 1000000U

#define CPUID 0xe000ed00U
#define ICSR 0xe000ed04U
#define VTOR 0xe000ed08U
#define CFSR 0xe000ed28U
#define HFSR 0xe000ed2cU
#define ICTR 0xe000e004U
#define NVIC_ICER 0xe000e180U
#define NVIC_ICPR 0xe000e280U
#define SYSTICK_CTRL 0xe000e010U
#define SYSTICK_LOAD 0xe000e014U
#define SYSTICK_VAL 0xe000e018U
#define MPU_CTRL 0xe000ed94U

#define VECTOR_SLOTS 256U
#define FNV_OFFSET_BASIS 2166136261U
#define FNV_PRIME 16777619U
#define REG(addr) (*(volatile uint32_t *)(addr))
#define NATIVE "BES2700IMP_ROM "

extern uint32_t __fault_vectors[], __stack_top[];
extern const uint8_t __ramtext_load[];
extern uint8_t __ramtext_start[], __ramtext_end[];
extern const uint8_t payload_start[], payload_end[];
__attribute__((noreturn)) void bth_enter_zephyr(uint32_t vtor, uint32_t msp, uint32_t entry);

static const char build_info[] __attribute__((used, section(".build_info"))) =
	"CHIP=best1503\nSW_TYPE=ZEPHYR_BES2700IMP\n";

static void sync_cpu(void)
{
	__asm__ volatile("dsb sy\n isb sy" ::: "memory");
}

static int wait_uart(uint32_t mask)
{
	for (unsigned int left = UART_POLL_LIMIT; left != 0U; left--) {
		if ((REG(UART + 0x18U) & mask) == 0U) {
			return 1;
		}
	}
	return 0;
}

static void print(const char *text)
{
	for (; *text != '\0' && wait_uart(UART_TX_FULL); text++) {
		REG(UART) = (uint8_t)*text;
	}
}

static void hex(const char *field, uint32_t value)
{
	static const char digits[] = "0123456789abcdef";
	char text[9];

	for (unsigned int i = 0; i < 8; i++) {
		text[i] = digits[(value >> (28U - i * 4U)) & 0xfU];
	}
	text[8] = '\0';
	print(field);
	print(text);
}

__attribute__((noreturn)) static void stop(void)
{
	(void)wait_uart(UART_BUSY);
	for (;;) {
		__asm__ volatile("wfi");
	}
}

__attribute__((noreturn)) static void fault(void)
{
	__asm__ volatile("cpsid i");
	print("\r\n" NATIVE "ERROR exception");
	hex(" cfsr=", REG(CFSR));
	hex(" hfsr=", REG(HFSR));
	print("\r\n");
	stop();
}

static void init_vectors(void)
{
	volatile uint32_t *table = __fault_vectors;

	table[0] = (uintptr_t)__stack_top;
	for (unsigned int slot = 1; slot < VECTOR_SLOTS; slot++) {
		table[slot] = (uintptr_t)fault;
	}
	REG(VTOR) = (uintptr_t)table;
	sync_cpu();
}

static void init_uart(void)
{
	REG(CMU + 0x80U) = 0xcafe0001U;
	REG(CMU + 0x08U) = UART_PCLK;
	REG(CMU + 0x10U) = UART_OCLK;
	REG(CMU + 0x40U) = UART_PCLK;
	REG(CMU + 0x48U) = UART_OCLK;
	sync_cpu();
	if ((REG(UART + 0x30U) & UART_ENABLED) != 0U && !wait_uart(UART_BUSY)) {
		stop();
	}
	REG(UART + 0x30U) = 0U;
	REG(CMU + 0x70U) &= ~0x1fU;
	REG(IOMUX + 0x50U) |= 3U;
	REG(IOMUX + 0x0cU) = (REG(IOMUX + 0x0cU) & ~0xff00U) | 0x2200U;
	REG(IOMUX + 0x2cU) = (REG(IOMUX + 0x2cU) & ~(3U << 18)) | (1U << 18);
	REG(IOMUX + 0x30U) &= ~(3U << 18);

	/* UART0 uses the oscillator directly, with 16x oversampling and 8N1. */
	REG(UART + 0x38U) = 0U;
	REG(UART + 0x44U) = 0x7ffU;
	REG(UART + 0x48U) = 0U;
	REG(UART + 0x4cU) = 0U;
	REG(UART + 0x50U) = 0U;
	REG(UART + 0x24U) = 1U;
	REG(UART + 0x28U) = 40U;
	REG(UART + 0x2cU) = 0x70U;
	REG(UART + 0x30U) = UART_ENABLED | UART_TRANSMIT | UART_RECEIVE;
	sync_cpu();
}

static void reset_interrupt_state(void)
{
	const unsigned int groups = (REG(ICTR) & 0xfU) + 1U;

	REG(SYSTICK_CTRL) = 0U;
	REG(SYSTICK_LOAD) = 0U;
	REG(SYSTICK_VAL) = 0U;
	for (unsigned int group = 0; group < groups; group++) {
		REG(NVIC_ICER + group * 4U) = UINT32_MAX;
		REG(NVIC_ICPR + group * 4U) = UINT32_MAX;
	}
	REG(ICSR) = (1U << 25) | (1U << 27); /* Clear pending SysTick and PendSV. */
	REG(MPU_CTRL) = 0U;
	sync_cpu();
}

typedef uint32_t payload_word __attribute__((__may_alias__));

__attribute__((noinline, noclone, section(".ramtext")))
static void copy_payload(volatile uint8_t *dest, const uint8_t *src, uint32_t size)
{
	uint32_t copied = 0;

	if ((((uintptr_t)dest | (uintptr_t)src) & (sizeof(payload_word) - 1U)) == 0U) {
		volatile payload_word *words = (volatile payload_word *)dest;
		const payload_word *input = (const payload_word *)src;
		uint32_t count = size / sizeof(payload_word);

		for (uint32_t i = 0; i < count; i++) {
			words[i] = input[i];
		}
		copied = count * sizeof(payload_word);
	}

	for (; copied < size; copied++) {
		dest[copied] = src[copied];
	}
}

__attribute__((noinline, noclone, section(".ramtext")))
static uint32_t payload_digest(const volatile uint8_t *ram, uint32_t size)
{
	uint32_t digest = FNV_OFFSET_BASIS;
	uint32_t checked = 0;

	if (((uintptr_t)ram & (sizeof(payload_word) - 1U)) == 0U) {
		const volatile payload_word *words = (const volatile payload_word *)ram;
		uint32_t count = size / sizeof(payload_word);

		/* ROM words are little endian. Feed FNV-1a in address order. */
		for (uint32_t i = 0; i < count; i++) {
			uint32_t word = words[i];

			digest = (digest ^ (word & 0xffU)) * FNV_PRIME;
			digest = (digest ^ ((word >> 8) & 0xffU)) * FNV_PRIME;
			digest = (digest ^ ((word >> 16) & 0xffU)) * FNV_PRIME;
			digest = (digest ^ (word >> 24)) * FNV_PRIME;
		}
		checked = count * sizeof(payload_word);
	}

	for (; checked < size; checked++) {
		digest = (digest ^ ram[checked]) * FNV_PRIME;
	}
	return digest;
}

#if CORE_CLOCK_HZ == 48000000U
#define CMU_SYS_CLOCK      (CMU + 0x60U)
#define SYS_CPU_SELECTORS  (0xffU << 5)
#define SYS_XTAL_SELECT    (3U << 8)
#define SYS_FAST_SELECT    (1U << 10)
#define SYS_OSC2_POWER     (1U << 14)
#define SYS_OSC2_ENABLE    (1U << 13)

static void select_osc2(void)
{
	uint32_t before = REG(CMU_SYS_CLOCK);
	uint32_t after;

	if ((before & SYS_CPU_SELECTORS) != SYS_XTAL_SELECT) {
		print(NATIVE "ERROR unexpected ROM system clock\r\n");
		stop();
	}
	REG(CMU_SYS_CLOCK) = SYS_OSC2_POWER;
	/* Keep the 24 MHz source for at least 100 us while OSCx2 starts. */
	for (unsigned int i = 0; i < 2400U; i++) {
		__asm__ volatile("nop");
	}
	REG(CMU_SYS_CLOCK) = SYS_OSC2_ENABLE;
	sync_cpu();
	REG(CMU_SYS_CLOCK) = SYS_FAST_SELECT;
	sync_cpu();
	after = REG(CMU_SYS_CLOCK);
	if (after != (before | SYS_OSC2_POWER | SYS_OSC2_ENABLE | SYS_FAST_SELECT)) {
		print(NATIVE "ERROR system clock switch readback\r\n");
		stop();
	}
}
#endif

__attribute__((noreturn)) void bes2700imp_rom_main(void)
{
	volatile uint8_t *const ram = (volatile uint8_t *)SRAM_BASE;
	volatile const uint32_t *const vectors = (volatile const uint32_t *)SRAM_BASE;
	const uint32_t size = (uintptr_t)payload_end - (uintptr_t)payload_start;
	uint8_t *code = __ramtext_start;
	const uint8_t *code_image = __ramtext_load;
	uint32_t msp, ipsr, digest;

	init_vectors();
	init_uart();
	print("\r\n" NATIVE "build=" NATIVE_BUILD " stage=entry\r\n");
	__asm__ volatile("mrs %0, msp\n mrs %1, ipsr" : "=r"(msp), "=r"(ipsr));
	hex(NATIVE "STATE cpuid=", REG(CPUID));
	hex(" ipsr=", ipsr);
	hex(" vtor=", REG(VTOR));
	hex(" msp=", msp);
	hex(" ibrd=", REG(UART + 0x24U));
	hex(" fbrd=", REG(UART + 0x28U));
	hex(" uart_cr=", REG(UART + 0x30U));
	print("\r\n");

	reset_interrupt_state();
	if (size != PAYLOAD_BYTES || size > SRAM_END - SRAM_BASE) {
		print(NATIVE "ERROR payload size\r\n");
		stop();
	}
	/* Load the copy and digest routines before executing them from SRAM. */
	while ((uintptr_t)code < (uintptr_t)__ramtext_end) {
		*code++ = *code_image++;
	}
	sync_cpu();
	copy_payload(ram, payload_start, size);
	sync_cpu();
	digest = payload_digest(ram, size);
	if (digest != PAYLOAD_FNV || vectors[0] != ZEPHYR_MSP ||
	    vectors[1] != ZEPHYR_RESET) {
		print(NATIVE "ERROR payload readback or vectors\r\n");
		stop();
	}
	hex(NATIVE "SRAM bytes=", size);
	hex(" fnv=", digest);
	hex(" msp=", vectors[0]);
	hex(" reset=", vectors[1]);
	print("\r\n");
#if CORE_CLOCK_HZ == 48000000U
	select_osc2();
	print(NATIVE "stage=handoff clock=osc2\r\n");
#else
	print(NATIVE "stage=handoff clock=rom\r\n");
#endif
	if (!wait_uart(UART_BUSY)) {
		stop();
	}
	bth_enter_zephyr(SRAM_BASE, vectors[0], vectors[1]);
}
