/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

#define RUNNER_PRIORITY   2
#define WORKER_STACK_SIZE (512 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define ITERATIONS        16
#define WAIT_MS           1000U

BUILD_ASSERT(CONFIG_MP_MAX_NUM_CPUS == 1);
BUILD_ASSERT(!IS_ENABLED(CONFIG_TIMESLICING));

K_THREAD_STACK_DEFINE(worker_stack, WORKER_STACK_SIZE);
K_SEM_DEFINE(wake_worker, 0, 1);

static struct k_thread worker_thread;
static atomic_t sequence;
static atomic_t expired;
static atomic_t worker_waiting;
static atomic_t worker_ran;
static bool callback_in_isr;
static bool worker_in_isr;
static uint32_t callback_end_order;
static uint32_t worker_order;

static void timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	callback_in_isr = k_is_in_isr();
	(void)atomic_inc(&sequence);
	k_sem_give(&wake_worker);
	callback_end_order = atomic_inc(&sequence) + 1U;
	atomic_set(&expired, 1);
}

K_TIMER_DEFINE(wake_timer, timer_expiry, NULL);

static void worker(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	atomic_set(&worker_waiting, 1);
	(void)k_sem_take(&wake_worker, K_FOREVER);
	worker_in_isr = k_is_in_isr();
	worker_order = atomic_inc(&sequence) + 1U;
	atomic_set(&worker_ran, 1);
}

static void check_preemption(int32_t target_priority, bool lock_scheduler)
{
	int32_t saved_priority = k_thread_priority_get(k_current_get());
	uint32_t runner_order;
	uint32_t started;
	bool timer_fired;
	bool ran_before_resume;
	bool waiting;
	int32_t join_result;
	k_tid_t tid;

	k_sem_reset(&wake_worker);
	atomic_clear(&sequence);
	atomic_clear(&expired);
	atomic_clear(&worker_waiting);
	atomic_clear(&worker_ran);
	callback_in_isr = false;
	worker_in_isr = false;
	callback_end_order = 0U;
	worker_order = 0U;

	k_thread_priority_set(k_current_get(), RUNNER_PRIORITY);
	/* Start at a higher priority so the worker blocks before we resume. */
	tid = k_thread_create(&worker_thread, worker_stack, K_THREAD_STACK_SIZEOF(worker_stack),
			      worker, NULL, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	waiting = atomic_get(&worker_waiting) != 0;
	k_thread_priority_set(tid, target_priority);

	if (lock_scheduler) {
		k_sched_lock();
	}

	started = k_uptime_get_32();
	k_timer_start(&wake_timer, K_MSEC(10), K_NO_WAIT);
	/* No sleep, yield, semaphore wait or console output in this window. */
	while ((atomic_get(&expired) == 0) && ((uint32_t)(k_uptime_get_32() - started) < WAIT_MS)) {
	}

	timer_fired = atomic_get(&expired) != 0;
	runner_order = atomic_inc(&sequence) + 1U;
	ran_before_resume = atomic_get(&worker_ran) != 0;

	if (lock_scheduler) {
		k_sched_unlock();
	}
	k_timer_stop(&wake_timer);
	join_result = k_thread_join(tid, K_MSEC(WAIT_MS));
	if (join_result != 0) {
		k_thread_abort(tid);
	}
	k_thread_priority_set(k_current_get(), saved_priority);

	zassert_true(waiting, "worker did not reach its wait");
	zassert_true(timer_fired, "system timer did not expire");
	zassert_true(callback_in_isr, "timer callback was not in an ISR");
	zassert_ok(join_result, "worker did not finish");
	zassert_false(worker_in_isr, "worker ran in interrupt context");
	zassert_equal(callback_end_order, 2U);
	zassert_true(worker_order > callback_end_order, "worker ran before the callback completed");

	if ((target_priority < RUNNER_PRIORITY) && !lock_scheduler) {
		zassert_true(ran_before_resume, "higher priority worker did not preempt");
		zassert_equal(worker_order, 3U);
		zassert_equal(runner_order, 4U);
	} else {
		zassert_false(ran_before_resume, "worker preempted despite lock or priority");
		zassert_equal(runner_order, 3U);
		zassert_equal(worker_order, 4U);
	}
}

ZTEST(preempt_irq, test_higher_priority)
{
	for (int32_t i = 0; i < ITERATIONS; i++) {
		check_preemption(K_PRIO_PREEMPT(0), false);
	}
}

ZTEST(preempt_irq, test_scheduler_lock)
{
	for (int32_t i = 0; i < ITERATIONS; i++) {
		check_preemption(K_PRIO_PREEMPT(0), true);
	}
}

ZTEST(preempt_irq, test_equal_priority)
{
	for (int32_t i = 0; i < ITERATIONS; i++) {
		check_preemption(K_PRIO_PREEMPT(RUNNER_PRIORITY), false);
	}
}

ZTEST(preempt_irq, test_lower_priority)
{
	for (int32_t i = 0; i < ITERATIONS; i++) {
		check_preemption(K_PRIO_PREEMPT(3), false);
	}
}

ZTEST_SUITE(preempt_irq, NULL, NULL, NULL, NULL, NULL);
