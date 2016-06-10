/*
 * Copyright 2014 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * KFD Interrupts.
 *
 * AMD GPUs deliver interrupts by pushing an interrupt description onto the
 * interrupt ring and then sending an interrupt. KGD receives the interrupt
 * in ISR and sends us a pointer to each new entry on the interrupt ring.
 *
 * We generally can't process interrupt-signaled events from ISR, so we call
 * out to each interrupt client module (currently only the scheduler) to ask if
 * each interrupt is interesting. If they return true, then it requires further
 * processing so we copy it to an internal interrupt ring and call each
 * interrupt client again from a work-queue.
 *
 * There's no acknowledgment for the interrupts we use. The hardware simply
 * queues a new interrupt each time without waiting.
 *
 * The fixed-size internal queue means that it's possible for us to lose
 * interrupts because we have no back-pressure to the hardware.
 */

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kfifo.h>
#include "kfd_priv.h"

static void interrupt_wq(struct work_struct *);

struct ih_work {
	struct work_struct interrupt_work;
	struct kfd_dev *kfd;
	char data[];
};

static void ih_work_init(struct ih_work *w, struct kfd_dev *kfd,
			const void *data)
{
	INIT_WORK(&w->interrupt_work, interrupt_wq);
	w->kfd = kfd;
	memcpy(w->data, data, kfd->device_info->ih_ring_entry_size);
}

static size_t ih_work_size(struct kfd_dev *kfd)
{
	return sizeof(struct ih_work) + kfd->device_info->ih_ring_entry_size;
}

int kfd_interrupt_init(struct kfd_dev *kfd)
{
	spin_lock_init(&kfd->interrupt_lock);
	kfd->interrupts_active = true;

	/*
	 * After this function returns, the interrupt will be enabled. This
	 * barrier ensures that the interrupt running on a different processor
	 * sees all the above writes.
	 */
	smp_wmb();

	return 0;
}

void kfd_interrupt_exit(struct kfd_dev *kfd)
{
	/*
	 * Stop the interrupt handler from writing to the ring and scheduling
	 * workqueue items. The spinlock ensures that any interrupt running
	 * after we have unlocked sees interrupts_active = false.
	 */
	unsigned long flags;

	spin_lock_irqsave(&kfd->interrupt_lock, flags);
	kfd->interrupts_active = false;
	spin_unlock_irqrestore(&kfd->interrupt_lock, flags);

	/*
	 * flush_work ensures that there are no outstanding
	 * work-queue items that will access interrupt_ring. New work items
	 * can't be created because we stopped interrupt handling above.
	 */
	flush_scheduled_work();
}

/*
 * This assumes that the interrupt_lock is held.
 */
bool enqueue_ih_ring_entry(struct kfd_dev *kfd,	const void *ih_ring_entry)
{
	struct ih_work *work = kzalloc(ih_work_size(kfd), GFP_ATOMIC);

	if (!work) {
		/* This is very bad, the system is likely to hang. */
		dev_err_ratelimited(kfd_chardev(),
			"Interrupt allocation failed, dropping interrupt.\n");
		return false;
	}
	ih_work_init(work, kfd, ih_ring_entry);

	// TODO: Maybe consider other queues? setup our own queue?
	// use system default for now
	if (!schedule_work(&work->interrupt_work)) {
		dev_err_ratelimited(kfd_chardev(), "KFD: Failed to chedule work\n");
		kfree(work);
		return false;
	}
	return true;
}

/*
 * Assumption: single reader/writer. This function is not re-entrant
 */
static void interrupt_wq(struct work_struct *work)
{
	struct ih_work *w = container_of(work, struct ih_work, interrupt_work);

	w->kfd->device_info->event_interrupt_class->interrupt_wq(w->kfd, (void*)w->data);
	// We are done and the work has been removed from the work queue
	// nothing should touch this memory
	kfree(w);
}

bool interrupt_is_wanted(struct kfd_dev *dev,
			const uint32_t *ih_ring_entry,
			uint32_t *patched_ihre, bool *flag)
{
	/* integer and bitwise OR so there is no boolean short-circuiting */
	unsigned int wanted = 0;

	wanted |= dev->device_info->event_interrupt_class->interrupt_isr(dev,
					 ih_ring_entry, patched_ihre, flag);

	return wanted != 0;
}
