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
	int max_packets;
	int packets;
	char data[];
};
static size_t ih_work_size(struct kfd_dev *kfd, int packets)
{
	return sizeof(struct ih_work) +
		(kfd->device_info->ih_ring_entry_size * packets);
}


static struct ih_work * ih_work_alloc(struct kfd_dev *kfd)
{
	int packets = max((unsigned short)1, interrupts_per_task);
	struct ih_work *work = kzalloc(ih_work_size(kfd, packets), GFP_ATOMIC);
	if (work) {
		INIT_WORK(&work->interrupt_work, interrupt_wq);
		work->kfd = kfd;
		work->max_packets = packets;
		work->packets = 0;
	}
	return work;
}

static void ih_work_add_packet(struct ih_work *w, const void *data)
{
	size_t packet_size = w->kfd->device_info->ih_ring_entry_size;

	BUG_ON(w->packets >= w->max_packets);
	memcpy(w->data + (packet_size * w->packets++), data, packet_size);
}

static void interrupt_wdt(struct work_struct *work)
{
	unsigned long flags;
	struct kfd_dev *kfd =
		container_of(work, struct kfd_dev, irq_watchdog.work);

	/* Early checks without lock, to reduce contention */
	// If we race with a new task creation, it will take some time to timeout
	if (!kfd->irq_task_in_progress)
		return;

	spin_lock_irqsave(&kfd->interrupt_lock, flags);
	// Check task in progress again under lock
	if (kfd->irq_task_in_progress) {
			if (!queue_work(kfd->irq_wq,
			                &kfd->irq_task_in_progress->interrupt_work)) {
				dev_err_ratelimited(kfd_chardev(),
					"KFD: Failed to schedule interrupt work\n");
			} else {
				kfd->irq_task_in_progress = NULL;
			}
	}
	spin_unlock_irqrestore(&kfd->interrupt_lock, flags);
}

int kfd_interrupt_init(struct kfd_dev *kfd)
{
	spin_lock_init(&kfd->interrupt_lock);
	INIT_DELAYED_WORK(&kfd->irq_watchdog, interrupt_wdt);
	kfd->in_progress_time_stamp = 0;
	kfd->irq_task_in_progress = NULL;
	kfd->irq_wq = alloc_workqueue("kfd_irq%d", WQ_UNBOUND | WQ_SYSFS, 0,
	                              kfd->id);
	if (!kfd->irq_wq)
		return ENOMEM;
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
	struct workqueue_struct *wq = kfd->irq_wq;

	/* Cancel interrupt watchdog if running */
	cancel_delayed_work_sync(&kfd->irq_watchdog);

	spin_lock_irqsave(&kfd->interrupt_lock, flags);
	if (kfd->irq_task_in_progress) {
		struct ih_work *w = kfd->irq_task_in_progress;
		kfd->irq_task_in_progress = NULL;
		if (!kfd->irq_wq || !queue_work(kfd->irq_wq, &w->interrupt_work)) {
			dev_err(kfd_chardev(), "KFD: Failed to schedule work %s\n", kfd->irq_wq ? "" : "(NO workqueue)");
			kfree(w);
		}
	}
	kfd->interrupts_active = false;
	kfd->irq_wq = NULL;
	spin_unlock_irqrestore(&kfd->interrupt_lock, flags);

	/*
	 * flush_work ensures that there are no outstanding
	 * work-queue items that will access interrupt_ring. New work items
	 * can't be created because we stopped interrupt handling above.
	 */
	flush_workqueue(wq);
	destroy_workqueue(wq);
}

/*
 * This assumes that the interrupt_lock is held.
 */
bool enqueue_ih_ring_entry(struct kfd_dev *kfd,	const void *ih_ring_entry)
{
	struct ih_work *work = kfd->irq_task_in_progress;

	BUG_ON(spin_can_lock(&kfd->interrupt_lock));

	if (!work)
		kfd->irq_task_in_progress = work = ih_work_alloc(kfd);

	if (!work) {
		/* This is very bad, the system is likely to hang. */
		dev_err_ratelimited(kfd_chardev(),
			"Interrupt allocation failed, dropping interrupt.\n");
		return false;
	}

	BUG_ON(work->kfd != kfd);
	ih_work_add_packet(work, ih_ring_entry);

	if (work->packets < work->max_packets) {
		/* Either a new task, or we update on every packet */
		if (work->packets == 1 || interrupts_delay_extend) {
			kfd->in_progress_time_stamp = jiffies;
			/* There might have been an old timer running,
			 * modify it. Ignore failure, if it's waiting for
			 * interrupt lock it will rechedule automatically */
			mod_delayed_work(system_wq, &kfd->irq_watchdog,
			                 msecs_to_jiffies(interrupts_coalesce_delay));
		}
		return true;
	}

	/* Work has a full load of packets */
	kfd->irq_task_in_progress = NULL;
	if (!kfd->irq_wq || !queue_work(kfd->irq_wq, &work->interrupt_work)) {
		dev_err_ratelimited(kfd_chardev(), "KFD: Failed to schedule "
		                    "interrupt work %s\n",
		                    kfd->irq_wq ? "" : "(NO workqueue)");
		kfree(work);
		return false;
	}
	BUG_ON(!delayed_work_pending(&kfd->irq_watchdog) &&
	       kfd->irq_task_in_progress);
	return true;
}

/*
 * Assumption: single reader/writer. This function is not re-entrant
 */
static void interrupt_wq(struct work_struct *work)
{
	struct ih_work *w = container_of(work, struct ih_work, interrupt_work);
	size_t packet_size = w->kfd->device_info->ih_ring_entry_size;
	int i;

	for (i = 0; i < w->packets; ++i) {
		w->kfd->device_info->event_interrupt_class->interrupt_wq(w->kfd,
			(void*)(w->data + (i * packet_size)));
	}

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
