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

#include <linux/atomic.h>
#include <linux/file.h>
#include <linux/fsnotify.h>
#include <linux/highmem.h>
#include <linux/kfd_sc.h>
#include <linux/mm.h>
#include <linux/mmu_context.h>
#include <linux/pagemap.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <asm/errno.h>
#include <asm/unistd.h>

#include "kfd_priv.h"

/* TODO: These are almost exact copies of functions in  fs/read_write.c.
 * Export and reuse those functions instead. */
static struct fd fdget_pos_task(unsigned int fd, struct task_struct *task)
{
	return __to_fd(__fdget_pos_task(fd, task));
}

static void fdput_pos_task(struct fd f)
{
	if (f.flags & FDPUT_POS_UNLOCK)
		mutex_unlock(&f.file->f_pos_lock);
	fdput(f);
}

/* Mostly a copy of sys_read. */
static ssize_t gpu_sc_read(struct kfd_process *p, unsigned int fd,
                           unsigned long ptr, size_t count)
{
	struct fd f = fdget_pos_task(fd, p->lead_thread);
	ssize_t ret = -EBADF;

	if (f.file) {
		// file_pos_read
		loff_t pos = f.file->f_pos;
		ret = vfs_read(p->lead_thread, f.file, (char *)ptr, count, &pos);
		if (ret >= 0)
			f.file->f_pos = pos; // file_pos_write(f.file, pos);
		fdput_pos_task(f);
	}

	return ret;
}

/* Mostly a copy of sys_pread64. */
static ssize_t gpu_sc_pread(struct kfd_process *p, unsigned int fd,
                            unsigned long ptr, size_t count, loff_t pos)
{
	struct fd f = {0,};
	ssize_t ret = -EBADF;

	if (pos < 0)
		return -EINVAL;

	f = fdget_task(fd, p->lead_thread);
	if (f.file) {
		ret = -ESPIPE;
		if (f.file->f_mode & FMODE_PREAD)
			ret = vfs_read(p->lead_thread, f.file, (char *)ptr, count, &pos);
		fdput(f);
	}

	return ret;
}

/* Mostly a copy of sys_write. */
static ssize_t gpu_sc_write(struct kfd_process *p, unsigned int fd,
                            unsigned long ptr, size_t count)
{
	struct fd f = fdget_pos_task(fd, p->lead_thread);
	ssize_t ret = -EBADF;

	if (f.file) {
		// file_pos_read
		loff_t pos = f.file->f_pos;
		ret = vfs_write(p->lead_thread, f.file, (char *)ptr, count, &pos);
		if (ret >= 0)
			f.file->f_pos = pos; // file_pos_write(f.file, pos);
		fdput_pos_task(f);
	}

	return ret;
}

/* Mostly a copy of sys_pwrite64. */
static ssize_t gpu_sc_pwrite(struct kfd_process *p, unsigned int fd,
                             unsigned long ptr, size_t count, loff_t pos)
{
	struct fd f;
	ssize_t ret = -EBADF;

	if (pos < 0)
		return -EINVAL;

	f = fdget_task(fd, p->lead_thread);
	if (f.file) {
		ret = -ESPIPE;
		if (f.file->f_mode & FMODE_PWRITE)
			ret = vfs_write(p->lead_thread, f.file, (char *)ptr, count, &pos);
		fdput(f);
	}

	return ret;
}

static void kfd_sc_process(struct kfd_process *p, struct kfd_sc *s,
                           bool *usemm)
{
	u32 sc_num;
	int ret = 0;
	bool noret;

	//TODO: verify that this si legal to do.
	atomic_t *status = (atomic_t*)&s->status;
	/* Somebody took it from us. Tis should not really happen since
	 * only one CPU should recieve interrupt with corresponding wave id */
	if (atomic_cmpxchg(status, KFD_SC_STATUS_READY, KFD_SC_STATUS_BUSY) !=
	    KFD_SC_STATUS_READY)
		return;

	pr_debug("KFD_SC: padding: 0x%llx:0x%llx\n",
		s->padding >> 32, s->padding & 0xffffffff);

	sc_num = s->sc_num & ~KFD_SC_NORET_FLAG;
	noret = (s->sc_num & KFD_SC_NORET_FLAG) != 0;
	switch (sc_num) {
	case __NR_restart_syscall: /* hijack __NR_restart_syscall as nop syscall */
		ret = 0;
		break;
	case __NR_read:
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = gpu_sc_read(p, s->arg[0], s->arg[1], s->arg[2]);
		break;
	case __NR_pread64:
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = gpu_sc_pread(p, s->arg[0], s->arg[1], s->arg[2], s->arg[3]);
		break;
	case __NR_write:
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = gpu_sc_write(p, s->arg[0], s->arg[1], s->arg[2]);
		break;
	case __NR_pwrite64:
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = gpu_sc_pwrite(p, s->arg[0], s->arg[1], s->arg[2], s->arg[3]);
		break;
	default:
		pr_warn("KFD_SC: Found pending syscall: "
		       "%x:%x:%llx:%llx:%llx:%llx:%llx:%llx\n",
		       s->status, s->sc_num, s->arg[0], s->arg[1], s->arg[2],
		       s->arg[3], s->arg[4], s->arg[5]);
		ret = -ENOSYS;
	}
	s->arg[0] = ret;
	atomic_set(status, noret ? KFD_SC_STATUS_FREE : KFD_SC_STATUS_FINISHED);
}

#define WAVESIZE 64
int kfd_syscall(struct kfd_process *p, unsigned data)
{
	bool usemm = false;
	unsigned start, to_scan, i, handled;
	unsigned wf_id = ( ((data >> 18) & 0x3f) | ((data >> 1) & 0x40) | ((data >> 17) & 0x180) ) * 10 + ((data >> 14) & 0xf);

	pr_debug("KFD_SC: Handling syscall for process: %p\n", p);
	if (!p)
		return -ESRCH;

	if ((data >> 26) == 2) {
		pr_err("KFD_SC: WF %x received error interrupt message: %x\n",
			wf_id, data);
		return -EIO;
	}

	if (!p->sc_location || !p->sc_kloc) {
		pr_err("KFD_SC: System call request without registered area\n");
		return -EIO;
	}

	pr_debug("KFD_SC: syscall wf_id: %x(%x)\n", wf_id, data);
	to_scan = WAVESIZE;
	start = wf_id * WAVESIZE;
	handled = 0;
	pr_debug("KFD_SC: scanning from: %d(%d-%p)\n", wf_id, start,
		p->sc_kloc + start);

retry:
	BUG_ON(start + to_scan > p->sc_elements);
	for (i = start; i < (start + to_scan); ++i) {
		if (p->sc_kloc[i].status == KFD_SC_STATUS_READY) {
			if (to_scan != WAVESIZE)
				pr_info("KFD_SC: Found request at %d\n", i);
			kfd_sc_process(p, &(p->sc_kloc[i]), &usemm);
			++handled;
		}
	}

	if ((handled == 0) && (to_scan == WAVESIZE)) {
		pr_warn("KFD_SC: Failed to find SC request (0x%x:0x%x:%p) "
		        "scanning all %lu elements\n", wf_id, data,
			p->sc_kloc + start, p->sc_elements);
		to_scan = p->sc_elements;
		start = 0;
		goto retry;
	}
	if (handled == 0) {
		pr_err("KFD_SC: Failed to find SC request despite scanning "
		       " %u elements. The GPU will hang.\n", to_scan);
		return -EIO;
	}
	if (usemm)
		unuse_mm(p->mm);
	pr_debug("KFD_SC: Handled %u syscall requests in %u\n",
		handled, to_scan);

	return 0;
}
