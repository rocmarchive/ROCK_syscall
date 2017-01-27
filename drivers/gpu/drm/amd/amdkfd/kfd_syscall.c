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
#include <asm/unistd.h>

#include <linux/atomic.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/fsnotify.h>
#include <linux/highmem.h>
#include <linux/hugetlb.h>
#include <linux/kfd_sc.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/mmu_context.h>
#include <linux/net.h>
#include <linux/pagemap.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/socket.h>
#include <linux/uio.h>
#include <linux/userfaultfd_k.h>

#include <asm/errno.h>

#include "kfd_priv.h"
#include "kfd_dbgmgr.h"

/*
 *	Send a datagram to a given address. We move the address into kernel
 *	space and check the user space data area is readable before invoking
 *	the protocol.
 */

static int gpu_sc_sendto(struct kfd_process *p, int fd, void __user * buff,
                         size_t len, unsigned int flags,
                         struct sockaddr __user * addr, int addr_len)
{
	struct socket *sock;
	struct sockaddr_storage address;
	int err;
	struct msghdr msg;
	struct iovec iov;

	err = import_single_range(WRITE, buff, len, &iov, &msg.msg_iter);
	if (unlikely(err))
		return err;
	sock = sockfd_lookup_tsk(p->lead_thread, fd, &err);
	if (!sock)
		goto out;

	msg.msg_name = NULL;
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	msg.msg_namelen = 0;
	if (addr) {
		err = move_addr_to_kernel(addr, addr_len, &address);
		if (err < 0)
			goto out_put;
		msg.msg_name = (struct sockaddr *)&address;
		msg.msg_namelen = addr_len;
	}
	if (sock->file->f_flags & O_NONBLOCK)
		flags |= MSG_DONTWAIT;
	msg.msg_flags = flags;
	err = sock_sendmsg(p->lead_thread, sock, &msg);

out_put:
	fput(sock->file);
out:
	return err;
}

/*
 *	Receive a frame from the socket and optionally record the address of the
 *	sender. We verify the buffers are writable and if needed move the
 *	sender address from kernel to user space.
 */

static int gpu_sc_recvfrom(struct kfd_process *p, int fd, void __user *ubuf,
                           size_t size, unsigned int flags,
                           struct sockaddr __user *addr, int __user *addr_len)
{
	struct socket *sock;
	struct iovec iov;
	struct msghdr msg;
	struct sockaddr_storage address;
	int err, err2;

	err = import_single_range(READ, ubuf, size, &iov, &msg.msg_iter);
	if (unlikely(err))
		return err;
	sock = sockfd_lookup_tsk(p->lead_thread, fd, &err);
	if (!sock)
		goto out;

	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	/* Save some cycles and don't copy the address if not needed */
	msg.msg_name = addr ? (struct sockaddr *)&address : NULL;
	/* We assume all kernel code knows the size of sockaddr_storage */
	msg.msg_namelen = 0;
	msg.msg_iocb = NULL;
	if (sock->file->f_flags & O_NONBLOCK)
		flags |= MSG_DONTWAIT;

	err = sock_recvmsg(p->lead_thread, sock, &msg, flags);

	if (err >= 0 && addr != NULL) {
		err2 = move_addr_to_user(&address,
					 msg.msg_namelen, addr, addr_len);
		if (err2 < 0)
			err = err2;
	}

	fput(sock->file);
out:
	return err;
}

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

static ssize_t gpu_sc_lseek(struct kfd_process *p,  unsigned int fd,
                            off_t offset, unsigned int whence)
{
	off_t retval;
	struct fd f = fdget_pos_task(fd, p->lead_thread);
	if (!f.file)
		return -EBADF;

	retval = -EINVAL;
	if (whence <= SEEK_MAX) {
		loff_t res = vfs_llseek(f.file, offset, whence);
		retval = res;
		if (res != (loff_t)retval)
			retval = -EOVERFLOW;	/* LFS: should only happen on 32 bit platforms */
	}
	fdput_pos_task(f);
	return retval;
}

static void kfd_sc_process(struct kfd_process *p, struct kfd_sc *s,
                           bool *usemm, bool *needs_resume)
{
	long ret = 0;
	bool noret;
	u32 sc_num;
	u64 *arg;
	u64 larg[6];

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
	if (noret) {
		memcpy(larg, s->arg, sizeof(larg));
		arg = larg;
		atomic_set(status, KFD_SC_STATUS_FREE);
	} else {
		arg = s->arg;
	}

	switch (sc_num) {
	case __NR_restart_syscall: /* hijack __NR_restart_syscall as nop syscall */
		ret = 0;
		break;
	case __NR_read:
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = gpu_sc_read(p, arg[0], arg[1], arg[2]);
		break;
	case __NR_pread64:
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = gpu_sc_pread(p, arg[0], arg[1], arg[2], arg[3]);
		break;
	case __NR_write:
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = gpu_sc_write(p, arg[0], arg[1], arg[2]);
		break;
	case __NR_pwrite64:
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = gpu_sc_pwrite(p, arg[0], arg[1], arg[2], arg[3]);
		break;
	case __NR_lseek:
		ret = gpu_sc_lseek(p, arg[0], arg[1], arg[2]);
		break;
	case __NR_open:
		/* We need to have access to the filename buffer */
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = do_sys_open(p->lead_thread, AT_FDCWD,
		                  (const char* __user)arg[0], arg[1], arg[2]);
		break;
	case __NR_close:
		ret = __close_fd(p->lead_thread->files, arg[0]);
		break;
	case __NR_mmap:
		ret = mmap_pgoff_task(p->lead_thread, arg[0], arg[1], arg[2],
		                      arg[3], arg[4], arg[5]);
		break;
	case __NR_munmap: {
		LIST_HEAD(uf);
		down_write(&p->lead_thread->mm->mmap_sem);
		ret = do_munmap(p->lead_thread->mm, arg[0], arg[1], &uf);
		up_write(&p->lead_thread->mm->mmap_sem);
		userfaultfd_unmap_complete(p->lead_thread->mm, &uf);
		break;
	}
	case __NR_madvise:
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = __do_madvise(p->lead_thread, arg[0], arg[1], arg[2]);
		break;
	case __NR_recvfrom:
		/* We need to have access to the filename buffer */
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = gpu_sc_recvfrom(p, arg[0], (void*)arg[1], arg[2],
		                      arg[3], (void*)arg[4], (void*)arg[5]);
		break;
	case __NR_sendto:
		/* We need to have access to the filename buffer */
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		ret = gpu_sc_sendto(p, arg[0], (void*)arg[1], arg[2],
		                    arg[3], (void*)arg[4], arg[5]);
		break;
	case __NR_getrusage:
		/* We need to have access to the rusage buffer */
		if (!*usemm) {
			use_mm(p->mm);
			*usemm = true;
		}
		if (s->arg[0] != RUSAGE_SELF && s->arg[0] != RUSAGE_CHILDREN &&
		    s->arg[0] != RUSAGE_THREAD)
			ret = -EINVAL;
		else
			ret = getrusage(p->lead_thread, s->arg[0], (void*)s->arg[1]);
		break;
	default:
		pr_warn("KFD_SC: Found pending syscall: "
		       "%x:%x:%llx:%llx:%llx:%llx:%llx:%llx\n",
		       s->status, s->sc_num, s->arg[0], s->arg[1], s->arg[2],
		       s->arg[3], s->arg[4], s->arg[5]);
		ret = -ENOSYS;
	}
	if (!noret) {
		s->arg[0] = ret;
		atomic_set(status, KFD_SC_STATUS_FINISHED);
		*needs_resume = true;
	}
}

#define DEFAULT_COUNT 25
static void kfd_sc_resume_wave(struct kfd_process *p, uint32_t msg_id, bool all)
{
	struct dbg_wave_control_info wac_info;
	struct kfd_dev *dev;
	long ret;
	int count = DEFAULT_COUNT;

	memset((void *) &wac_info, 0, sizeof(struct dbg_wave_control_info));
	wac_info.process = p;
	wac_info.operand = HSA_DBG_WAVEOP_RESUME;
	wac_info.mode = all ? HSA_DBG_WAVEMODE_BROADCAST_PROCESS : HSA_DBG_WAVEMODE_SINGLE;
	wac_info.trapId = 0; //not used for resume
	wac_info.dbgWave_msg.DbgWaveMsg.WaveMsgInfoGen2.Value = all ? 0 : msg_id;

	/* Make sure we don't send unintended data */
	wac_info.dbgWave_msg.DbgWaveMsg.WaveMsgInfoGen2.ui32.UserData = 0;
	wac_info.dbgWave_msg.DbgWaveMsg.WaveMsgInfoGen2.ui32.Priv = 0;
	wac_info.dbgWave_msg.DbgWaveMsg.WaveMsgInfoGen2.ui32.Reserved0 = 0;
	wac_info.dbgWave_msg.DbgWaveMsg.WaveMsgInfoGen2.ui32.Reserved1 = 0;
	wac_info.dbgWave_msg.MemoryVA = NULL; //not used for resume

	dev = kfd_get_first_process_device_data(p)->dev;
	mutex_lock(get_dbgmgr_mutex());
	do {
		/* ENOMEM just means we ran out of queue space */
		if (count == 0 && ret != -ENOMEM) {
			wac_info.mode = HSA_DBG_WAVEMODE_BROADCAST_PROCESS;
			wac_info.dbgWave_msg.DbgWaveMsg.WaveMsgInfoGen2.Value = 0;
			pr_warn("KFD_SC: Resuming all waves.\n");
		}
		ret = kfd_dbgmgr_wave_control(dev->dbgmgr, &wac_info);
		if (ret)
			pr_err("KFD_SC: Failed to resume wave %x: %ld\n", msg_id, ret);
	} while ((ret == -ETIME || ret == -ENOMEM) && (count-- > 0));
	mutex_unlock(get_dbgmgr_mutex());
	if (count != DEFAULT_COUNT)
		pr_info("KFD_SC: Wave resume status %x: %ld (%d retries)\n",
			msg_id, ret, DEFAULT_COUNT - count);
}

#define WAVESIZE 64
int kfd_syscall(struct kfd_process *p, unsigned data)
{
	bool usemm = false;
	bool needs_resume = false;
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
			kfd_sc_process(p, &(p->sc_kloc[i]), &usemm, &needs_resume);
			++handled;
		}
	}

	if ((handled == 0) && (to_scan == WAVESIZE) && sc_rescan_fallback) {
		pr_warn("KFD_SC: Failed to find SC request (0x%x:0x%x:%p) "
		        "scanning all %lu elements\n", wf_id, data,
			p->sc_kloc + start, p->sc_elements);
		to_scan = p->sc_elements;
		start = 0;
		goto retry;
	}
	if (handled == 0 && sc_rescan_fallback) {
		pr_err("KFD_SC: Failed to find SC request despite scanning "
		       " %u elements. The GPU might hang.\n", to_scan);
		return -EIO;
	}
	if (usemm)
		unuse_mm(p->mm);
	if (needs_resume && sc_use_wave_management) {
		pr_debug("KFD_SC: Resumeing wave %x\n", data);
		kfd_sc_resume_wave(p, data, to_scan != WAVESIZE);
	}
	pr_debug("KFD_SC: Handled %u syscall requests in %u\n",
		handled, to_scan);

	return 0;
}
