/*
 * drivers/block/sunxi_nand/nfd/nand_blk.c
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/spinlock.h>
#include <linux/hdreg.h>
#include <linux/init.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#include "../src/include/nand_type.h"
#include "../src/include/nand_drv_cfg.h"
#include "../nfc/nfc_i.h"
#include "nand_blk.h"
#include <mach/clock.h>
#include <plat/sys_config.h>

#include "nand_private.h"
#include "../include/type_def.h"
#include "../nandtest/nand_test.h"

#include "nand_private.h"
#include <linux/wait.h>
#include <linux/sched.h>

#define BLK_ERR_MSG_ON
#ifdef  BLK_ERR_MSG_ON
#define dbg_err(fmt, args...) printk("[NAND]"fmt, ## args)
#else
#define dbg_err(fmt, ...)  ({})
#endif


//#define BLK_INFO_MSG_ON
#ifdef  BLK_INFO_MSG_ON
#define dbg_inf(fmt, args...) printk("[NAND]"fmt, ## args)
#else
#define dbg_inf(fmt, ...)  ({})
#endif


#define REMAIN_SPACE   0
#define PART_FREE      0x55
#define PART_DUMMY     0xff
#define PART_READONLY  0x85
#define PART_WRITEONLY 0x86
#define PART_NO_ACCESS 0x87

#define TIMEOUT  1  // in seconds

//#define NAND_CACHE_FLUSH_EVERY_SEC
//#define NAND_BIO_ALIGN     // for 2.6.29 ?
#define NAND_CACHE_RW
#define USE_SYS_PIN
//#define USE_SYS_CLK

/**
*USE_BIO_MERGE level description:
*1	:	merge bvc in one bio
*2	:	merge bvc in one bio and merge bios in one request
*/
#define USE_BIO_MERGE   0
#define NAND_TEST_TICK  0

#ifdef NAND_CACHE_FLUSH_EVERY_SEC
static int after_write = 0;
struct collect_ops {
	unsigned long timeout;
	wait_queue_head_t wait;
	struct completion thread_exit;
	unsigned char quit;
};
struct collect_ops collect_arg;
#endif


DEFINE_SEMAPHORE(nand_mutex);
static unsigned char volatile IS_IDLE = 1;
u32 nand_handle = 0;
spinlock_t nand_rb_lock;

#ifdef USE_SYS_CLK
static struct clk *ahb_nand_clk = NULL;
static struct clk *mod_nand_clk = NULL;
#endif

static int nand_flush(struct nand_blk_dev *dev);

static irqreturn_t nand_rb_interrupt(int irq, void* dev_id)
{
    unsigned long iflags;

    spin_lock_irqsave(&nand_rb_lock, iflags);
    NAND_RbInterrupt();
    spin_unlock_irqrestore(&nand_rb_lock, iflags);

	return IRQ_HANDLED;
}


#if USE_BIO_MERGE==0
static int cache_align_page_request(struct nand_blk_ops* nandr,
				    struct nand_blk_dev* dev,
				    struct request* req)
{
	unsigned long start,nsector;
	char *buf;
	__s32 ret;

	int cmd = rq_data_dir(req);

	if (dev->disable_access ||
	    ((cmd == WRITE) && (dev->readonly)) ||
	    ((cmd == READ)  && (dev->writeonly))) {
		dbg_err("can not access this part\n");
		return -EIO;
	}

	//for2.6.36
	buf = req->buffer;
	start = blk_rq_pos(req);
	nsector = blk_rq_cur_bytes(req)>>9;

	if ( (start + nsector) > get_capacity(req->rq_disk)) {
		dbg_err("over the limit of disk\n");
		return -EIO;
	}
	start += dev->off_size;

	switch(cmd) {

	case READ:

		dbg_inf("READ:%lu from %lu\n",nsector,start);

#ifndef NAND_CACHE_RW
		LML_FlushPageCache();
		ret = LML_Read(start, nsector, buf);
#else
		//printk("Rs %lu %lu \n",start, nsector);
		LML_FlushPageCache();
		ret = NAND_CacheRead(start, nsector, buf);
		//printk("Rs %lu %lu \n",start, nsector);
#endif // NAND_CACHE_RW
		if (ret) {
			dbg_err("cache_align_page_request:read err\n");
			return -EIO;
		}
		return 0;

	case WRITE:

		dbg_inf("WRITE:%lu from %lu\n",nsector,start);
#ifndef NAND_CACHE_RW
		ret = LML_Write(start, nsector, buf);
#else
		//printk("Ws %lu %lu \n",start, nsector);
		ret = NAND_CacheWrite(start, nsector, buf);
		//printk("We %lu %lu \n",start, nsector);
#endif // NAND_CACHE_RW
		if (ret) {
			dbg_err("cache_align_page_request:write err\n");
			return -EIO;
		}
		return 0;

	default:
		dbg_err("Unknown request \n");
		return -EIO;
	}
}
#endif // USE_BIO_MERGE==0

//for 2.6.29
#ifdef NAND_BIO_ALIGN

static int nand_blk_phys_contig_segment(struct request_queue* q, struct bio* bio, struct bio* nxt)
{
	//if (!(q->queue_flags & (1 << QUEUE_FLAG_CLUSTER)))
	//	return 0;

	if (!BIOVEC_PHYS_MERGEABLE(__BVEC_END(bio), __BVEC_START(nxt)))
		return 0;
	if (bio->bi_size + nxt->bi_size > q->limits.max_segment_size)
		return 0;

	/*
	 * bio and nxt are contigous in memory, check if the queue allows
	 * these two to be merged into one
	 */
	if (BIO_SEG_BOUNDARY(q, bio, nxt))
		return 1;

	return 0;
}


/* Compute maximal contiguous buffer size. */
static int nand_buffer_chain_size(struct request* req)
{
	struct bio *bio,*prevbio = NULL;
	struct bio_vec *bv;
	int i;
	unsigned long size = 0;
	char *base = bio_data(req->bio);
	unsigned int phys_size = 0;
	struct request_queue *q = req->q;

	__rq_for_each_bio(bio, req) {
		if(prevbio) {
			int pseg = phys_size + prevbio->bi_size + bio->bi_size;
			if (!nand_blk_phys_contig_segment(q, prevbio, bio) ||
			    pseg > q->limits.max_segment_size)
				break;
		}
		bio_for_each_segment(bv, bio, i) {
			if (page_address(bv->bv_page) + bv->bv_offset != base + size)
				break;
			size += bv->bv_len;
		}
		prevbio = bio;
	}
	//return size>>9;
	return size;
}

static void reset(struct request* req)
{
	//req->current_nr_sectors = req->hard_cur_sectors = nand_buffer_chain_size(req);
	nand_buffer_chain_size(req);
}

#endif // NAND_BIO_ALIGN


#if USE_BIO_MERGE
#define nand_bio_kmap(bio,idx,kmtype)	\
	(page_address(bio_iovec_idx((bio), (idx))->bv_page) +	bio_iovec_idx((bio), (idx))->bv_offset)


static int nand_transfer(struct nand_blk_dev* dev,
			 unsigned long start, unsigned long nsector,
			 char* buf, int cmd)
{
	__s32 ret;

	if(dev->disable_access || ( (cmd == WRITE) && (dev->readonly) ) \
		|| ((cmd == READ) && (dev->writeonly))){
		dbg_err("can not access this part\n");
		return -EIO;
	}
	//printk("[N]start=%lx,nsec=%lx,buffer=%p,cmd=%d\n",start,nsector,buf,cmd);
	start += dev->off_size;

	switch(cmd) {

	case READ:
        //printk("R %lu %lu 0x%x \n",start, nsector, (__u32)buf);
		dbg_inf("READ:%lu from %lu\n",nsector,start);

#ifndef NAND_CACHE_RW
		LML_FlushPageCache();
		ret = LML_Read(start, nsector, buf);
#else
		//printk("Rs %lu %lu \n",start, nsector);
      		LML_FlushPageCache();
		ret = NAND_CacheRead(start, nsector, buf);
		//printk("Rs %lu %lu \n",start, nsector);
#endif // NAND_CACHE_RW
		if (ret) {
			dbg_err("cache_align_page_request:read err\n");
			return -EIO;
		}
		return 0;

	case WRITE:
        //printk("W %lu %lu 0x%x \n",start, nsector, (__u32)buf);
		dbg_inf("WRITE:%lu from %lu\n",nsector,start);
#ifndef NAND_CACHE_RW
		ret = LML_Write(start, nsector, buf);
#else
		//printk("Ws %lu %lu \n",start, nsector);
		ret = NAND_CacheWrite(start, nsector, buf);
		//printk("We %lu %lu \n",start, nsector);
#endif // NAND_CACHE_RW
		if (ret) {
			dbg_err("cache_align_page_request:write err\n");
			return -EIO;
		}
		return 0;

	default:
		dbg_err("Unknown request \n");
		return -EIO;
	}
}
#endif

#if NAND_TEST_TICK
static unsigned long nand_rw_time = 0;
#endif

#if USE_BIO_MERGE==1
static int nand_xfer_bio(struct request_queue* rq, struct nand_blk_dev* dev,
			 struct bio* bio, unsigned long start_idx)
{
	struct nand_blk_ops *nandr = rq->queuedata;
	struct bio_vec* bvec;
	unsigned long long sector = bio->bi_sector;
	char* buffer;
	unsigned long rq_len = 0;

	buffer = nand_bio_kmap(bio, bio->bi_idx, KM_USER0);

	bio_for_each_segment(bvec, bio, start_idx) {
		if(start_idx < bio->bi_vcnt - 1) {
			if (nand_bio_kmap(bio, start_idx + 1, KM_USER0) ==
			    nand_bio_kmap(bio, start_idx,     KM_USER0) + bvec->bv_len) {
				rq_len += bvec->bv_len;
			}
			else {
				rq_len += bvec->bv_len;
				spin_unlock_irq(rq->queue_lock);
				down(&nandr->nand_ops_mutex);
#if NAND_TEST_TICK
				tick = jiffies;
				nand_transfer(dev, sector, rq_len>>9, buffer, bio_data_dir(bio));
				nand_rw_time += jiffies - tick;
#else
				nand_transfer(dev, sector, rq_len>>9, buffer, bio_data_dir(bio));
#endif // NAND_TEST_TICK
				up(&nandr->nand_ops_mutex);
				spin_lock_irq(rq->queue_lock);
				sector += rq_len>>9;
				rq_len = 0;
				buffer = nand_bio_kmap(bio, start_idx + 1, KM_USER0);
			}
		}
		else {
			rq_len += bvec->bv_len;
			spin_unlock_irq(rq->queue_lock);
			down(&nandr->nand_ops_mutex);
#if NAND_TEST_TICK
			tick = jiffies;
			nand_transfer(dev, sector,  rq_len>>9, buffer, bio_data_dir(bio));
			nand_rw_time += jiffies - tick;
#else
			nand_transfer(dev, sector,  rq_len>>9, buffer, bio_data_dir(bio));
#endif // NAND_TEST_TICK
			up(&nandr->nand_ops_mutex);
			spin_lock_irq(rq->queue_lock);
			rq_len=0;
		}
	}
	return 0;
}

#elif USE_BIO_MERGE==2
static int nand_xfer_bio(struct request_queue* rq, struct nand_blk_dev* dev,
			 struct bio* bio, unsigned long start_idx)
{
	struct nand_blk_ops *nandr = rq->queuedata;
	struct bio_vec* bvec;
	unsigned long long sector = bio->bi_sector;
	char* buffer = NULL;
	unsigned long rq_len = 0;

//	buffer = nand_bio_kmap(bio, bio->bi_idx, KM_USER0);

	bio_for_each_segment(bvec, bio, start_idx) {
		if (nand_bio_kmap(bio, start_idx, KM_USER0) ==
		    buffer + rq_len) {
			/*merge vec*/
			rq_len += bvec->bv_len;
		}
		else {
			/*flush previous data*/
			if (rq_len) {
				spin_unlock_irq(rq->queue_lock);
				down(&nandr->nand_ops_mutex);
#if NAND_TEST_TICK
				tick = jiffies;
				nand_transfer(dev, sector,  rq_len>>9, buffer, rw_flag);
				nand_rw_time += jiffies - tick;
#else
				nand_transfer(dev, sector, rq_len>>9, buffer, rw_flag);
#endif
				up(&nandr->nand_ops_mutex);
				spin_lock_irq(rq->queue_lock);
			}
			/*update new*/
			sector += rq_len>>9;
			buffer = nand_bio_kmap(bio, start_idx, KM_USER0);
			rq_len = bvec->bv_len;
		}
	}
	return 0;
}

#else // USE_BIO_MERGE
// nand_xfer_bio() is undefined

#endif // USE_BIO_MERGE


static int nand_xfer_request(struct request_queue* rq, struct request* req)
{
	struct nand_blk_dev* dev = req->rq_disk->private_data;
	struct nand_blk_ops *nandr = rq->queuedata;
	int res = 0;

#if NAND_TEST_TICK
	unsigned long tick=0;
#endif

#if USE_BIO_MERGE
	struct req_iterator rq_iter;
	unsigned long long sector = ULLONG_MAX;
	unsigned long rq_len = 0;
	int rw_flag = 0;
#endif

	pr_info("%s START request in direction %d\n", __FUNCTION__, rq_data_dir(req));

#if USE_BIO_MERGE==1
	//spin_unlock_irq(rq->queue_lock);
	IS_IDLE = 0;
	__rq_for_each_bio(rq_iter.bio, req) {
		if(!bio_segments(rq_iter.bio))
			continue;
		nand_xfer_bio(rq, dev, rq_iter.bio, rq_iter.i);
	}
#if NAND_TEST_TICK
	printk("[N]ticks=%ld\n",nand_rw_time);
#endif

#ifdef NAND_CACHE_FLUSH_EVERY_SEC
	if(req->cmd_flags & REQ_WRITE)
		after_write = 1;
	if(req->cmd_flags & REQ_SYNC)
		wake_up_interruptible(&collect_arg.wait);
#endif

	//spin_lock_irq(rq->queue_lock);
	__blk_end_request_all(req, 0);
	req = NULL;

#elif USE_BIO_MERGE==2
	//IS_IDLE = 0;

	rw_flag = req->cmd_flags & REQ_WRITE;

	__rq_for_each_bio(rq_iter.bio, req) {
		if (!bio_segments(rq_iter.bio))
			continue;
		if (unlikely(sector == ULLONG_MAX))
			/*new bio, no data exists*/
			sector = (rq_iter.bio)->bi_sector;
		else {
			/*last bio data exists*/
			if ((rq_iter.bio)->bi_sector == (sector + (rq_len>>9))) {
				//printk("[N]bio merge\n");
			}
			else {
				/*flush last bio data here*/
				spin_unlock_irq(rq->queue_lock);
				down(&nandr->nand_ops_mutex);
#if NAND_TEST_TICK
				tick = jiffies;
				nand_transfer(dev, sector,  rq_len>>9, buffer, rw_flag);
				nand_rw_time += jiffies - tick;
#else
				nand_transfer(dev, sector, rq_len>>9, buffer, rw_flag);
#endif // NAND_TEST_TICK
				up(&nandr->nand_ops_mutex);
				spin_lock_irq(rq->queue_lock);
				/*update new bio*/
				sector = (rq_iter.bio)->bi_sector;
				buffer = 0;
				rq_len = 0;
			}
		}
		nand_xfer_bio(rq, dev, rq_iter.bio, rq_iter.i);
	}

	if (rq_len) {
		spin_unlock_irq(rq->queue_lock);
		down(&nandr->nand_ops_mutex);
#if NAND_TEST_TICK
		tick = jiffies;
		nand_transfer(dev, sector,  rq_len>>9, buffer, rw_flag);
		nand_rw_time += jiffies - tick;
#else
		nand_transfer(dev, sector, rq_len>>9, buffer, rw_flag);
#endif // NAND_TEST_TICK
		up(&nandr->nand_ops_mutex);
		spin_lock_irq(rq->queue_lock);
		sector = ULLONG_MAX;
		rq_len = 0;
		buffer = NULL;
	}

#if NAND_TEST_TICK
	printk("[N]ticks=%ld\n",nand_rw_time);
#endif

#ifdef NAND_CACHE_FLUSH_EVERY_SEC
	if(rw_flag == REQ_WRITE)
		after_write = 1;
	if(req->cmd_flags&REQ_SYNC)
		wake_up_interruptible(&collect_arg.wait);
#endif

	//spin_lock_irq(rq->queue_lock);
	__blk_end_request_all(req,0);
	req = NULL;

#elif USE_BIO_MERGE==0

	nandr = dev->nandr;
	spin_unlock_irq(rq->queue_lock);
	down(&nandr->nand_ops_mutex);
	IS_IDLE = 0;

#ifdef NAND_BIO_ALIGN
	reset(req);
#endif // NAND_BIO_ALIGN

#if NAND_TEST_TICK
	tick = jiffies;
	res = cache_align_page_request(nandr, dev, req);
	nand_rw_time += jiffies - tick;
#else
	res = cache_align_page_request(nandr, dev, req);
#endif // NAND_TEST_TICK

	up(&nandr->nand_ops_mutex);
	IS_IDLE = 1;
	spin_lock_irq(rq->queue_lock);

	if(!__blk_end_request_cur(req, res)) {
		req = NULL;

#if NAND_TEST_TICK
		printk("[N]ticks=%ld\n",nand_rw_time);
#endif // NAND_TEST_TICK

	}
#endif // USE_BIO_MERGE

	pr_info("%s END", __FUNCTION__);

	return 0;
}

static void block_thread_signals(sigset_t* old)
{
	spin_lock_irq(&current->sighand->siglock);
	*old = current->blocked;

	sigfillset(&current->blocked);

#ifdef THREAD_CAPTURES_SIGNALS
	sigdelset(&current->blocked, SIGKILL);
	sigdelset(&current->blocked, SIGSTOP);
	sigdelset(&current->blocked, SIGINT);
#endif // THREAD_CAPTURES_SIGNALS

	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);
}

#ifdef UNBLOCK_SIGNALS
static void unblock_thread_signals(sigset_t* old)
{
	spin_lock_irq(&current->sighand->siglock);
	current->blocked = *old;
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);
}
#endif // UNBLOCK_SIGNALS

static int nand_blktrans_thread(void *arg)
{
	struct nand_blk_ops *nandr = arg;
	struct request_queue *rq = nandr->rq;
	struct request *req = NULL;
	sigset_t saved_signals;

#ifdef THREAD_CAPTURES_SIGNALS
	int ret = 0;
#endif // THREAD_CAPTURES_SIGNALS

	/* we might get involved when memory gets low, so use PF_MEMALLOC */
	current->flags |= PF_MEMALLOC | PF_NOFREEZE;
	daemonize("%sd", nandr->name);

	pr_info("%s BEGIN\n", __FUNCTION__);

	// Why does one need to block signals and why unblocked signals may
	// cause a kernel oops?
	block_thread_signals(&saved_signals);

	// A pre-condition for the wait queue.
	spin_lock_irq(rq->queue_lock);

	while (!nandr->quit) {
//		wait_event_interruptible_locked_irq(nandr->thread_wq,
//		                                    (req = blk_fetch_request(rq)));

		// Inlined wait_event_interruptible_locked_irq() using the
		// request queue lock instead of the wait queue lock.
		DEFINE_WAIT(wait);
		while (!(req = blk_fetch_request(rq))) {
			if (likely(list_empty(&wait.task_list)))
				add_wait_queue(&nandr->thread_wq, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
#ifdef THREAD_CAPTURES_SIGNALS
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}
#endif // THREAD_CAPTURES_SIGNALS
			spin_unlock_irq(rq->queue_lock);
			schedule();
			spin_lock_irq(rq->queue_lock);
		}
		__remove_wait_queue(&nandr->thread_wq, &wait);
		__set_current_state(TASK_RUNNING);
		// End of the modified wait_event_interruptible_locked_irq().

		if (likely(req))
			nand_xfer_request(rq, req);

#ifdef THREAD_CAPTURES_SIGNALS
		if (unlikely(ret))
			break;
#endif // THREAD_CAPTURES_SIGNALS
	}
	if (req)
		__blk_end_request_all(req, -EIO);
	spin_unlock_irq(rq->queue_lock);

	pr_info("%s END\n", __FUNCTION__);

//	unblock_thread_signals(&saved_signals);    // May be unnecessary.
	complete_and_exit(&nandr->thread_exit, 0);
}


static void nand_blk_request(struct request_queue *rq)
{
	struct nand_blk_ops *nandr = rq->queuedata;
	wake_up(&nandr->thread_wq);
}

static int nand_open(struct block_device *bdev, fmode_t mode)
{
	struct nand_blk_dev *dev;
	struct nand_blk_ops *nandr;
	int ret = -ENODEV;

	dev = bdev->bd_disk->private_data;
	nandr = dev->nandr;

	if (!try_module_get(nandr->owner))
		goto out;

	ret = 0;
	if (nandr->open && (ret = nandr->open(dev))) {
		out:
		module_put(nandr->owner);
	}
	return ret;
}
static int nand_release(struct gendisk *disk, fmode_t mode)
{
	struct nand_blk_dev *dev;
	struct nand_blk_ops *nandr;

	int ret = 0;

	dev = disk->private_data;
	nandr = dev->nandr;
	//nand_flush(NULL);
	if (nandr->release)
		ret = nandr->release(dev);

	if (!ret) {
		module_put(nandr->owner);
	}

	return ret;
}


/*filp->f_dentry->d_inode->i_bdev->bd_disk->fops->ioctl(filp->f_dentry->d_inode, filp, cmd, arg);*/
#define DISABLE_WRITE        _IO('V',0)
#define ENABLE_WRITE         _IO('V',1)
#define DISABLE_READ 	     _IO('V',2)
#define ENABLE_READ 	     _IO('V',3)
static int nand_ioctl(struct block_device *bdev, fmode_t mode, unsigned int cmd, unsigned long arg)
{
	struct nand_blk_dev *dev = bdev->bd_disk->private_data;
	struct nand_blk_ops *nandr = dev->nandr;

	switch (cmd) {
	case BLKFLSBUF:
		dbg_err("BLKFLSBUF called!\n");
		if (nandr->flush)
			return nandr->flush(dev);
		/* The core code did the work, we had nothing to do. */
		return 0;

	case HDIO_GETGEO:
		if (nandr->getgeo) {
			struct hd_geometry g;
			int ret;

			memset(&g, 0, sizeof(g));
			ret = nandr->getgeo(dev, &g);
			if (ret)
				return ret;
  			dbg_err("HDIO_GETGEO called!\n");
			g.start = get_start_sect(bdev);
			if (copy_to_user((void __user *)arg, &g, sizeof(g)))
				return -EFAULT;

			return 0;
		}
		return 0;
	case ENABLE_WRITE:
		dbg_err("enable write!\n");
		dev->disable_access = 0;
		dev->readonly = 0;
		set_disk_ro(dev->blkcore_priv, 0);
		return 0;

	case DISABLE_WRITE:
		dbg_err("disable write!\n");
		dev->readonly = 1;
		set_disk_ro(dev->blkcore_priv, 1);
		return 0;

	case ENABLE_READ:
		dbg_err("enable read!\n");
		dev->disable_access = 0;
		dev->writeonly = 0;
		return 0;

	case DISABLE_READ:
		dbg_err("disable read!\n");
		dev->writeonly = 1;
		return 0;
	default:
		return -ENOTTY;
	}
}

struct block_device_operations nand_blktrans_ops = {
	.owner		= THIS_MODULE,
	.open		= nand_open,
	.release	= nand_release,
	.ioctl		= nand_ioctl,
};

void set_part_mod(char *name,int cmd)
{
	struct file *filp = NULL;
	filp = filp_open(name, O_RDWR, 0);
	filp->f_dentry->d_inode->i_bdev->bd_disk->fops->ioctl(filp->f_dentry->d_inode->i_bdev, 0, cmd, 0);
	filp_close(filp, current->files);
}

static int nand_add_dev(struct nand_blk_ops *nandr)
{
	struct nand_blk_dev *dev;
	struct gendisk *gd;
	unsigned long temp;

	if (!down_trylock(&nand_mutex)) {
		up(&nand_mutex);
		BUG();
	}

	dev = &nandr->dev;
	if (dev->blkcore_priv)
		return -EBUSY;

	memset(dev, 0, sizeof(*dev));
	dev->nandr = nandr;
	dev->size = DiskSize;
	dev->off_size = 0;
	dev->devnum = 0;

	dev->cylinders = 1024;
	dev->heads = 16;

	temp = dev->cylinders * dev->heads;
	dev->sectors = ( dev->size) / temp;
	if ((dev->size) % temp) {
		dev->sectors++;
		temp = dev->cylinders * dev->sectors;
		dev->heads = (dev->size)  / temp;

		if ((dev->size)   % temp) {
			dev->heads++;
			temp = dev->heads * dev->sectors;
			dev->cylinders = (dev->size)  / temp;
		}
	}

	gd = alloc_disk(1 << nandr->minorbits);
	if (!gd) {
		return -ENOMEM;
	}
	gd->major = nandr->major;
	gd->first_minor = (dev->devnum) << nandr->minorbits;
	gd->fops = &nand_blktrans_ops;

	/* /dev/nand */
	snprintf(gd->disk_name, sizeof(gd->disk_name),
		 "%s", nandr->name);

	/* 2.5 has capacity in units of 512 bytes while still
	   having BLOCK_SIZE_BITS set to 10. Just to keep us amused. */
	set_capacity(gd, dev->size);

	gd->private_data = dev;
	dev->blkcore_priv = gd;
	gd->queue = nandr->rq;

	if (dev->readonly)
		set_disk_ro(gd, 1);
	add_disk(gd);
	return 0;
}

static int nand_remove_dev(struct nand_blk_dev *dev)
{
	struct gendisk *gd;

	if (!down_trylock(&nand_mutex)) {
		up(&nand_mutex);
		BUG();
	}
	gd = dev->blkcore_priv;
	dev->blkcore_priv = NULL;
	gd->queue = NULL;
	del_gendisk(gd);
	put_disk(gd);
	return 0;
}


#ifdef NAND_CACHE_FLUSH_EVERY_SEC
static int collect_thread(void *tmparg)
{
	unsigned long ret;
	struct collect_ops *arg = tmparg;

	current->flags |= PF_MEMALLOC | PF_NOFREEZE;
	daemonize("%sd", "nfmt");

	pr_info("%s started\n", __FUNCTION__);

	/*
	spin_lock_irq(&current->sighand->siglock);
	sigfillset(&current->blocked);
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);
	*/
#if 0
	while (!arg->quit)
	{
		ret = wait_event_interruptible_timeout(arg->wait, 0, arg->timeout*HZ);
		if (0 ==  ret)
		{
			nand_flush(NULL);
			IS_IDLE = 1;
		}
		arg->timeout = TIMEOUT;
	}
#else
	while (!arg->quit){
		ret = wait_event_interruptible(arg->wait, after_write);
		if(ret==0) {
			do {
				after_write = 0;
				ssleep(arg->timeout);
			} while(after_write);
			//IS_IDLE = 1;
			nand_flush(NULL);
			//IS_IDLE = 1;
		}
	}
#endif // 0
	complete_and_exit(&arg->thread_exit, 0);
}
#endif // NAND_CACHE_FLUSH_EVERY_SEC


int nand_blk_register(struct nand_blk_ops *nandr)
{
	int ret;

	down(&nand_mutex);

	ret = register_blkdev(nandr->major, nandr->name);
	if (ret < 0) {
		pr_err("%s: failed to register block device\n", __FUNCTION__);
		goto out;
	}

	spin_lock_init(&nandr->queue_lock);
	init_completion(&nandr->thread_exit);
	init_waitqueue_head(&nandr->thread_wq);
	sema_init(&nandr->nand_ops_mutex, 1);

	nandr->rq = blk_init_queue(nand_blk_request, &nandr->queue_lock);
	if (IS_ERR(nandr->rq)) {
		pr_err("%s: init queue error\n", __FUNCTION__);
		ret = PTR_ERR(nandr->rq);
		goto undo_blkdev;
	}

	//for 2.6.29
	//elevator_exit(nandr->rq->elevator);
	//for 2.6.36
	//null

	ret = elevator_change(nandr->rq, "noop");
	if (ret < 0) {
		pr_err("%s: elevator_change error %d\n", __FUNCTION__, ret);
		goto undo_queue;
	}

	nandr->rq->queuedata = nandr;

	ret = kernel_thread(nand_blktrans_thread, nandr, CLONE_KERNEL);
	if (ret < 0) {
		pr_err("%s: worker thread creation ERROR %d\n", __FUNCTION__, ret);
		goto undo_queue;
	}
	else
		pr_info("%s: worker thread created\n", __FUNCTION__);

#ifdef NAND_CACHE_FLUSH_EVERY_SEC
	collect_arg.quit = 0;
	collect_arg.timeout = TIMEOUT;
	init_completion(&collect_arg.thread_exit);
	init_waitqueue_head(&collect_arg.wait);
	ret = kernel_thread(collect_thread, &collect_arg, CLONE_KERNEL);
 	if (ret < 0) {
		pr_err("%s: collector thread creation ERROR %d\n", __FUNCTION__, ret);
		goto undo_queue;
	}
	else
		pr_info("%s: collector thread created\n", __FUNCTION__);
#endif

	//devfs_mk_dir(nandr->name);

	memset(&nandr->dev, 0, sizeof(nandr->dev));
	ret = nandr->add_dev(nandr);
	pr_info("%s: device added with status %d\n", __FUNCTION__, ret);
	if (ret < 0)
		goto undo_queue;

	goto out;

  undo_queue:
	blk_cleanup_queue(nandr->rq);
  undo_blkdev:
	unregister_blkdev(nandr->major, nandr->name);
  out:
	up(&nand_mutex);
	return ret;
}


void nand_blk_unregister(struct nand_blk_ops *nandr)
{
	pr_info("%s BEGIN", __FUNCTION__);
	down(&nand_mutex);
	/* Clean up the kernel thread */
	nandr->quit = 1;
	wake_up(&nandr->thread_wq);
	wait_for_completion(&nandr->thread_exit);

#ifdef NAND_CACHE_FLUSH_EVERY_SEC
	collect_arg.quit =1;
	wake_up(&collect_arg.wait);
	wait_for_completion(&collect_arg.thread_exit);
#endif
	/* Remove it from the list of active majors */
	nandr->remove_dev(&nandr->dev);

	//devfs_remove(nandr->name);
	blk_cleanup_queue(nandr->rq);

	unregister_blkdev(nandr->major, nandr->name);

	up(&nand_mutex);
	pr_info("%s END", __FUNCTION__);
}

static int nand_getgeo(struct nand_blk_dev *dev,  struct hd_geometry *geo)
{
	geo->heads = dev->heads;
	geo->sectors = dev->sectors;
	geo->cylinders = dev->cylinders;

	return 0;
}

static struct nand_blk_ops mytr = {
	.quit        = 0,
	.name        =  "nand",
	.major       = 93,
	.minorbits   = 4, /* ⌈log₂(MAX_PART_COUNT + 1)⌉ */
	.getgeo      = nand_getgeo,
	.add_dev     = nand_add_dev,
	.remove_dev  = nand_remove_dev,
	.flush       = nand_flush,
	.owner       = THIS_MODULE,
};


static int nand_flush(struct nand_blk_dev *dev)
{
	if (!down_trylock(&mytr.nand_ops_mutex)) {
		IS_IDLE = 0;
#ifdef NAND_CACHE_RW
		NAND_CacheFlush();
#else
		LML_FlushPageCache();
#endif
		up(&mytr.nand_ops_mutex);
		IS_IDLE = 1;

		dbg_inf("nand_flush \n");
	}
	else
		pr_info("%s: cannot lock mutex\n", __FUNCTION__);
	return 0;
}


static void nand_flush_all(void)
{
	int     timeout = 0;

#ifdef NAND_CACHE_FLUSH_EVERY_SEC
	/* wait write finish */
	for (timeout=0; timeout<10; timeout++) {
		if (after_write)
			msleep(500);
		else
			break;
	}
#endif // NAND_CACHE_FLUSH_EVERY_SEC

	pr_info("%s: NAND flush all after a delay of %d ms\n",
		__FUNCTION__, timeout * 500);

	/* get nand ops mutex */
	down(&mytr.nand_ops_mutex);

#ifdef NAND_CACHE_RW
	NAND_CacheFlush();
#else
	LML_FlushPageCache();
#endif
	BMM_WriteBackAllMapTbl();
	pr_info("%s: NAND flush all complete\n", __FUNCTION__);
}


int cal_partoff_within_disk(char *name,struct inode *i)
{
	struct gendisk *gd = i->i_bdev->bd_disk;
	int current_minor = MINOR(i->i_bdev->bd_dev)  ;
	int index = current_minor & ((1<<mytr.minorbits) - 1) ;
	if(!index)
		return 0;
	return ( gd->part_tbl->part[ index - 1]->start_sect);
}

void set_nand_pio(void)
{
#ifndef USE_SYS_PIN
	__u32	cfg0;
	__u32	cfg1;
	__u32	cfg2;

	void* gpio_base;

	/*
	  gpio_base = ioremap(PIOC_REGS_pBASE, 4096 );
	  if (gpio_base == NULL) {
	  printk(KERN_ERR "gpio failed to remap register block\n");
	  return ;
	  }
	*/
	//modify for f20
	gpio_base = (void *)SW_VA_PORTC_IO_BASE;

	cfg0 = *(volatile __u32 *)(gpio_base + 0x48);
	cfg1 = *(volatile __u32 *)(gpio_base + 0x4c);
	cfg2 = *(volatile __u32 *)(gpio_base + 0x50);

	/*set PIOC for nand*/
	cfg0 &= 0x0;
	cfg0 |= 0x22222222;
	cfg1 &= 0x0;
	cfg1 |= 0x22222222;
	cfg2 &= 0x0;
	cfg2 |= 0x22222222;

	*(volatile __u32 *)(gpio_base + 0x48) = cfg0;
	*(volatile __u32 *)(gpio_base + 0x4c) = cfg1;
	*(volatile __u32 *)(gpio_base + 0x50) = cfg2;

	//iounmap(gpio_base);
#else
	nand_handle = gpio_request_ex("nand_para",NULL);
#endif
}

void release_nand_pio(void)
{

	#ifndef USE_SYS_PIN
			void* gpio_base;

			/*
			gpio_base = ioremap(PIOC_REGS_pBASE, 4096 );
			if (gpio_base == NULL) {
				printk(KERN_ERR "gpio failed to remap register block\n");
				return ;
			}
			*/
			//modify for f20
			gpio_base = (void *)SW_VA_PORTC_IO_BASE;



			*(volatile __u32 *)(gpio_base + 0x48) = 0;
			*(volatile __u32 *)(gpio_base + 0x4c) = 0;
			*(volatile __u32 *)(gpio_base + 0x50) = 0;

			//iounmap(gpio_base);
	#else
			//printk("[NAND] nand gpio_release\n");
			//gpio_release("nand_para",NULL);
	#endif
}


#ifndef USE_SYS_CLK
__u32 get_cmu_clk(void)
{
	__u32 reg_val;
	__u32 div_p, factor_n;
	__u32 factor_k, factor_m;
	__u32 clock;

	reg_val  = *(volatile unsigned int *)(0xf1c20000 + 0x20);
	div_p    = (reg_val >> 16) & 0x3;
	factor_n = (reg_val >> 8) & 0x1f;
	factor_k = ((reg_val >> 4) & 0x3) + 1;
	factor_m = ((reg_val >> 0) & 0x3) + 1;

	clock = 24 * factor_n * factor_k/div_p/factor_m;

	return clock;
}

void set_nand_clock(__u32 nand_max_clock)
{
	__u32 edo_clk, cmu_clk;
	__u32 cfg;
	__u32 nand_clk_divid_ratio;

	/*open ahb nand clk */
	cfg = *(volatile __u32 *)(0xf1c20000 + 0x60);
	cfg |= (0x1<<13);
	*(volatile __u32 *)(0xf1c20000 + 0x60) = cfg;

	/*set nand clock*/
	//edo_clk = (nand_max_clock > 20)?(nand_max_clock-10):nand_max_clock;
	edo_clk = nand_max_clock * 2;

	cmu_clk = get_cmu_clk( );
	nand_clk_divid_ratio = cmu_clk / edo_clk;
	if (cmu_clk % edo_clk)
			nand_clk_divid_ratio++;
	if (nand_clk_divid_ratio){
		if (nand_clk_divid_ratio > 16)
			nand_clk_divid_ratio = 15;
		else
			nand_clk_divid_ratio--;
	}
	/*set nand clock gate on*/
	cfg = *(volatile __u32 *)(0xf1c20000 + 0x80);

	/*gate on nand clock*/
	cfg |= (1U << 31);
	/*take cmu pll as nand src block*/
	cfg &= ~(0x3 << 24);
	cfg |=  (0x2 << 24);
	//set divn = 0
	cfg &= ~(0x03 << 12);

	/*set ratio*/
	cfg &= ~(0x0f << 0);
	cfg |= (nand_clk_divid_ratio & 0xf) << 0;

	*(volatile __u32 *)(0xf1c20000 + 0x80) = cfg;
}


void release_nand_clock(void)
{
	__u32 cfg;
	__u32 ccmu_base;

	ccmu_base = 0xf1c20000;

	/*set nand clock gate on*/
	cfg = *(volatile __u32 *)(ccmu_base + 0x14);
	cfg &= (~(0x1<<15));
	*(volatile __u32 *)(ccmu_base + 0x14) = cfg;
}

void active_nand_clock(void)
{
	__u32 cfg;
	__u32 ccmu_base;

	ccmu_base = 0xf1c20000;

	/*set nand clock gate on*/
	cfg = *(volatile __u32 *)(ccmu_base + 0x14);
	cfg |= (0x1<<15);
	*(volatile __u32 *)(ccmu_base + 0x14) = cfg;
}
#else

int nand_request_clk(void)
{
	ahb_nand_clk = clk_get(NULL,"ahb_nfc");
	if(!ahb_nand_clk) {
		return -1;
	}
	mod_nand_clk = clk_get(NULL,"nfc");
		if(!mod_nand_clk) {
		return -1;
	}
	return 0;
}

void nand_release_clk(void)
{
	clk_put(ahb_nand_clk);
	clk_put(mod_nand_clk);
}


int nand_ahb_clk_enable(void)
{
	return clk_enable(ahb_nand_clk);
}

int nand_module_clk_enable(void)
{
	return clk_enable(mod_nand_clk);
}

void nand_ahb_clk_disable(void)
{
		clk_disable(ahb_nand_clk);
}

void nand_module_clk_disable(void)
{
	clk_disable(mod_nand_clk);
}

int nand_set_module_clk(__u32 nand_clk)
{
	return clk_set_rate(mod_nand_clk, nand_clk*2);
}

__u32 nand_get_module_clk(void)
{
	return clk_get_rate(mod_nand_clk);
}


#endif

#ifndef CONFIG_SUNXI_NAND_TEST
static int __init init_blklayer(void)
{
	int ret;
	unsigned long irqflags;

#ifndef USE_SYS_CLK
	__u32 nand_clk;
	set_nand_clock(20);
#else
	ret = nand_request_clk();
	if(ret) {
		printk("[NAND] nand_request_clk fail \n");
		return -1;
	}

	ret = nand_ahb_clk_enable();
	if(ret) {
		printk("[NAND] nand_ahb_clk_enable fail \n");
		return -1;
	}

	ret = nand_module_clk_enable();
	if(ret) {
		printk("[NAND] nand_module_clk_enable fail \n");
		return -1;
	}

	ret = nand_set_module_clk(20000000);
	if(ret) {
		printk("[NAND] nand_set_module_clk fail \n");
		return -1;
	}

#endif
	set_nand_pio();
	clear_NAND_ZI();

	printk("[NAND] nand driver version: 0x%x 0x%x \n", NAND_VERSION_0,NAND_VERSION_1);
	NAND_ClearRbInt();
	spin_lock_init(&nand_rb_lock);
	irqflags = IRQF_DISABLED;

	if (request_irq(SW_INT_IRQNO_NAND, nand_rb_interrupt, irqflags, mytr.name, &mytr)) {
		printk("nand interrupte register error\n");
		return -EAGAIN;
	}

	ret = PHY_Init();
	if (ret) {
		PHY_Exit();
		return -1;
	}

	ret = SCN_AnalyzeNandSystem();
	if (ret < 0)
		return ret;

	//set nand clk
#ifndef USE_SYS_CLK
	nand_clk =NandStorageInfo.FrequencePar;
	if(nand_clk>30)
		nand_clk = 30;
	set_nand_clock(nand_clk);
	dbg_inf("set nand clk to %x \n", nand_clk);
#else
	ret = nand_set_module_clk((NandStorageInfo.FrequencePar)*1000000);
	if(ret) {
		printk("[NAND] nand_set_module_clk fail \n");
		return -1;
	}
#endif

	ret = PHY_ChangeMode(1);
	if (ret < 0)
		return ret;

	ret = PHY_ScanDDRParam();
	if (ret < 0)
		return ret;

	ret = FMT_Init();
	if (ret < 0)
		return ret;

	ret = FMT_FormatNand();
	if (ret < 0)
		return ret;
	FMT_Exit();

	/*init logic layer*/
	ret = LML_Init();
	if (ret < 0)
		return ret;

#ifdef NAND_CACHE_RW
	NAND_CacheOpen();
#endif

	return nand_blk_register(&mytr);
}

static void  __exit exit_blklayer(void)
{
	nand_flush(NULL);
	nand_blk_unregister(&mytr);
	LML_Exit();
	FMT_Exit();
	PHY_Exit();
#ifdef NAND_CACHE_RW
	NAND_CacheClose();
#endif
}

#else
static int __init init_blklayer(void)
{
	printk("[NAND] for nand test, init_blklayer \n");
	return 0;
}

static void  __exit exit_blklayer(void)
{

}
#endif

#ifdef CONFIG_SUNXI_NAND_TEST
int nand_suspend(struct platform_device *plat_dev, pm_message_t state)
#else
static int nand_suspend(struct platform_device *plat_dev, pm_message_t state)
#endif
{
	int i=0;

	printk("[NAND] nand_suspend \n");

	if(!IS_IDLE){
		for(i=0;i<10;i++){
			msleep(200);
			if(IS_IDLE)
				break;
		}
	}
	if(i==10){
		return -EBUSY;
	}else{
		down(&mytr.nand_ops_mutex);
		#ifndef USE_SYS_CLK
			release_nand_clock();
		#else
			nand_module_clk_disable();
		#endif

		release_nand_pio();
		printk("[NAND] nand_suspend ok \n");
		return 0;
	}
}

#ifdef CONFIG_SUNXI_NAND_TEST
int nand_resume(struct platform_device *plat_dev)
#else
static int nand_resume(struct platform_device *plat_dev)
#endif
{

	printk("[NAND] nand_resume \n");
	set_nand_pio();

	#ifndef USE_SYS_CLK
		active_nand_clock();
	#else
		nand_module_clk_enable();
	#endif

	up(&mytr.nand_ops_mutex);


	return 0;
}

static int nand_probe(struct platform_device *plat_dev)
{
	return 0;
}

static int nand_remove(struct platform_device *plat_dev)
{
	return 0;
}

void nand_shutdown(struct platform_device *plat_dev)
{
	pr_info("%s", __FUNCTION__);
	nand_flush_all();
}

static struct platform_driver nand_driver = {
	.probe = nand_probe,
	.remove = nand_remove,
	.shutdown =  nand_shutdown,
	.suspend = nand_suspend,
	.resume = nand_resume,
	.driver = {
		.name = "sw_nand",
		.owner = THIS_MODULE,
	}
};

/****************************************************************************
 *
 * Module stuff
 *
 ****************************************************************************/

#ifndef CONFIG_SUNXI_NAND_TEST

static int __init nand_init(void)
{
	s32 ret;
	int nand_used = 0;

	ret = script_parser_fetch("nand_para","nand_used", &nand_used, sizeof(int));
	if (ret)
		printk("nand init fetch emac using configuration failed\n");

	if(nand_used == 0) {
		printk("nand driver is disabled \n");
		return 0;
	}

	ret = init_blklayer();
	if(ret) {
		dbg_err("init_blklayer fail \n");
		return -1;
	}

	ret = platform_driver_register(&nand_driver);
	if(ret) {
		dbg_err("platform_driver_register fail \n");
		return -1;
	}
	pr_info("%s OK\n", __FUNCTION__);
	return 0;
}

static void __exit nand_exit(void)
{
	s32 ret;
	int nand_used = 0;

	ret = script_parser_fetch("nand_para", "nand_used", &nand_used, sizeof(int));
	if (ret)
		printk("nand init fetch emac using configuration failed\n");

	if(nand_used == 0) {
		printk("nand driver is disabled \n");
		return;
	}

//	mytr->quit = 1; // Flag the child threads to exit their busy loops.

	pr_info("%s: thread busy loop quit flag value is %d\n", __FUNCTION__, mytr.quit);
	platform_driver_unregister(&nand_driver);
	//platform_device_unregister(&nand_device);
	exit_blklayer();
}

#else // CONFIG_SUNXI_NAND_TEST

static int __init nand_init(void)
{
	return nand_test_init();
}

static void __exit nand_exit(void)
{
	nand_test_exit();
}

#endif // CONFIG_SUNXI_NAND_TEST


module_init(nand_init);
module_exit(nand_exit);
MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("nand flash groups");
MODULE_DESCRIPTION ("Generic NAND flash driver code");
