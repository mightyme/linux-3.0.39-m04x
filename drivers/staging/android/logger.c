/*
 * drivers/misc/logger.c
 *
 * A Logging Subsystem
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * Robert Love <rlove@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/cma.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/rtc.h>
#include <linux/ctype.h>
#include "logger.h"

#include <asm/ioctls.h>

#define log_w_off() (*(log->w_off))
#define log_head() (*(log->head))
#define log_size() (*(log->size))
#define LOGGER_SIG (0x35490214)

/*
 * struct logger_log - represents a specific log, such as 'main' or 'radio'
 *
 * This structure lives from module insertion until module removal, so it does
 * not need additional reference counting. The structure is protected by the
 * mutex 'mutex'.
 */
struct logger_log {
	unsigned char 		*buffer;/* the ring buffer itself */
	struct miscdevice	misc;	/* misc device representing the log */
	wait_queue_head_t	wq;	/* wait queue for readers */
	struct list_head	readers; /* this log's readers */
	struct mutex		mutex;	/* mutex protecting buffer */
	size_t			*w_off;	/* current write head offset */
	size_t			*head;	/* new readers start here */
	size_t			*size;	/* size of the log */
};

/*
 * struct logger_reader - a logging device open for reading
 *
 * This object lives from open to release, so we don't need additional
 * reference counting. The structure is protected by log->mutex.
 */
struct logger_reader {
	struct logger_log	*log;	/* associated log */
	struct list_head	list;	/* entry in logger_log's list */
	size_t			r_off;	/* current read head offset */
};

/* logger_offset - returns index 'n' into the log via (optimized) modulus */
#define logger_offset(n)	((n) & (log_size() - 1))

/*
 * file_get_log - Given a file structure, return the associated log
 *
 * This isn't aesthetic. We have several goals:
 *
 * 	1) Need to quickly obtain the associated log during an I/O operation
 * 	2) Readers need to maintain state (logger_reader)
 * 	3) Writers need to be very fast (open() should be a near no-op)
 *
 * In the reader case, we can trivially go file->logger_reader->logger_log.
 * For a writer, we don't want to maintain a logger_reader, so we just go
 * file->logger_log. Thus what file->private_data points at depends on whether
 * or not the file was opened for reading. This function hides that dirtiness.
 */
static inline struct logger_log *file_get_log(struct file *file)
{
	if (file->f_mode & FMODE_READ) {
		struct logger_reader *reader = file->private_data;
		return reader->log;
	} else
		return file->private_data;
}

/*
 * get_entry_len - Grabs the length of the payload of the next entry starting
 * from 'off'.
 *
 * Caller needs to hold log->mutex.
 */
static __u32 get_entry_len(struct logger_log *log, size_t off)
{
	__u16 val;

	switch (log_size() - off) {
	case 1:
		memcpy(&val, log->buffer + off, 1);
		memcpy(((char *) &val) + 1, log->buffer, 1);
		break;
	default:
		memcpy(&val, log->buffer + off, 2);
	}

	return sizeof(struct logger_entry) + val;
}

/*
 * do_read_log_to_user - reads exactly 'count' bytes from 'log' into the
 * user-space buffer 'buf'. Returns 'count' on success.
 *
 * Caller must hold log->mutex.
 */
static ssize_t do_read_log_to_user(struct logger_log *log,
				   struct logger_reader *reader,
				   char __user *buf,
				   size_t count)
{
	size_t len;

	/*
	 * We read from the log in two disjoint operations. First, we read from
	 * the current read head offset up to 'count' bytes or to the end of
	 * the log, whichever comes first.
	 */
	len = min(count, log_size() - reader->r_off);
	if (copy_to_user(buf, log->buffer + reader->r_off, len))
		return -EFAULT;

	/*
	 * Second, we read any remaining bytes, starting back at the head of
	 * the log.
	 */
	if (count != len)
		if (copy_to_user(buf + len, log->buffer, count - len))
			return -EFAULT;

	reader->r_off = logger_offset(reader->r_off + count);

	return count;
}

/*
 * logger_read - our log's read() method
 *
 * Behavior:
 *
 * 	- O_NONBLOCK works
 * 	- If there are no log entries to read, blocks until log is written to
 * 	- Atomically reads exactly one log entry
 *
 * Optimal read size is LOGGER_ENTRY_MAX_LEN. Will set errno to EINVAL if read
 * buffer is insufficient to hold next entry.
 */
static ssize_t logger_read(struct file *file, char __user *buf,
			   size_t count, loff_t *pos)
{
	struct logger_reader *reader = file->private_data;
	struct logger_log *log = reader->log;
	ssize_t ret;
	DEFINE_WAIT(wait);

start:
	while (1) {
		prepare_to_wait(&log->wq, &wait, TASK_INTERRUPTIBLE);

		mutex_lock(&log->mutex);
		ret = (log_w_off() == reader->r_off);
		mutex_unlock(&log->mutex);
		if (!ret)
			break;

		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}

		schedule();
	}

	finish_wait(&log->wq, &wait);
	if (ret)
		return ret;

	mutex_lock(&log->mutex);

	/* is there still something to read or did we race? */
	if (unlikely(log_w_off() == reader->r_off)) {
		mutex_unlock(&log->mutex);
		goto start;
	}

	/* get the size of the next entry */
	ret = get_entry_len(log, reader->r_off);
	if (count < ret) {
		ret = -EINVAL;
		goto out;
	}

	/* get exactly one entry from the log */
	ret = do_read_log_to_user(log, reader, buf, ret);

out:
	mutex_unlock(&log->mutex);

	return ret;
}

/*
 * get_next_entry - return the offset of the first valid entry at least 'len'
 * bytes after 'off'.
 *
 * Caller must hold log->mutex.
 */
static size_t get_next_entry(struct logger_log *log, size_t off, size_t len)
{
	size_t count = 0;

	do {
		size_t nr = get_entry_len(log, off);
		off = logger_offset(off + nr);
		count += nr;
	} while (count < len);

	return off;
}

/*
 * clock_interval - is a < c < b in mod-space? Put another way, does the line
 * from a to b cross c?
 */
static inline int clock_interval(size_t a, size_t b, size_t c)
{
	if (b < a) {
		if (a < c || b >= c)
			return 1;
	} else {
		if (a < c && b >= c)
			return 1;
	}

	return 0;
}

/*
 * fix_up_readers - walk the list of all readers and "fix up" any who were
 * lapped by the writer; also do the same for the default "start head".
 * We do this by "pulling forward" the readers and start head to the first
 * entry after the new write head.
 *
 * The caller needs to hold log->mutex.
 */
static void fix_up_readers(struct logger_log *log, size_t len)
{
	size_t old = log_w_off();
	size_t new = logger_offset(old + len);
	struct logger_reader *reader;

	if (clock_interval(old, new, log_head()))
		log_head() = get_next_entry(log, log_head(), len);

	list_for_each_entry(reader, &log->readers, list)
		if (clock_interval(old, new, reader->r_off))
			reader->r_off = get_next_entry(log, reader->r_off, len);
}

/*
 * do_write_log - writes 'len' bytes from 'buf' to 'log'
 *
 * The caller needs to hold log->mutex.
 */
static void do_write_log(struct logger_log *log, const void *buf, size_t count)
{
	size_t len;

	len = min(count, log_size() - log_w_off());
	memcpy(log->buffer + log_w_off(), buf, len);

	if (count != len)
		memcpy(log->buffer, buf + len, count - len);

	log_w_off() = logger_offset(log_w_off() + count);

}

/*
 * do_write_log_user - writes 'len' bytes from the user-space buffer 'buf' to
 * the log 'log'
 *
 * The caller needs to hold log->mutex.
 *
 * Returns 'count' on success, negative error code on failure.
 */
static ssize_t do_write_log_from_user(struct logger_log *log,
				      const void __user *buf, size_t count)
{
	size_t len;

	len = min(count, log_size() - log_w_off());
	if (len && copy_from_user(log->buffer + log_w_off(), buf, len))
		return -EFAULT;

	if (count != len)
		if (copy_from_user(log->buffer, buf + len, count - len))
			return -EFAULT;

	log_w_off() = logger_offset(log_w_off() + count);

	return count;
}

/*
 * logger_aio_write - our write method, implementing support for write(),
 * writev(), and aio_write(). Writes are our fast path, and we try to optimize
 * them above all else.
 */
ssize_t logger_aio_write(struct kiocb *iocb, const struct iovec *iov,
			 unsigned long nr_segs, loff_t ppos)
{
	struct logger_log *log = file_get_log(iocb->ki_filp);
	size_t orig = log_w_off();
	struct logger_entry header;
	struct timespec now;
	ssize_t ret = 0;

	now = current_kernel_time();

	header.pid = current->tgid;
	header.tid = current->pid;
	header.sec = now.tv_sec;
	header.nsec = now.tv_nsec;
	header.len = min_t(size_t, iocb->ki_left, LOGGER_ENTRY_MAX_PAYLOAD);

	/* null writes succeed, return zero */
	if (unlikely(!header.len))
		return 0;

	mutex_lock(&log->mutex);

	/*
	 * Fix up any readers, pulling them forward to the first readable
	 * entry after (what will be) the new write offset. We do this now
	 * because if we partially fail, we can end up with clobbered log
	 * entries that encroach on readable buffer.
	 */
	fix_up_readers(log, sizeof(struct logger_entry) + header.len);

	do_write_log(log, &header, sizeof(struct logger_entry));

	while (nr_segs-- > 0) {
		size_t len;
		ssize_t nr;

		/* figure out how much of this vector we can keep */
		len = min_t(size_t, iov->iov_len, header.len - ret);

		/* write out this segment's payload */
		nr = do_write_log_from_user(log, iov->iov_base, len);
		if (unlikely(nr < 0)) {
			log_w_off() = orig;
			mutex_unlock(&log->mutex);
			return nr;
		}

		iov++;
		ret += nr;
	}

	mutex_unlock(&log->mutex);

	/* wake up any blocked readers */
	wake_up_interruptible(&log->wq);

	return ret;
}

static struct logger_log *get_log_from_minor(int);

/*
 * logger_open - the log's open() file operation
 *
 * Note how near a no-op this is in the write-only case. Keep it that way!
 */
static int logger_open(struct inode *inode, struct file *file)
{
	struct logger_log *log;
	int ret;

	ret = nonseekable_open(inode, file);
	if (ret)
		return ret;

	log = get_log_from_minor(MINOR(inode->i_rdev));
	if (!log)
		return -ENODEV;

	if (file->f_mode & FMODE_READ) {
		struct logger_reader *reader;

		reader = kmalloc(sizeof(struct logger_reader), GFP_KERNEL);
		if (!reader)
			return -ENOMEM;

		reader->log = log;
		INIT_LIST_HEAD(&reader->list);

		mutex_lock(&log->mutex);
		reader->r_off = log_head();
		list_add_tail(&reader->list, &log->readers);
		mutex_unlock(&log->mutex);

		file->private_data = reader;
	} else
		file->private_data = log;

	return 0;
}

/*
 * logger_release - the log's release file operation
 *
 * Note this is a total no-op in the write-only case. Keep it that way!
 */
static int logger_release(struct inode *ignored, struct file *file)
{
	if (file->f_mode & FMODE_READ) {
		struct logger_reader *reader = file->private_data;
		list_del(&reader->list);
		kfree(reader);
	}

	return 0;
}

/*
 * logger_poll - the log's poll file operation, for poll/select/epoll
 *
 * Note we always return POLLOUT, because you can always write() to the log.
 * Note also that, strictly speaking, a return value of POLLIN does not
 * guarantee that the log is readable without blocking, as there is a small
 * chance that the writer can lap the reader in the interim between poll()
 * returning and the read() request.
 */
static unsigned int logger_poll(struct file *file, poll_table *wait)
{
	struct logger_reader *reader;
	struct logger_log *log;
	unsigned int ret = POLLOUT | POLLWRNORM;

	if (!(file->f_mode & FMODE_READ))
		return ret;

	reader = file->private_data;
	log = reader->log;

	poll_wait(file, &log->wq, wait);

	mutex_lock(&log->mutex);
	if (log_w_off() != reader->r_off)
		ret |= POLLIN | POLLRDNORM;
	mutex_unlock(&log->mutex);

	return ret;
}

static long logger_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct logger_log *log = file_get_log(file);
	struct logger_reader *reader;
	long ret = -ENOTTY;

	mutex_lock(&log->mutex);

	switch (cmd) {
	case LOGGER_GET_LOG_BUF_SIZE:
		ret = log_size();
		break;
	case LOGGER_GET_LOG_LEN:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;
		if (log_w_off() >= reader->r_off)
			ret = log_w_off() - reader->r_off;
		else
			ret = (log_size() - reader->r_off) + log_w_off();
		break;
	case LOGGER_GET_NEXT_ENTRY_LEN:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;
		if (log_w_off() != reader->r_off)
			ret = get_entry_len(log, reader->r_off);
		else
			ret = 0;
		break;
	case LOGGER_FLUSH_LOG:
		if (!(file->f_mode & FMODE_WRITE)) {
			ret = -EBADF;
			break;
		}
		list_for_each_entry(reader, &log->readers, list)
			reader->r_off = log_w_off();
		log_head() = log_w_off();
		ret = 0;
		break;
	}

	mutex_unlock(&log->mutex);

	return ret;
}

static const struct file_operations logger_fops = {
	.owner = THIS_MODULE,
	.read = logger_read,
	.aio_write = logger_aio_write,
	.poll = logger_poll,
	.unlocked_ioctl = logger_ioctl,
	.compat_ioctl = logger_ioctl,
	.open = logger_open,
	.release = logger_release,
};

#define LOGGER_LOG_SIZE	(CONFIG_ANDROID_LOGGER_MEMSIZE * SZ_1K)

struct logger_header {
	size_t	w_off;
	size_t	head;
	size_t 	size;
};

#define LOGGER_BUFFER_SIZE (LOGGER_LOG_SIZE * 4 + PAGE_SIZE)

struct logger_buffer {
	uint32_t	sig;
	struct logger_header log_head[4];
#define LOG_BUF_SIZE (LOGGER_LOG_SIZE + (PAGE_SIZE - sizeof(uint32_t) - sizeof(struct logger_header)) / 4)
	unsigned char log_buf[4][LOG_BUF_SIZE];
};

struct logger_buffer *logger_buffer;
struct logger_buffer *old_logger_buffer;


/*
 * Defines a log structure with name 'NAME' and a size of 'SIZE' bytes, which
 * must be a power of two, greater than LOGGER_ENTRY_MAX_LEN, and less than
 * LONG_MAX minus LOGGER_ENTRY_MAX_LEN.
 */
#define DEFINE_LOGGER_DEVICE(VAR, NAME) \
static struct logger_log VAR = { \
	.misc = { \
		.minor = MISC_DYNAMIC_MINOR, \
		.name = NAME, \
		.fops = &logger_fops, \
		.parent = NULL, \
	}, \
	.wq = __WAIT_QUEUE_HEAD_INITIALIZER(VAR .wq), \
	.readers = LIST_HEAD_INIT(VAR .readers), \
	.mutex = __MUTEX_INITIALIZER(VAR .mutex), \
};

DEFINE_LOGGER_DEVICE(log_main, LOGGER_LOG_MAIN)
DEFINE_LOGGER_DEVICE(log_events, LOGGER_LOG_EVENTS)
DEFINE_LOGGER_DEVICE(log_radio, LOGGER_LOG_RADIO)
DEFINE_LOGGER_DEVICE(log_system, LOGGER_LOG_SYSTEM)

static struct logger_log *get_log_from_minor(int minor)
{
	if (log_main.misc.minor == minor)
		return &log_main;
	if (log_events.misc.minor == minor)
		return &log_events;
	if (log_radio.misc.minor == minor)
		return &log_radio;
	if (log_system.misc.minor == minor)
		return &log_system;
	return NULL;
}

static int __init init_log(struct logger_log *log)
{
	int ret;

	ret = misc_register(&log->misc);
	if (unlikely(ret)) {
		printk(KERN_ERR "logger: failed to register misc "
		       "device for log '%s'!\n", log->misc.name);
		return ret;
	}

	printk(KERN_INFO "logger: created %luK log '%s'\n",
	       (unsigned long) log_size() >> 10, log->misc.name);

	return 0;
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
extern int boot_from_crash(void);
#else
#define boot_from_crash()	1
#endif

static void logger_save_old(void)
{
	old_logger_buffer = kmalloc(LOGGER_BUFFER_SIZE, GFP_KERNEL);
	if (old_logger_buffer == NULL) {
		pr_err("Logger: Failed to allocate buffer\n");
		return;
	}
	pr_info("Save the old logger buffer, old_logger_buffer = %p, logger_buffer = %p\n", old_logger_buffer, logger_buffer);
	memcpy(old_logger_buffer, logger_buffer, LOGGER_BUFFER_SIZE);
}

static int logger_driver_probe(struct platform_device *pdev)
{
	int ret, i;
	size_t start, size;

	size = LOGGER_BUFFER_SIZE;
	start = cma_alloc(&pdev->dev, "logger", size, 0);
	if (IS_ERR_VALUE(start)) {
		pr_err("%s: CMA Alloc Error!!!", __func__);
		return start;
	}
	pr_info("logger: got buffer at %zx, size %zx\n", start, size);

	logger_buffer = cma_get_virt(start, size, 0);

	pr_info("logger: got buffer signature: %x\n", logger_buffer->sig);

	if (logger_buffer->sig == LOGGER_SIG) {
		/* Save the last logs for late accessing if boot from crash */
		if (boot_from_crash())
			logger_save_old();
	} else {
		pr_info("Logger: no valid data in buffer (sig = 0x%08x)\n", logger_buffer->sig);
	}

	memset(logger_buffer, '\0', LOGGER_BUFFER_SIZE);

	logger_buffer->sig = LOGGER_SIG;

	log_main.buffer = logger_buffer->log_buf[0];
	log_events.buffer = logger_buffer->log_buf[1];
	log_radio.buffer = logger_buffer->log_buf[2];
	log_system.buffer = logger_buffer->log_buf[3];

	for (i = 0; i < 4; i++) {
		logger_buffer->log_head[i].w_off = 0;
		logger_buffer->log_head[i].head = 0;
		logger_buffer->log_head[i].size =  LOGGER_LOG_SIZE;
	}

	log_main.w_off = &logger_buffer->log_head[0].w_off;
	log_main.head = &logger_buffer->log_head[0].head;
	log_main.size = &logger_buffer->log_head[0].size;

	log_events.w_off = &logger_buffer->log_head[1].w_off;
	log_events.head = &logger_buffer->log_head[1].head;
	log_events.size = &logger_buffer->log_head[1].size;

	log_radio.w_off = &logger_buffer->log_head[2].w_off;
	log_radio.head = &logger_buffer->log_head[2].head;
	log_radio.size = &logger_buffer->log_head[2].size;

	log_system.w_off = &logger_buffer->log_head[3].w_off;
	log_system.head = &logger_buffer->log_head[3].head;
	log_system.size = &logger_buffer->log_head[3].size;

	ret = init_log(&log_main);
	if (unlikely(ret))
		goto out;

	ret = init_log(&log_events);
	if (unlikely(ret))
		goto out;

	ret = init_log(&log_radio);
	if (unlikely(ret))
		goto out;

	ret = init_log(&log_system);
	if (unlikely(ret))
		goto out;

out:
	return ret;
}

static struct platform_driver logger_driver = {
	.probe = logger_driver_probe,
	.driver		= {
		.name	= "logger",
	},
};

struct platform_device logger_dev = {
	.name = "logger",
	.id = -1,
};

static int __init logger_init(void)
{
	int err = 0;

	pr_info("%s\n", __func__);
	err = platform_device_register(&logger_dev);
	if (err) {
		pr_err("unable to register logger platform device\n");
		return err;
	}

	err = platform_driver_register(&logger_driver);
	if (err)
		pr_err("%s: platform_driver_register fail\n", __func__);

	return err;
}
device_initcall(logger_init);

/*
struct logger_entry {
	__u16		len;	// length of the payload
	__u16		__pad;	// no matter what, we get 2 bytes of padding
	__s32		pid;	// generating process's pid
	__s32		tid;	// generating process's tid
	__s32		sec;	// seconds since Epoch
	__s32		nsec;	// nanoseconds
	char		msg[0];	// the entry's payload
};
*/

/* ported from (Android)/system/core/logcat/logcat.cpp */

typedef enum android_LogPriority {
    ANDROID_LOG_UNKNOWN = 0,
    ANDROID_LOG_DEFAULT,    /* only for SetMinPriority() */
    ANDROID_LOG_VERBOSE,
    ANDROID_LOG_DEBUG,
    ANDROID_LOG_INFO,
    ANDROID_LOG_WARN,
    ANDROID_LOG_ERROR,
    ANDROID_LOG_FATAL,
    ANDROID_LOG_SILENT,     /* only for SetMinPriority(); must be last */
} android_LogPriority;

static char filterPriToChar(android_LogPriority pri)
{
    switch (pri) {
        case ANDROID_LOG_VERBOSE:       return 'V';
        case ANDROID_LOG_DEBUG:         return 'D';
        case ANDROID_LOG_INFO:          return 'I';
        case ANDROID_LOG_WARN:          return 'W';
        case ANDROID_LOG_ERROR:         return 'E';
        case ANDROID_LOG_FATAL:         return 'F';
        case ANDROID_LOG_SILENT:        return 'S';

        case ANDROID_LOG_DEFAULT:
        case ANDROID_LOG_UNKNOWN:
        default:                        return '?';
    }
}

#define DEFINE_LOG_SHOW(LOG_NAME, ORDER)			\
static int LOG_NAME ## _show(struct seq_file *m, void *v)	\
{								\
	size_t i, j, len;					\
	struct logger_entry *entry;				\
	unsigned char *buf = old_logger_buffer->log_buf[ORDER];	\
	struct rtc_time tm;					\
	int msgbegin, msgend;					\
	android_LogPriority prio;				\
	const char *tag;					\
	unsigned char *p;					\
	char prio_char;						\
								\
	for (i = old_logger_buffer->log_head[ORDER].head; i <= old_logger_buffer->log_head[ORDER].size;) { 	\
		/* Cope with current entry */			\
		entry = (struct logger_entry *)((size_t)buf + i);	\
								\
		if (entry->len == 0)				\
			break;					\
		if (entry->len < 3) {				\
			seq_printf(m, "+++ LOG: entry too small\n");	\
			continue;					\
		}						\
		msgbegin = -1;				\
		msgend = -1;				\
		for (j = 1; j < entry->len; j++) {		\
			if (entry->msg[j] == '\0') {		\
				if (msgbegin == -1) {		\
					msgbegin = j + 1;	\
				} else {			\
					msgend = j;		\
				}				\
			}					\
		}						\
		if (msgbegin == -1) {				\
			seq_printf(m, "+++ LOG: malformed log message\n");	\
			continue;				\
		}						\
		if (msgend == -1) {				\
			msgend = entry->len - 1;		\
			entry->msg[msgend] = '\0';		\
		}						\
								\
		prio = entry->msg[0];				\
		prio_char = filterPriToChar(prio);		\
		tag = entry->msg + 1;				\
		len = msgend - msgbegin;			\
								\
		rtc_time_to_tm(entry->sec, &tm);		\
		seq_printf(m, "%02d-%02d %02d:%02d:%02d.%03d ", \
			tm.tm_mon + 1, tm.tm_mday, \
			(tm.tm_hour + 8) % 24, tm.tm_min, tm.tm_sec, entry->nsec/1000000);	\
		seq_printf(m, "%c/%-8s(%5d): ", prio_char, tag, entry->pid);	\
		for (j = msgbegin; j <= msgend; j++) {		\
			p =  entry->msg + j;			\
			seq_printf(m, "%c", *p);		\
			if (*p == '\n') {			\
				rtc_time_to_tm(entry->sec, &tm);		\
				seq_printf(m, "%02d-%02d %02d:%02d:%02d.%03d ", \
					tm.tm_mon + 1, tm.tm_mday, \
					(tm.tm_hour + 8) % 24, tm.tm_min, tm.tm_sec, entry->nsec/1000000);	\
				seq_printf(m, "%c/%-8s(%5d): ", prio_char, tag, entry->pid);	\
			}					\
		}						\
		seq_printf(m, "\n");				\
								\
		/* Next entry */				\
		i = i + entry->len + sizeof(struct logger_entry);	\
	}							\
	return 0;						\
}

DEFINE_LOG_SHOW(last_log_main, 0)
DEFINE_LOG_SHOW(last_log_radio, 2)
DEFINE_LOG_SHOW(last_log_system, 3)

/*
 * Extract a 4-byte value from a byte stream.
 */
static inline uint32_t get4LE(const uint8_t* src)
{
	return src[0] | (src[1] << 8) | (src[2] << 16) | (src[3] << 24);
}

/*
 * Extract an 8-byte value from a byte stream.
 */
static inline uint64_t get8LE(const uint8_t* src)
{
    uint32_t low, high;

    low = src[0] | (src[1] << 8) | (src[2] << 16) | (src[3] << 24);
    high = src[4] | (src[5] << 8) | (src[6] << 16) | (src[7] << 24);
    return ((long long) high << 32) | (long long) low;
}

/*
 * Event log entry types.  These must match up with the declarations in
 * java/android/android/util/EventLog.java.
 */
typedef enum {
    EVENT_TYPE_INT      = 0,
    EVENT_TYPE_LONG     = 1,
    EVENT_TYPE_STRING   = 2,
    EVENT_TYPE_LIST     = 3,
} AndroidEventLogType;

/*
 * Recursively convert binary log data to printable form.
 *
 * This needs to be recursive because you can have lists of lists.
 *
 * If we run out of room, we stop processing immediately.  It's important
 * for us to check for space on every output element to avoid producing
 * garbled output.
 *
 * Returns 0 on success, 1 on buffer full, -1 on failure.
 */
static int android_log_printBinaryEvent(const unsigned char** pEventData,
    size_t* pEventDataLen, char** pOutBuf, size_t* pOutBufLen)
{
    const unsigned char* eventData = *pEventData;
    size_t eventDataLen = *pEventDataLen;
    char* outBuf = *pOutBuf;
    size_t outBufLen = *pOutBufLen;
    unsigned char type;
    size_t outCount;
    int result = 0;

    if (eventDataLen < 1)
        return -1;
    type = *eventData++;
    eventDataLen--;

    /* fprintf(stderr, "--- type=%d (rem len=%d)\n", type, eventDataLen); */

    switch (type) {
    case EVENT_TYPE_INT:
        /* 32-bit signed int */
        {
            int ival;

            if (eventDataLen < 4)
                return -1;
            ival = get4LE(eventData);
            eventData += 4;
            eventDataLen -= 4;

            outCount = snprintf(outBuf, outBufLen, "%d", ival);
            if (outCount < outBufLen) {
                outBuf += outCount;
                outBufLen -= outCount;
            } else {
                /* halt output */
                goto no_room;
            }
        }
        break;
    case EVENT_TYPE_LONG:
        /* 64-bit signed long */
        {
            long long lval;

            if (eventDataLen < 8)
                return -1;
            lval = get8LE(eventData);
            eventData += 8;
            eventDataLen -= 8;

            outCount = snprintf(outBuf, outBufLen, "%lld", lval);
            if (outCount < outBufLen) {
                outBuf += outCount;
                outBufLen -= outCount;
            } else {
                /* halt output */
                goto no_room;
            }
        }
        break;
    case EVENT_TYPE_STRING:
        /* UTF-8 chars, not NULL-terminated */
        {
            unsigned int strLen;

            if (eventDataLen < 4)
                return -1;
            strLen = get4LE(eventData);
            eventData += 4;
            eventDataLen -= 4;

            if (eventDataLen < strLen)
                return -1;

            if (strLen < outBufLen) {
                memcpy(outBuf, eventData, strLen);
                outBuf += strLen;
                outBufLen -= strLen;
            } else if (outBufLen > 0) {
                /* copy what we can */
                memcpy(outBuf, eventData, outBufLen);
                outBuf += outBufLen;
                outBufLen -= outBufLen;
                goto no_room;
            }
            eventData += strLen;
            eventDataLen -= strLen;
            break;
        }
    case EVENT_TYPE_LIST:
        /* N items, all different types */
        {
            unsigned char count;
            int i;

            if (eventDataLen < 1)
                return -1;

            count = *eventData++;
            eventDataLen--;

            if (outBufLen > 0) {
                *outBuf++ = '[';
                outBufLen--;
            } else {
                goto no_room;
            }

            for (i = 0; i < count; i++) {
                result = android_log_printBinaryEvent(&eventData, &eventDataLen,
                        &outBuf, &outBufLen);
                if (result != 0)
                    goto bail;

                if (i < count-1) {
                    if (outBufLen > 0) {
                        *outBuf++ = ',';
                        outBufLen--;
                    } else {
                        goto no_room;
                    }
                }
            }

            if (outBufLen > 0) {
                *outBuf++ = ']';
                outBufLen--;
            } else {
                goto no_room;
            }
        }
        break;
    default:
        pr_err("Unknown binary event type %d\n", type);
        return -1;
    }

bail:
    *pEventData = eventData;
    *pEventDataLen = eventDataLen;
    *pOutBuf = outBuf;
    *pOutBufLen = outBufLen;
    return result;

no_room:
    result = 1;
    goto bail;
}

#undef ORDER
#define ORDER 1

static int last_log_events_show(struct seq_file *m, void *v)
{
	size_t i, outRemaining;
	int result, inCount;
	unsigned int tagIndex, tagLen, messageBufLen;
	struct rtc_time tm;
	struct logger_entry *entry;
	unsigned char *buf = old_logger_buffer->log_buf[ORDER];
	const char *tag;
	const unsigned char *eventData;
	char msgbuf[950], *messageBuf, *outBuf;

	for (i = old_logger_buffer->log_head[ORDER].head; i <= old_logger_buffer->log_head[ORDER].size;) {
		/* Cope with current entry */
		entry = (struct logger_entry *)((size_t)buf + i);

		inCount = entry->len;

		if (inCount == 0)
			break;
		if (inCount < 4) {
			seq_printf(m, "+++ LOG: entry too small\n");
			continue;
		}
		eventData = (const unsigned char *)entry->msg;
		tagIndex = get4LE(eventData);
		eventData += 4;
		inCount -= 4;
		messageBuf = msgbuf;
		messageBufLen = sizeof(msgbuf);
		tagLen = snprintf(messageBuf, sizeof(messageBuf), "[%d]", tagIndex);
		tag = messageBuf;
		messageBuf = messageBuf + tagLen + 1;
		messageBufLen = messageBufLen - (tagLen + 1);

		/*
		 * Format the event log data into the buffer.
		 */
		outBuf = messageBuf;
		outRemaining = messageBufLen-1;      /* leave one for nul byte */
		result = android_log_printBinaryEvent(&eventData, &inCount, &outBuf,
				&outRemaining);
		if (result < 0) {
			seq_printf(m, "Binary log entry conversion failed\n");
			continue;
		} else if (result == 1) {
			if (outBuf > messageBuf) {
				/* leave an indicator */
				*(outBuf-1) = '!';
			} else {
				/* no room to output anything at all */
				*outBuf++ = '!';
				outRemaining--;
			}
			/* pretend we ate all the data */
			inCount = 0;
		}

		/* eat the silly terminating '\n' */
		if (inCount == 1 && *eventData == '\n') {
			eventData++;
			inCount--;
		}

		if (inCount != 0) {
			/* seq_printf(m, "Warning: leftover binary log data (%d bytes)\n", inCount); */
		}

		/*
		* Terminate the buffer.  The NUL byte does not count as part of
		* entry->messageLen.
		*/
		*outBuf = '\0';

		rtc_time_to_tm(entry->sec, &tm);
		seq_printf(m, "%02d-%02d %02d:%02d:%02d.%03d ",
			tm.tm_mon + 1, tm.tm_mday,
			(tm.tm_hour + 8) % 24, tm.tm_min, tm.tm_sec, entry->nsec/1000000);
		seq_printf(m, "I/%-8s(%5d): ", tag, entry->pid);
		seq_printf(m, "%s\n", messageBuf);

		/* Next entry */
		i = i + entry->len + sizeof(struct logger_entry);
	}
	return 0;
}
#undef ORDER

static void *log_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *log_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void log_stop(struct seq_file *m, void *v)
{
}

#define DEFINE_LOG_PROC_FILE(LOG_NAME)			\
const struct seq_operations LOG_NAME ## _op = {		\
	.start	= log_start,				\
	.next	= log_next,				\
	.stop	= log_stop,				\
	.show	= LOG_NAME ## _show			\
};							\
							\
static int proc_ ## LOG_NAME ## _open(struct inode *inode, struct file *file)	\
{										\
	return seq_open(file, &LOG_NAME ## _op);				\
}										\
										\
static const struct file_operations proc_ ## LOG_NAME  ## _operations = {	\
	.open		= proc_ ## LOG_NAME ## _open,				\
	.read		= seq_read,						\
	.llseek		= seq_lseek,						\
	.release	= seq_release,						\
};

DEFINE_LOG_PROC_FILE(last_log_main)
DEFINE_LOG_PROC_FILE(last_log_events)
DEFINE_LOG_PROC_FILE(last_log_radio)
DEFINE_LOG_PROC_FILE(last_log_system)

static int __init logger_late_init(void)
{
	struct proc_dir_entry *entry;

	if (old_logger_buffer == NULL)
		return 0;

#define CREATE_LOG_PROC(LOG_NAME) 									\
	entry = proc_create(__stringify(LOG_NAME), S_IFREG | S_IRUGO, NULL, &proc_ ## LOG_NAME ## _operations);	\
	if (!entry) {											\
		pr_err("ram_console: failed to create proc entry of log_main\n");			\
		kfree(old_logger_buffer);								\
		old_logger_buffer = NULL;								\
		return 0;										\
	}

	CREATE_LOG_PROC(last_log_main)
	CREATE_LOG_PROC(last_log_events)
	CREATE_LOG_PROC(last_log_radio)
	CREATE_LOG_PROC(last_log_system)

	return 0;
}

late_initcall(logger_late_init);
