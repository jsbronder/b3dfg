#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

struct b3dfg_poll {
	int buffer_idx;
	unsigned int triplets_dropped;
};

struct b3dfg_wait {
	int buffer_idx;
	unsigned int timeout;
	unsigned int triplets_dropped;
};

#define B3DFG_IOC_MAGIC         0xb3 /* dfg :-) */
#define B3DFG_IOCGFRMSZ         _IOR(B3DFG_IOC_MAGIC, 1, int)
#define B3DFG_IOCTNUMBUFS       _IO(B3DFG_IOC_MAGIC, 2)
#define B3DFG_IOCTTRANS         _IO(B3DFG_IOC_MAGIC, 3)
#define B3DFG_IOCTQUEUEBUF      _IO(B3DFG_IOC_MAGIC, 4)
#define B3DFG_IOCTPOLLBUF       _IOWR(B3DFG_IOC_MAGIC, 5, struct b3dfg_poll)
#define B3DFG_IOCTWAITBUF       _IOWR(B3DFG_IOC_MAGIC, 6, struct b3dfg_wait)
#define B3DFG_IOCGWANDSTAT      _IOR(B3DFG_IOC_MAGIC, 7, int)

/* TODO: Read from driver. */
#define IMG_SIZE (1024*768)

#define DEV_NAME "/dev/b3dfg0"

static int fd = -1;
static int nr_bufs;
static unsigned long triplets_dropped;

/* 2s timeout for queue waits. */
#define QUEUE_WAIT_TIMEOUT (2 * 1000)

/*
 * tests:
 *  - waiting on buffers (w/ small timeout)
 */

void do_tx(void)
{
	sleep(1);
	ioctl(fd, B3DFG_IOCTTRANS, 1);
	sleep(1);
	ioctl(fd, B3DFG_IOCTTRANS, 0);
}

void do_mmap(void)
{
	int acc = 0;
	unsigned char *data, *ch;

	data = mmap(NULL, nr_bufs * IMG_SIZE * 3, PROT_READ, MAP_SHARED, fd, 0);
	if (data == MAP_FAILED)
		return;

	/* TODO: Read random locations instead of all. */
	for (ch = data; ch != data + nr_bufs * IMG_SIZE * 3; ++ch)
		acc += *ch;

	munmap(data, nr_bufs * IMG_SIZE * 3);
}

void do_queue(void)
{
	int idx;

	for (idx = 0; idx != nr_bufs; ++idx) {
		unsigned int timeout = 0;
		struct b3dfg_wait warg = {
			.buffer_idx = idx,
			.timeout = QUEUE_WAIT_TIMEOUT,
		};

        	if (ioctl(fd, B3DFG_IOCTQUEUEBUF, idx) == 0)
	        	ioctl(fd, B3DFG_IOCTWAITBUF, &warg);
	}
}

void do_poll(void)
{
	int idx;

	for (idx = 0; idx != nr_bufs; ++idx) {
		struct b3dfg_poll parg = {
			.buffer_idx = idx,
		};

	        if (ioctl(fd, B3DFG_IOCTPOLLBUF, &parg) == 0)
			triplets_dropped += parg.triplets_dropped;
	}
}

struct test {
	const char *name;
	const char *description;
	void (*func)(void);
	pid_t pid;
	int disabled;
};

struct test tx_test = {
	.name = "tx",
	.description = "Enable/disable transmission",
	.func = &do_tx,
};

struct test mmap_test = {
	.name = "mmap",
	.description = "Map, read and unmap device memory",
	.func = &do_mmap,
};

struct test queue_test = {
	.name = "queue",
	.description = "Queue and wait on buffers sequentially",
	.func = &do_queue,
};

struct test poll_test = {
	.name = "poll",
	.description = "Poll buffers",
	.func = &do_poll,
};

struct test *tests[] = {
	&tx_test, &mmap_test, &queue_test, &poll_test, NULL
};

int read_nr_bufs(void)
{
	/* TODO: Read from driver. */
	return 2;
}

void usage(FILE *out)
{
	struct test **tst;

	fprintf(out, "usage: b3dfgstress [[no]<test>]...\n\n");
	fprintf(out, "Tests:\n");
	for (tst = tests; *tst; ++tst) {
		fprintf(out, "\t%s\t%s\n", (*tst)->name, (*tst)->description);
	}
}

void run_test_loop(struct test *test)
{
	while (1) {
		test->func();
	}
}

int main(int argc, char *argv[])
{
	char **arg;
	struct test **tst;
	int tst_count = 0;

	for (arg = argv + 1; *arg; ++arg) {
		char *param = *arg;
		int handled = 0;
		int disable = 0;

		if (strcmp(param, "-h") == 0 || strcmp(param, "--help") == 0) {
			usage(stdout);
			exit(0);
		}

		if (strncmp(param, "no", 2) == 0) {
			disable = 1;
			param += 2;
		}

		for (tst = tests; *tst; ++tst) {
			if (strcmp(param, (*tst)->name) == 0) {
				handled = 1;
				(*tst)->disabled = disable;
				break;
			}
		}

		if (!handled) {
			fprintf(stderr, "unknown test: %s\n\n", *arg);
			usage(stderr);
			exit(1);
		}
	}

	fd = open(DEV_NAME, O_RDONLY);
	if (fd == -1) {
		fprintf(stderr, "cannot open device '%s': %s\n", DEV_NAME, strerror(errno));
		exit(2);
	}

	nr_bufs = read_nr_bufs();
	printf("buffers: %d\n", nr_bufs);

	for (tst = tests; *tst; ++tst) {
		if ((*tst)->disabled) {
			printf("disabled test: %s\n", (*tst)->name);
			continue;
		}

		(*tst)->pid = fork();
		if ((*tst)->pid < 0) {
			fprintf(stderr, "couldn't fork test %s: %s\n", (*tst)->name, strerror(errno));
			exit(3);
		} else if ((*tst)->pid) {
			++tst_count;
			printf("started test: %s\n", (*tst)->name);
		} else {
			sleep(1);
			run_test_loop(*tst);
		}
	}

	/* Wait for all tests. */
	while (tst_count > 0) {
		int pid = wait(NULL);
		if (pid < 0) {
			fprintf(stderr, "error waiting for tests to finish: %s\n", strerror(errno));
			exit(4);
		}
		--tst_count;
	}

	close(fd);
}
