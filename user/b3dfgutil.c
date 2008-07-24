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

#include <readline/readline.h>
#include <readline/history.h>

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

#define IMG_SIZE (1024*768)

#define DEV_NAME "/dev/b3dfg0"

static int fd = -1;
static unsigned char *data = NULL;

int require_open(const char *cmd)
{
	if (fd == -1)
		printf("%s error: device not open\n", cmd);

	return fd != -1;
}

void cmd_open(void)
{
	if (fd != -1)
		printf("open error: device already open\n");

	fd = open(DEV_NAME, O_RDONLY);
	if (fd == -1)
		fprintf(stderr, "cannot open device '%s': %s\n", DEV_NAME, strerror(errno));
}

void cmd_close(void)
{
	if (!require_open("close"))
		return;

	close(fd);
	fd = -1;
}

void cmd_wand(void)
{
	int res, status;

	if (!require_open("wand"))
		return;

	res = ioctl(fd, B3DFG_IOCGWANDSTAT, &status);
	if (res < 0)
		fprintf(stderr, "error checking cable status: %s\n", strerror(errno));
	else
		printf("cable status: %splugged\n", status ? "" : "un");
}

void cmd_bufs(const char *arg)
{
	int res, bufs;

	if (sscanf(arg, "%d", &bufs) != 1 || bufs < 1) {
		printf("bufs error: positive integer argument required\n");
		return;
	}

	if (!require_open("bufs"))
		return;

        res = ioctl(fd, B3DFG_IOCTNUMBUFS, bufs);
        if (res < 0)
                fprintf(stderr, "error setting buffer count: %s\n", strerror(errno));
        else
                printf("set buffer count: %d\n", bufs);
}

void cmd_mmap(void)
{
	if (!require_open("mmap"))
		return;

	if (data != NULL) {
		printf("mmap error: already mapped\n");
		return;
	}

	data = mmap(NULL, 2 * IMG_SIZE * 3, PROT_READ, MAP_SHARED, fd, 0);
	if (data == MAP_FAILED) {
		fprintf(stderr, "mmap error: %s\n", strerror(errno));
		data = NULL;
	}
}

void cmd_munmap(void)
{
	if (data == NULL) {
		printf("munmap error: not mapped\n");
		return;
	}

	if (munmap(data, 2 * IMG_SIZE * 3) != 0)
		fprintf(stderr, "munmap error: %s\n", strerror(errno));

	data = NULL;
}

void cmd_queue(const char *arg)
{
	int res, buf;

	if (sscanf(arg, "%d", &buf) != 1 || buf < 0) {
		printf("queue error: buffer index required\n");
		return;
	}

	if (!require_open("queue"))
		return;

	res = ioctl(fd, B3DFG_IOCTQUEUEBUF, buf);
        if (res < 0)
                fprintf(stderr, "error queuing buffer %d: %s\n", buf, strerror(errno));
        else
                printf("buffer %d queued: %d\n", buf, res);
}

void cmd_poll(const char *arg)
{
	int res;
	struct b3dfg_poll parg;

	if (sscanf(arg, "%d", &parg.buffer_idx) != 1 || parg.buffer_idx < 0) {
		printf("poll error: buffer index required\n");
		return;
	}

	if (!require_open("poll"))
		return;

        res = ioctl(fd, B3DFG_IOCTPOLLBUF, &parg);
        if (res < 0)
                fprintf(stderr, "error polling buffer %d: %s\n",
		        parg.buffer_idx, strerror(errno));
        else
                printf("poll buffer %d, result: %d, triplets dropped: %u\n",
		       parg.buffer_idx, parg.triplets_dropped, res);
}

void cmd_wait(const char *arg)
{
	int res;
	struct b3dfg_wait warg;

	if (sscanf(arg, "%d", &warg.buffer_idx) != 1 || warg.buffer_idx < 0) {
		printf("wait error: buffer index required\n");
		return;
	}

	/* TODO: Allow user to specify */
	warg.timeout = 60 * 1000;

	if (!require_open("wait"))
		return;

        res = ioctl(fd, B3DFG_IOCTWAITBUF, &warg);
        if (res < 0)
                fprintf(stderr, "error waiting on buffer %d: %s\n",
		        warg.buffer_idx, strerror(errno));
        else
                printf("wait buffer %d, result: %d, triplets dropped: %u\n",
		       warg.buffer_idx, warg.triplets_dropped, res);
}

void cmd_start(void)
{
	int res;

	if (!require_open("start"))
		return;

	res = ioctl(fd, B3DFG_IOCTTRANS, 1);
	if (res < 0)
		fprintf(stderr, "error starting transmission: %s\n", strerror(errno));
	else
		printf("transmission enabled\n");
}

void cmd_stop(void)
{
	int res;

	if (!require_open("stop"))
		return;

	res = ioctl(fd, B3DFG_IOCTTRANS, 0);
	if (res < 0)
		fprintf(stderr, "error stopping transmission: %s\n", strerror(errno));
	else
		printf("transmission disabled\n");
}

void cmd_write(const char *arg)
{
	int buf, count, offset;
	FILE *out;

	if (arg[0] == 'r' && arg[1] == ' ') {
		offset = 0;
	} else if (*arg == 'g' && arg[1] == ' ') {
		offset = IMG_SIZE;
	} else if (*arg == 'b' && arg[1] == ' ') {
		offset = IMG_SIZE * 2;
	} else {
		printf("write error: invalid color channel, requires one of 'r', 'g' or 'b'\n");
		return;
	}

	arg += 2;
	if (sscanf(arg, "%d %n", &buf, &count) < 1 || buf < 0) {
		printf("write error: buffer index required\n");
		return;
	}

	offset += IMG_SIZE * 3 * buf;

	arg += count;
	if (*arg == 0) {
		printf("write error: filename required\n");
		return;
	}

	if (data == NULL) {
		fprintf(stderr, "cannot write data: buffer not mapped\n");
		return;
	}

	out = fopen(arg, "w");
	if (out) {
		fwrite(data, 1, IMG_SIZE, out);
		fclose(out);
	} else {
		fprintf(stderr, "cannot open file '%s' for writing: %s\n", arg, strerror(errno));
	}
}

int main(int argc, char *argv[])
{
	char *cmd;

	while ((cmd = readline ("> ")) != NULL) {
		if (strcmp(cmd, "help") == 0) {
			printf("help\t\tthis text\n");
			printf("open\t\topen the device\n");
			printf("close\t\tclose the device\n");
			printf("wand\t\tcheck the current wand status\n");
			printf("bufs <buffers>\tset the number of buffers\n");
			printf("mmap\t\tmaps DMA buffers into process memory space\n");
			printf("munmap\t\tunmaps DMA buffers from process memory space\n");
			printf("queue <buffer>\tqueues a DMA buffer for receiving frames\n");
			printf("start\t\tstart transmission\n");
			printf("stop\t\tstop transmission\n");
			printf("poll <buffer>\t\tpolls whether the given buffer has been filled\n");
			printf("wait <buffer>\t\twaits for the given buffer to be filled\n");
			printf("write r|g|b <buffer> <file>\twrite either red, green or blue channel to a file\n");
			printf("quit\t\tquit the b3dfg utility\n");
		} else if (strcmp(cmd, "open") == 0) {
			cmd_open();
		} else if (strcmp(cmd, "close") == 0) {
			cmd_close();
		} else if (strncmp(cmd, "bufs ", 5) == 0) {
			cmd_bufs(cmd + 5);
		} else if (strcmp(cmd, "mmap") == 0) {
			cmd_mmap();
		} else if (strcmp(cmd, "munmap") == 0) {
			cmd_munmap();
		} else if (strncmp(cmd, "queue ", 6) == 0) {
			cmd_queue(cmd + 6);
		} else if (strncmp(cmd, "poll", 4) == 0) {
			cmd_poll(cmd + 4);
		} else if (strncmp(cmd, "wait", 4) == 0) {
			cmd_wait(cmd + 4);
		} else if (strncmp(cmd, "write ", 6) == 0) {
			cmd_write(cmd + 6);
		} else if (strcmp(cmd, "start") == 0) {
			cmd_start();
		} else if (strcmp(cmd, "stop") == 0) {
			cmd_stop();
		} else if (strcmp(cmd, "wand") == 0) {
			cmd_wand();
		} else if (strcmp(cmd, "quit") == 0) {
			free(cmd);
			break;
		} else {
			printf("unknown command: %s\n", cmd);
			printf("type help for a list of commands\n");
		}
		add_history(cmd);
		free(cmd);
	}

	if (fd != -1)
		close(fd);
}
