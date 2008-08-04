/*
 * Brontes frame grabber example
 *
 * Copyright (C) 2008 3M Company
 * Author: Daniel Drake <ddrake@brontes3d.com>
 *
 * Private software. Source code may not be distributed.
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include <fcntl.h>
#include <termios.h>

#define COMM_PATH "/dev/ttyUSB0"
#define OPEN_FLAGS  O_RDWR | O_NOCTTY | O_NONBLOCK

#define SET_COMM_PARM                                   \
    struct termios newtio;                              \
    newtio.c_iflag     = IGNPAR;                        \
    newtio.c_oflag     = 0;                             \
    newtio.c_cflag     = B115200 | CS8 | CLOCAL | CREAD;\
    newtio.c_lflag     = 0;                             \
    newtio.c_cc[VMIN]  = 1;                             \
    newtio.c_cc[VTIME] = 0;


#include <libb3dfg/b3dfg.h>

#define NUM_BUFFERS 2
#define TIMEOUT 5000 // ms for wait/poll

static b3dfg_dev *dev;
static unsigned char *mapping, *mpng;
static unsigned int frame_size;
static int devfd;

static void write_frame(int buffer, int frame)
{
	char filename[50];
	FILE *fd;
	int num = (buffer * 3) + frame;

	sprintf(filename, "cap%02d.pgm", num);
	fd = fopen(filename, "w");
	fprintf(fd, "P5 1024 768 255 ");
	fwrite(mapping + (buffer * frame_size * 3) + (frame_size * frame), 1,
		frame_size, fd);
	fclose(fd);
	printf("=== writing buffer %d, frame %d to %s (0x%p, s=%d)\n", buffer, frame, filename, mapping+(buffer*frame_size*3)+(frame_size*frame), frame_size);
}

static void write_to_file(int buffer)
{
	write_frame(buffer, 0);
	write_frame(buffer, 1);
	write_frame(buffer, 2);
}

int main(void)
{
	int i, fd;
	int r = 1;

	printf("=== calling open(" COMM_PATH ")\n");
        if ((fd = open(COMM_PATH, OPEN_FLAGS)) == -1) { r=errno;  printf("No device path " COMM_PATH "\n"); goto out; }
        SET_COMM_PARM
        if (tcflush(fd, TCIFLUSH)) {
            printf("Failed in tcflush\n");
            return -2;
        }        
        if (tcsetattr(fd, TCSANOW, &newtio)) {
            printf("Invalid serial port parameters\n");
            return -3;
        }
	printf("            " COMM_PATH " opened on file %d\n", fd);

	printf("=== writing reset command to " COMM_PATH "\n");
        write(fd, "!0DF2*", 6);
        usleep(1000000);

//      printf("=== writing frame period command (500ms) to " COMM_PATH "\n");
//      write(fd, "!04C35068*", 10);
//      printf("=== writing frame period command (200ms) to " COMM_PATH "\n");
//      write(fd, "!044E2095*", 10);

	printf("=== calling b3dfg_open(0)\n");
	if (!(dev = b3dfg_open(0))) { fprintf(stderr, "    ******* b3dfg_open failed\n"); goto out; }
	printf("            b3dfg_open returned %p\n", dev);

	printf("=== calling b3dfg_get_fd(%p)\n", dev);
	devfd = b3dfg_get_fd(dev);
	printf("            b3dfg_get_fd returned %d\n", devfd);

	printf("=== calling b3dfg_get_frame_size(%p)\n", dev);
	frame_size = b3dfg_get_frame_size(dev);
	printf("            b3dfg_get_frame_size returned %d (%dK)\n", frame_size, frame_size/1024);

	printf("=== calling b3dfg_set_num_buffers(%p, %d)\n", dev, NUM_BUFFERS);
	if((r = b3dfg_set_num_buffers(dev, NUM_BUFFERS))) { fprintf(stderr, "    ******* b3dfg_set_num_buffers failed\n"); goto out; }
	printf("            b3dfg_set_num_buffers returned %d\n", r);

	printf("=== calling b3dfg_map_buffers(%p, 0)\n", dev);
	if (!(mapping = b3dfg_map_buffers(dev, 0))) { fprintf(stderr, "    ******* b3dfg_map_buffers failed\n"); goto out; }
	printf("            b3dfg_map_buffers returned buffer address %p\n", mapping);

	printf("=== calling b3dfg_get_mapping(%p)\n", dev);
	if (!(mpng = b3dfg_get_mapping(dev))) { fprintf(stderr, "    ******* b3dfg_get_mapping failed\n"); goto out; }
	printf("            b3dfg_get_mapping returned buffer address %p\n", mpng);

#if 1
	printf("=== acquisition prep ---------------------------------------\n");
	printf("=== calling b3dfg_queue_buffer(%p, %d)\n", dev, 0);
	if ((r = b3dfg_queue_buffer(dev, 0))) { fprintf(stderr, "    ******* b3dfg_queue_buffer failed\n"); goto out; }
	printf("            b3dfg_queue_buffer returned %d\n", r);

	printf("=== calling b3dfg_set_transmission(%p, 1)\n", dev);
	if ((r = b3dfg_set_transmission(dev, 1))) { fprintf(stderr, "    ******* b3dfg_set_transmission failed\n"); goto out; }
	printf("            b3dfg_set_transmission returned %d\n", r);

	printf("=== writing start command to " COMM_PATH "\n");
        write(fd, "!01FE*", 6);

	printf("=== acquisition loop ---------------------------------------\n");
	int maxtime = 0;
	int mintime = 99999;
	int maxdropped = 0;
	for (i = 1; i <= 20; i++) {
//		printf("=== calling b3dfg_queue_buffer(%p, %d)\n", dev, i&1);
		if ((r = b3dfg_queue_buffer(dev, i&1))) { fprintf(stderr, "    ******* b3dfg_queue_buffer failed\n"); goto out; }
//		printf("            b3dfg_queue_buffer returned %d\n", r);

#define WTIME 300000
if (i==10) { printf("\n*** usleep(%d) (%dms)\n", WTIME, WTIME/1000); usleep(WTIME); }

		printf("=== calling b3dfg_wait_buffer(%p, %d, %dms, NULL)\n", dev, !(i&1), TIMEOUT);
		int dropped;
		if ((r = b3dfg_wait_buffer(dev, !(i&1), TIMEOUT, &dropped)) && r < 0) {
			if (r == -ETIMEDOUT)
				fprintf(stderr, "    ******* b3dfg_wait_buffer timed out\n");
			else
				fprintf(stderr, "    ******* b3dfg_wait_buffer failed (%d)\n", r);
			goto out;
		}
//		printf("            b3dfg_wait_buffer returned after %dms\n", TIMEOUT-r);
		int atime = TIMEOUT-r;
		if (i > 1 && atime > maxtime) maxtime = atime;
		if (i > 1 && atime < mintime) mintime = atime;
		if (dropped > maxdropped) maxdropped = dropped;
		printf("=== queued buffer %d, populated buffer %d after %dms(%dms/%dms), frame %d, dropped %d(%d)\n", i&1, !(i&1), atime, mintime, maxtime, i, dropped, maxdropped);
		if (atime < 45 || atime > 55)
//		if (atime < 195 || atime > 205)
			printf("*** wait time is %d\n", atime);

//		if (i == 5 || i == 6)
//			write_to_file(!(i&1));
	}

	printf("=== writing stop command to " COMM_PATH "\n");
        write(fd, "!02FD*", 6);
        usleep(1000000);

	printf("=== calling b3dfg_set_transmission(%p, 0)\n", dev);
	if ((r = b3dfg_set_transmission(dev, 0))) { fprintf(stderr, "    ******* set_transmission failed\n"); goto out; }
	printf("            b3dfg_set_transmission returned %d\n", r);
#else
	for (i = 0; i < NUM_BUFFERS; i++) {
		printf("=== calling b3dfg_queue_buffer(%p, %d)\n", dev, i);
		if ((r = b3dfg_queue_buffer(dev, i))) { fprintf(stderr, "    ******* b3dfg_queue_buffer failed\n"); goto out; }
		printf("            b3dfg_queue_buffer returned %d\n", r);
	}

	printf("=== calling b3dfg_set_transmission(%p, 1)\n", dev);
	if ((r = b3dfg_set_transmission(dev, 1))) { fprintf(stderr, "    ******* b3dfg_set_transmission failed\n"); goto out; }
	printf("            b3dfg_set_transmission returned %d\n", r);

	for (i = 0; i < NUM_BUFFERS || 1; i++) {
		printf("=== calling b3dfg_wait_buffer(%p, %d, %dms, NULL)\n", dev, i, TIMEOUT);
		if ((r = b3dfg_wait_buffer(dev, i, TIMEOUT, NULL)) && r < 0) {
			if (r == -ETIMEDOUT)
				fprintf(stderr, "    ******* b3dfg_wait_buffer timed out\n");
			else
				fprintf(stderr, "    ******* b3dfg_wait_buffer failed (%d)\n", r);
			goto out;
		}
		printf("            b3dfg_wait_buffer returned after %dms\n", TIMEOUT-r);
		write_to_file(i);
	}
#endif
	r = 0;
out:
	printf("\n");
	if (dev) {
		printf("=== calling b3dfg_unmap_buffers\n");
		b3dfg_unmap_buffers(dev);
		printf("            b3dfg_unmap_buffers called\n");
		}
	printf("=== calling b3dfg_close\n");
	b3dfg_close(dev);
	printf("            b3dfg_close called\n");

	printf("=== calling close(" COMM_PATH ")\n");
        if (close(fd) == -1)
            fprintf(stderr, "close failed\n");

	return r;
}
