#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdio.h>

#define B3DFG_IOC_MAGIC         0xb3 /* dfg :-) */
#define B3DFG_IOCGFRMSZ         _IOR(B3DFG_IOC_MAGIC, 1, int)
#define B3DFG_IOCTNUMBUFS       _IO(B3DFG_IOC_MAGIC, 2)
#define B3DFG_IOCTTRANS         _IO(B3DFG_IOC_MAGIC, 3)
#define B3DFG_IOCTQUEUEBUF      _IO(B3DFG_IOC_MAGIC, 4)
#define B3DFG_IOCQPOLLBUF       _IO(B3DFG_IOC_MAGIC, 5)
#define B3DFG_IOCQWAITBUF       _IO(B3DFG_IOC_MAGIC, 6)

#define IMG_SIZE (1024*768)

static int fd;

static void print_frame_size(void)
{
	int frmsz = 0;
	int r = ioctl(fd, B3DFG_IOCGFRMSZ, &frmsz);
	if (r < 0)
		perror("print_frame_size ioctl");
	printf("print_frame_size ioctl ret %d frmsz %d\n", r, frmsz);
}

static void set_num_buffers(int num_buffers)
{
	int r = ioctl(fd, B3DFG_IOCTNUMBUFS, num_buffers);
	if (r < 0)
		perror("set_num_buffers ioctl");
	else
		printf("set %d buffers result %d\n", num_buffers, r);
}

static void set_transmission(int enabled)
{
	int r = ioctl(fd, B3DFG_IOCTTRANS, enabled);
	if (r < 0)
		perror("set_transmission ioctl");
	else
		printf("set transmission %d result %d\n", enabled, r);
}

static void poll_buffer(int buffer)
{
	int r = ioctl(fd, B3DFG_IOCQPOLLBUF, buffer);
	if (r < 0)
		perror("poll_buffer failed");
	else
		printf("poll_buffer %d result %d\n", buffer, r);
}

static void wait_buffer(int buffer)
{
	int r = ioctl(fd, B3DFG_IOCQWAITBUF, buffer);
	if (r < 0)
		perror("wait_buffer failed");
	else
		printf("wait_buffer %d result %d\n", buffer, r);
}

static void queue_buffer(int buffer)
{
	int r = ioctl(fd, B3DFG_IOCTQUEUEBUF, buffer);
	if (r < 0)
		perror("queue_buffer");
	else
		printf("queue_buffer %d result %d\n", buffer, r);
}

static void write_img(const char *pfx, unsigned char *data)
{
	unsigned char filename[50];
	sprintf(filename, "%s.raw", pfx);

	FILE *fd = fopen(filename, "w");
	if (!fd) {
		perror("fopen");
		return;
	}

	fwrite(data, 1, IMG_SIZE, fd);
	fclose(fd);

	sprintf(filename, "%s.pgm", pfx);
	fd = fopen(filename, "w");
	fprintf(fd, "P5 1024 768 255 ");
	fwrite(data, 1, IMG_SIZE, fd);
	fclose(fd);
}

int main(void)
{
	unsigned char *data;

	fd = open("/dev/b3dfg0", O_RDONLY);
	if (fd == -1) {
		perror("open");
		exit(1);
	}

	print_frame_size();
	set_num_buffers(2);

	data = mmap(NULL, 2 * IMG_SIZE * 3, PROT_READ, MAP_SHARED, fd, 0);
	if (data == MAP_FAILED) {
		perror("mmap");
		exit(1);
	}

	queue_buffer(0);
	queue_buffer(1);
	set_transmission(1);
	poll_buffer(0);
	wait_buffer(0);
	write_img("01_red", data);
	write_img("02_green", data + IMG_SIZE);
	write_img("03_blue", data + IMG_SIZE + IMG_SIZE);

	wait_buffer(1);
	write_img("04_red", data + (IMG_SIZE * 3));
	write_img("05_green", data + (IMG_SIZE * 4));
	write_img("06_blue", data + (IMG_SIZE * 5));

	munmap(data, 2 * IMG_SIZE * 3);
	close(fd);
}
