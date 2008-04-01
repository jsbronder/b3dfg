#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <stdio.h>

#define B3DFG_IOC_MAGIC     0xb3 /* dfg :-) */
#define B3DFG_IOCGFRMSZ     _IOR(B3DFG_IOC_MAGIC, 1, int)
#define B3DFG_IOCSNUMBUFS   _IOW(B3DFG_IOC_MAGIC, 2, int)
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
	int r = ioctl(fd, B3DFG_IOCSNUMBUFS, num_buffers);
	if (r < 0)
		perror("set_num_buffers ioctl");
	else
		printf("set %d buffers result %d\n", num_buffers, r);
}

int main(void)
{
	fd = open("/dev/b3dfg0", O_RDONLY);
	if (fd == -1) {
		perror("open");
		exit(1);
	}

	print_frame_size();
	set_num_buffers(3);

	close(fd);
}
