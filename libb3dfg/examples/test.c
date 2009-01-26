/*
 * Brontes frame grabber example
 *
 * Copyright (C) 2008 3M Company
 * Author: Daniel Drake <ddrake@brontes3d.com>
 *
 * Private software. Source code may not be distributed.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

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
#define TIMEOUT 1000 // ms for wait/poll

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
    printf(">>> writing buffer %d, frame %d to %s (0x%p, s=%d)\n", buffer, frame, filename, mapping+(buffer*frame_size*3)+(frame_size*frame), frame_size);
}


static void write_to_file(int buffer)
{
    write_frame(buffer, 0);
    write_frame(buffer, 1);
    write_frame(buffer, 2);
}


#define SOM                 '!'     // start of message
#define EOM                 '*'     // end of message
#define DO_N_PULSES         0x03    // start N trigger pulses + 1 word
#define SET_LOW_TIME        0x04    // set duration of low  trigger phase + 1 word (*32 microseconds)
#define SET_LED_INTENSITY   0x06    // *set brightness of strobe LEDs + 1 byte (0=off to 7=brightest)

static char message[48];

static char *makeCommandStr(const char *command) {
    // convert a command into a checksummed message
    // message must be large enough to hold strlen(command)+4
    // return 0, or >0 if error
    size_t length = strlen(command);
    if ((length > 8) || (length & 1)) return NULL;
    int cs = 0xFF;
    for (int i=0; i < length;) {
        int v;
        char c = command[i++];
        if (c >= '0' && c <= '9')       v = (c - '0') * 16;
        else if (c >= 'A' && c <= 'F')  v = (c - 'A' + 10) * 16;
        else return NULL;
        c = command[i++];
        if (c >= '0' && c <= '9')       v += (c - '0');
        else if (c >= 'A' && c <= 'F')  v += (c - 'A' + 10);
        else return NULL;
        cs ^= v;
    }
    sprintf(message, "%c%s%02X%c", SOM, command, cs, EOM);
    return message;
}

static int sendCommandStr(int fd, char *command) {
    if (!makeCommandStr(command)) {
        printf("***** error making command string for '%s'\n", command);
        return -1;
    }
    write(fd, message, strlen(message));
    return 0;
}


int main(void)
{
    int i, fd;
    int r = 1;
    char serialCommand[32];

    printf(">>> calling open(" COMM_PATH ")\n");
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

    printf(">>> writing reset command to " COMM_PATH "\n");
    write(fd, "!0DF2*", 6);
    usleep(1100000);

    #define FRAME_PERIOD 50//ms
    printf(">>> writing frame period command (%dms) to " COMM_PATH "\n", FRAME_PERIOD);
    sprintf(serialCommand, "%02X%04X", SET_LOW_TIME, FRAME_PERIOD*100); // frame period in 10's of microseconds
    if (sendCommandStr(fd, serialCommand)) goto out;

    #define LED_BRIGHTNESS 0x10    
    printf(">>> writing LED brightness 0x%02X to " COMM_PATH "\n", LED_BRIGHTNESS);
    sprintf(serialCommand, "%02X%02X", SET_LED_INTENSITY, LED_BRIGHTNESS);
    if (sendCommandStr(fd, serialCommand)) goto out;

    printf("=== calling b3dfg_init()\n");
    if ((i = b3dfg_init())) { fprintf(stderr, "    ******* b3dfg_init failed, returning %d\n", i); goto out; }
    printf("            b3dfg_init returned %d\n", i);

    printf("=== calling b3dfg_open(0)\n");
    if (!(dev = b3dfg_open(0))) { fprintf(stderr, "    ******* b3dfg_open failed\n"); goto out; }
    printf("            b3dfg_open returned %p\n", dev);

    printf("=== calling b3dfg_get_wand_status()\n");
    i = b3dfg_get_wand_status(dev);
    printf("            b3dfg_get_wand_status returned %d\n", i);

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

    int buffer = 0;

    printf("=== calling b3dfg_queue_buffer(%p, %d)\n", dev, buffer);
    if ((r = b3dfg_queue_buffer(dev, buffer))) { fprintf(stderr, "    ******* b3dfg_queue_buffer failed\n"); goto out; }
    printf("            b3dfg_queue_buffer returned %d\n", r);
    buffer = (buffer+1)&1;

    printf("=== calling b3dfg_queue_buffer(%p, %d)\n", dev, buffer);
    if ((r = b3dfg_queue_buffer(dev, buffer))) { fprintf(stderr, "    ******* b3dfg_queue_buffer failed\n"); goto out; }
    printf("            b3dfg_queue_buffer returned %d\n", r);
    buffer = (buffer+1)&1;

    printf("=== calling b3dfg_set_transmission(%p, 1)\n", dev);
    if ((r = b3dfg_set_transmission(dev, 1))) { fprintf(stderr, "    ******* b3dfg_set_transmission failed\n"); goto out; }
    printf("            b3dfg_set_transmission returned %d\n", r);

    printf("=== start acquisition loop ---------------------------------------\n");

    #define NTESTS 150
    int ntrig[NTESTS] = {
        3,   2,   3,  33,   1,  55,   2, 118,   5, 333,
        9,   2,   1,  44,   1,   8,  34, 187,   3, 444,
       23,   9,   3,   2,   1,  89,  41,   5,   1, 555,
        2,   3,   4,   5,   1, 232,  99,  32,  16, 666,
        8,   8,   7,   6,   5,   4,   3,   2,   1, 777,
       12,   2,   1,  44,   1,   8,  34, 187,   3, 888,
        5, 222,   1,  47,   5,   3,  35, 198,   6,  99,
       27,  13,   3,   2,   1,  89,  41,   5,   1,  88,
        2, 503,   4,   5,   1, 232,  99,  32,  16,  77,
        8,   8,   7,   6, 445, 804, 703, 602, 501,  66,
       12,  82,   1,  44,   1,   8,  34, 387,   3,  55,
        5, 222,   1,  47, 715,   3,  35, 298,   6,  44,
       27,  13, 998,   2,   1,  89,  41,   5,   1,  33,
        2,   3,   4, 425,   1, 232, 999,  32,  16,  22,
        9,  34,   7,   6,  85, 404, 303, 202, 101,   1,
                        };
    int j;
    int maxtime = 0;
    int mintime = 9999;
    int maxdropped = 0;

    struct timespec t1s;
    clock_gettime(CLOCK_REALTIME, &t1s);
    time_t t0s = t1s.tv_sec;
    double t0 = t1s.tv_nsec / 1e9;

    for (j = 0; j < NTESTS; j++) {
        int ncaptures = ntrig[j];

        printf(">>> writing startNtriggers(%d) command to " COMM_PATH "\n", ncaptures);
        sprintf(serialCommand, "%02X%04X", DO_N_PULSES, ncaptures);
        if (sendCommandStr(fd, serialCommand)) goto out;

        for (i = 1; i <= ncaptures+0; i++) {
            printf("=== calling b3dfg_wait_buffer(%p, %d, %dms, &dropped)\n", dev, buffer, TIMEOUT);
            int dropped;
            if ((r = b3dfg_wait_buffer(dev, buffer, TIMEOUT, &dropped)) < 0) {
                if (r == -ETIMEDOUT) {
                    if (i == ncaptures+1){
                        printf("=== hit expected timeout\n");
                        break; // this is expected
                    }
                    fprintf(stderr, "    ******* b3dfg_wait_buffer timed out\n");
                } else
                    fprintf(stderr, "    ******* b3dfg_wait_buffer failed (%d)\n", r);
                goto out;
            }
            int atime = TIMEOUT-r;
            printf("            b3dfg_wait_buffer returned after %dms, dropped=%d\n", atime, dropped);

            if (i > 1 && atime > maxtime) maxtime = atime;
            if (i > 1 && atime < mintime) mintime = atime;
            if (dropped > maxdropped) maxdropped = dropped;
            clock_gettime(CLOCK_REALTIME, &t1s);
            double t1 = t1s.tv_sec-t0s + t1s.tv_nsec / 1e9;
        
            printf("=== calling b3dfg_queue_buffer(%p, %d)\n", dev, buffer);
            if ((r = b3dfg_queue_buffer(dev, buffer))) {
                fprintf(stderr, "    ******* b3dfg_queue_buffer failed\n");
                goto out;
            }
            printf("            b3dfg_queue_buffer returned %d\n", r);

            double seconds = t1 - t0;
            int minutes = seconds / 60;
            seconds -= minutes * 60;
            printf("=== got buffer %d after %4dms(%4dms/%4dms), test %d frame %3d, dropped %d(%d)  time=%dm%.3fs\n", buffer, atime, i==1?0:mintime, i==1?0:maxtime, j, i, dropped, maxdropped, minutes, seconds);

            buffer = (buffer+1)&1;
        }
        usleep(50000);
    }
//    printf(">>> writing stop command to " COMM_PATH "\n");
//    write(fd, "!02FD*", 6);

    printf("=== end   acquisition loop ---------------------------------------\n");

    printf("=== calling b3dfg_set_transmission(%p, 0)\n", dev);
    if ((r = b3dfg_set_transmission(dev, 0))) { fprintf(stderr, "    ******* set_transmission failed\n"); goto out; }
    printf("            b3dfg_set_transmission returned %d\n", r);
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

    printf("=== calling b3dfg_exit\n");
    b3dfg_exit();
    printf("            b3dfg_exit called\n");

    printf("=== calling close(" COMM_PATH ")\n");
    if (close(fd) == -1)
        fprintf(stderr, "close failed\n");

    return r;
}
