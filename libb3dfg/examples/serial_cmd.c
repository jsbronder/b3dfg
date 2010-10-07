/*
 * Brontes frame grabber example
 *
 * Copyright (c) 2009, 3M
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the 3M nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 *
 * Author:
 * 	Justin Bronder <jsbronder@brontes3d.com>
 *
 */


#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <serial_cmd.h>

static char message[48];
static char * str2cmd(const char *command)
{
    // convert a command into a checksummed message
    // message must be large enough to hold strlen(command)+4
    // return 0, or >0 if error
    size_t length = strlen(command);
    int cs = 0xFF;
    int i, v;
    char c;

    if ((length > 8) || (length & 1))
        return NULL;
    for (i = 0; i < length;) {
        c = command[i++];
        if (c >= '0' && c <= '9')       v = (c - '0') * 16;
        else if (c >= 'A' && c <= 'F')  v = (c - 'A' + 10) * 16;
        else return NULL;
        c = command[i++];
        if (c >= '0' && c <= '9')       v += (c - '0');
        else if (c >= 'A' && c <= 'F')  v += (c - 'A' + 10);
        else return NULL;
        cs ^= v;
    }
    sprintf(message, "!%s%02X*", command, cs);
    return message;
}

int send_cmd(int fd, char *command) {
    if (!str2cmd(command)){
        fprintf(stderr, "error:  failed to make command string for '%s'\n", command);
        return -1;
    }
    return write(fd, message, strlen(message));
}

int open_serial(const char *path) {
    int fd;
    int rc = 0;
    struct termios tio = {
        .c_iflag    = IGNPAR,
        .c_oflag    = 0,
        .c_cflag    = B115200|CS8|CLOCAL|CREAD,
        .c_lflag    = 0,
        .c_cc[VMIN] = 1,
        .c_cc[VTIME]= 0,
    };

    if ( (fd = open(path, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0 ){
        rc = errno;
        fprintf(stderr, "Cannot open %s:  %s\n", path, strerror(rc));
        return -1;
    }

    if (tcflush(fd, TCIFLUSH) == -1){
        rc = errno;
        fprintf(stderr, "tcflush: %s\n", strerror(rc));
        return -1;
    }

    if (tcsetattr(fd, TCSANOW, &tio) == -1){
        rc = errno;
        fprintf(stderr, "tcsetattr: %s\n", strerror(rc));
        return -1;
    }

    return fd;
}





