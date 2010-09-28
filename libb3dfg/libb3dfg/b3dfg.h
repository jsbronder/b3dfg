/*
 * Brontes frame grabber access library public header
 *
 * Copyright (C) 2008 3M Company
 * Author: Daniel Drake <ddrake@brontes3d.com>
 *
 * Private software. Source code may not be distributed.
 */

#ifndef __B3DFG_H___
#define __B3DFG_H___
#include <sys/time.h>

struct b3dfg_dev;
typedef struct b3dfg_dev b3dfg_dev;

struct b3dfg_buffer_state {
	unsigned int dropped;
	struct timeval stamp;
	int buffer;
	char *addr;
};
typedef struct b3dfg_buffer_state b3dfg_buffer_state;

b3dfg_dev *b3dfg_init(unsigned int idx);
void b3dfg_exit(b3dfg_dev *dev);

int b3dfg_open(b3dfg_dev *dev, int prefault);
void b3dfg_close(b3dfg_dev *dev);

int b3dfg_get_wand_status(b3dfg_dev *dev);
int b3dfg_get_wand_status_fd(b3dfg_dev *dev);

int b3dfg_get_fd(b3dfg_dev *dev);
unsigned int b3dfg_get_frame_size(b3dfg_dev *dev);

int b3dfg_release_buffer(b3dfg_dev *dev);
int b3dfg_get_buffer(b3dfg_dev *dev, unsigned int timeout, 
	b3dfg_buffer_state *state);
#endif

// vim: noet
