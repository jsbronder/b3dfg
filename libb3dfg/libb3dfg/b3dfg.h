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

struct b3dfg_dev;
typedef struct b3dfg_dev b3dfg_dev;

int b3dfg_init(void);
void b3dfg_exit(void);

b3dfg_dev *b3dfg_open(unsigned int idx);
void b3dfg_close(b3dfg_dev *dev);

int b3dfg_get_fd(b3dfg_dev *dev);
unsigned int b3dfg_get_frame_size(b3dfg_dev *dev);
int b3dfg_set_transmission(b3dfg_dev *dev, int enabled);

int b3dfg_set_num_buffers(b3dfg_dev *dev, int buffers);
int b3dfg_queue_buffer(b3dfg_dev *dev, int buffer);
int b3dfg_poll_buffer(b3dfg_dev *dev, int buffer, unsigned int *dropped);
int b3dfg_wait_buffer(b3dfg_dev *dev, int buffer, unsigned int *dropped);

unsigned char *b3dfg_map_buffers(b3dfg_dev *dev, int prefault);
unsigned char *b3dfg_get_mapping(b3dfg_dev *dev);
void b3dfg_unmap_buffers(b3dfg_dev *dev);

#endif

