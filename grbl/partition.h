#ifndef PARTITION_H
#define PARTITION_H

#include <stdint.h>
#include "sd_raw_config.h"

#define PARTITION_TYPE_FREE 0x00
#define PARTITION_TYPE_FAT16 0x06
#define PARTITION_TYPE_FAT32 0x0b
#define PARTITION_TYPE_UNKNOWN 0xff

typedef uint8_t (*device_read_t)(offset_t offset, uint8_t *buffer, uintptr_t length);
typedef uint8_t (*device_read_callback_t)(uint8_t *buffer, offset_t offset, void *p);
typedef uint8_t (*device_read_interval_t)(offset_t offset, uint8_t *buffer, uintptr_t interval, uintptr_t length, device_read_callback_t callback, void *p);

struct partition_struct {
  device_read_t device_read;
  device_read_interval_t device_read_interval;
  uint8_t type;
  uint32_t offset;
  uint32_t length; };

struct partition_struct *partition_open(device_read_t device_read, device_read_interval_t device_read_interval, int8_t index);
void partition_close(struct partition_struct *partition);

#endif
