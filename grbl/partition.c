#include "byteordering.h"
#include "partition.h"

#include <string.h>

static struct partition_struct partition_handles[1];

struct partition_struct *partition_open(device_read_t device_read, device_read_interval_t device_read_interval, int8_t index) {
  struct partition_struct *new_partition = 0;
  uint8_t buffer[0x10];

  if(!device_read || !device_read_interval || index >= 4) {
    return 0; }

  if(index >= 0) {
    if(!device_read(0x01be + index * 0x10, buffer, sizeof(buffer))) {
      return 0; }

    if(buffer[4] == 0x00) {
      return 0; } }

  new_partition = partition_handles;
  uint8_t i;
  for(i = 0; i < 1; ++i) {
    if(new_partition->type == PARTITION_TYPE_FREE) {
      break; }

    ++new_partition; }

  if(i >= 1) {
    return 0; }

  memset(new_partition, 0, sizeof(*new_partition));

  new_partition->device_read = device_read;
  new_partition->device_read_interval = device_read_interval;

  if(index >= 0) {
    new_partition->type = buffer[4];
    new_partition->offset = read32(&buffer[8]);
    new_partition->length = read32(&buffer[12]);
  } else {
    new_partition->type = 0xff; }

  return new_partition; }

void partition_close(struct partition_struct *partition) {
  if(partition) {
    partition->type = PARTITION_TYPE_FREE; } }
