#include "byteordering.h"
#include "partition.h"
#include "fat.h"
#include "sd_raw_config.h"

#include <string.h>

#define FAT16_CLUSTER_FREE 0x0000
#define FAT16_CLUSTER_RESERVED_MIN 0xfff0
#define FAT16_CLUSTER_RESERVED_MAX 0xfff6
#define FAT16_CLUSTER_BAD 0xfff7
#define FAT16_CLUSTER_LAST_MIN 0xfff8
#define FAT16_CLUSTER_LAST_MAX 0xffff

#define FAT32_CLUSTER_FREE 0x00000000
#define FAT32_CLUSTER_RESERVED_MIN 0x0ffffff0
#define FAT32_CLUSTER_RESERVED_MAX 0x0ffffff6
#define FAT32_CLUSTER_BAD 0x0ffffff7
#define FAT32_CLUSTER_LAST_MIN 0x0ffffff8
#define FAT32_CLUSTER_LAST_MAX 0x0fffffff

#define FAT_DIRENTRY_DELETED 0xe5

struct fat_header_struct {
  offset_t size;

  offset_t fat_offset;
  uint32_t fat_size;

  uint16_t sector_size;
  uint16_t cluster_size;

  offset_t cluster_zero_offset;

  offset_t root_dir_offset;
  cluster_t root_dir_cluster; };

struct fat_fs_struct {
  struct partition_struct* partition;
  struct fat_header_struct header;
  cluster_t cluster_free; };

struct fat_file_struct {
  struct fat_fs_struct* fs;
  struct fat_dir_entry_struct dir_entry;
  offset_t pos;
  cluster_t pos_cluster; };

struct fat_dir_struct {
  struct fat_fs_struct* fs;
  struct fat_dir_entry_struct dir_entry;
  cluster_t entry_cluster;
  uint16_t entry_offset; };

struct fat_read_dir_callback_arg {
  struct fat_dir_entry_struct* dir_entry;
  uintptr_t bytes_read;
  uint8_t finished; };

static struct fat_fs_struct fat_fs_handles[1];
static struct fat_file_struct fat_file_handles[1];
static struct fat_dir_struct fat_dir_handles[1];

static uint8_t fat_read_header(struct fat_fs_struct *fs);
static cluster_t fat_get_next_cluster(const struct fat_fs_struct *fs, cluster_t cluster_num);
static offset_t fat_cluster_offset(const struct fat_fs_struct *fs, cluster_t cluster_num);
static uint8_t fat_dir_entry_read_callback(uint8_t *buffer, offset_t offset, void *p);

struct fat_fs_struct *fat_open(struct partition_struct* partition) {
  if(!partition) {
    return 0; }

  struct fat_fs_struct *fs = fat_fs_handles;
  uint8_t i;
  for(i = 0; i < 1; ++i) {
    if(!fs->partition) {
      break; }

    ++fs; }
  if(i >= 1) {
    return 0; }

  memset(fs, 0, sizeof(*fs));

  fs->partition = partition;
  if(!fat_read_header(fs)) {
    fs->partition = 0;
    return 0; }

  return fs; }

void fat_close(struct fat_fs_struct *fs) {
  if(!fs) {
    return; }

  fs->partition = 0; }

uint8_t fat_read_header(struct fat_fs_struct *fs) {
  if(!fs) {
    return 0; }

  struct partition_struct *partition = fs->partition;
  if(!partition) {
    return 0; }

  uint8_t buffer[37];

  offset_t partition_offset = (offset_t) partition->offset * 512;
  if(!partition->device_read(partition_offset + 0x0b, buffer, sizeof(buffer))) {
    return 0; }

  uint16_t bytes_per_sector = read16(&buffer[0x00]);
  uint16_t reserved_sectors = read16(&buffer[0x03]);
  uint8_t sectors_per_cluster = buffer[0x02];
  uint8_t fat_copies = buffer[0x05];
  uint16_t max_root_entries = read16(&buffer[0x06]);
  uint16_t sector_count_16 = read16(&buffer[0x08]);
  uint16_t sectors_per_fat = read16(&buffer[0x0b]);
  uint32_t sector_count = read32(&buffer[0x15]);
  uint32_t sectors_per_fat32 = read32(&buffer[0x19]);
  uint32_t cluster_root_dir = read32(&buffer[0x21]);

  if(sector_count == 0) {
    if(sector_count_16 == 0) {
      return 0;
    } else {
      sector_count = sector_count_16; } }
  if(sectors_per_fat != 0) {
    sectors_per_fat32 = sectors_per_fat;
  } else if(sectors_per_fat32 == 0) {
    return 0; }

  uint32_t data_sector_count = sector_count - reserved_sectors - sectors_per_fat32 * fat_copies - ((max_root_entries * 32 + bytes_per_sector - 1) / bytes_per_sector);
  uint32_t data_cluster_count = data_sector_count / sectors_per_cluster;
  if(data_cluster_count < 4085) {
    return 0;
  } else if(data_cluster_count < 65525) {
    partition->type = PARTITION_TYPE_FAT16;
  } else {
    partition->type = PARTITION_TYPE_FAT32; }

  struct fat_header_struct *header = &fs->header;
  memset(header, 0, sizeof(*header));

  header->size = (offset_t) sector_count * bytes_per_sector;

  header->fat_offset = partition_offset + (offset_t) reserved_sectors * bytes_per_sector;
  header->fat_size = (data_cluster_count + 2) * (partition->type == PARTITION_TYPE_FAT16 ? 2 : 4);

  header->sector_size = bytes_per_sector;
  header->cluster_size = (uint16_t) bytes_per_sector * sectors_per_cluster;

  if(partition->type == PARTITION_TYPE_FAT16) {
    header->root_dir_offset = header->fat_offset + (offset_t) fat_copies * sectors_per_fat * bytes_per_sector;

    header->cluster_zero_offset = header->root_dir_offset + (offset_t) max_root_entries * 32;
  } else {
    header->cluster_zero_offset = header->fat_offset + (offset_t) fat_copies * sectors_per_fat32 * bytes_per_sector;

    header->root_dir_cluster = cluster_root_dir; }

  return 1; }

cluster_t fat_get_next_cluster(const struct fat_fs_struct *fs, cluster_t cluster_num) {
  if(!fs || cluster_num < 2) {
    return 0; }

  if(fs->partition->type == PARTITION_TYPE_FAT32) {
    uint32_t fat_entry;
    if(!fs->partition->device_read(fs->header.fat_offset + (offset_t) cluster_num * sizeof(fat_entry), (uint8_t*) &fat_entry, sizeof(fat_entry))) {
      return 0; }

    cluster_num = ltoh32(fat_entry);

    if(cluster_num == FAT32_CLUSTER_FREE || cluster_num == FAT32_CLUSTER_BAD || (cluster_num >= FAT32_CLUSTER_RESERVED_MIN && cluster_num <= FAT32_CLUSTER_RESERVED_MAX) || (cluster_num >= FAT32_CLUSTER_LAST_MIN && cluster_num <= FAT32_CLUSTER_LAST_MAX)) {
      return 0; }
  } else {
    uint16_t fat_entry;
    if(!fs->partition->device_read(fs->header.fat_offset + (offset_t) cluster_num * sizeof(fat_entry), (uint8_t*) &fat_entry, sizeof(fat_entry))) {
      return 0; }

    cluster_num = ltoh16(fat_entry);

    if(cluster_num == FAT16_CLUSTER_FREE || cluster_num == FAT16_CLUSTER_BAD || (cluster_num >= FAT16_CLUSTER_RESERVED_MIN && cluster_num <= FAT16_CLUSTER_RESERVED_MAX) || (cluster_num >= FAT16_CLUSTER_LAST_MIN && cluster_num <= FAT16_CLUSTER_LAST_MAX)) {
      return 0; } }

  return cluster_num; }

offset_t fat_cluster_offset(const struct fat_fs_struct *fs, cluster_t cluster_num) {
  if(!fs || cluster_num < 2) {
    return 0; }

  return fs->header.cluster_zero_offset + (offset_t) (cluster_num - 2) * fs->header.cluster_size; }

uint8_t fat_get_dir_entry_of_path(struct fat_fs_struct *fs, const char* path, struct fat_dir_entry_struct *dir_entry) {
  if(!fs || !path || path[0] == '\0' || !dir_entry) {
    return 0; }

  if(path[0] == '/') {
    ++path; }

  memset(dir_entry, 0, sizeof(*dir_entry));
  dir_entry->attributes = FAT_ATTRIB_DIR;

  while(1) {
    if(path[0] == '\0') {
      return 1; }

    struct fat_dir_struct *dd = fat_open_dir(fs, dir_entry);
    if(!dd) {
      break; }

    const char *sub_path = strchr(path, '/');
    uint8_t length_to_sep;
    if(sub_path) {
      length_to_sep = sub_path - path;
      ++sub_path;
    } else {
      length_to_sep = strlen(path);
      sub_path = path + length_to_sep; }

    while(fat_read_dir(dd, dir_entry)) {
      if((strlen(dir_entry->long_name) != length_to_sep || strncmp(path, dir_entry->long_name, length_to_sep) != 0)) {
        continue; }

      fat_close_dir(dd);
      dd = 0;

      if(path[length_to_sep] == '\0') {
        return 1; }

      if(dir_entry->attributes & FAT_ATTRIB_DIR) {
        path = sub_path;
        break; }

      return 0; }

    fat_close_dir(dd); }

  return 0; }

struct fat_file_struct *fat_open_file(struct fat_fs_struct *fs, const struct fat_dir_entry_struct *dir_entry) {
  if(!fs || !dir_entry || (dir_entry->attributes & FAT_ATTRIB_DIR)) {
    return 0; }

  struct fat_file_struct *fd = fat_file_handles;
  uint8_t i;
  for(i = 0; i < 1; ++i) {
    if(!fd->fs) {
      break; }

    ++fd; }
  if(i >= 1) {
    return 0; }

  memcpy(&fd->dir_entry, dir_entry, sizeof(*dir_entry));
  fd->fs = fs;
  fd->pos = 0;
  fd->pos_cluster = dir_entry->cluster;

  return fd; }

void fat_close_file(struct fat_file_struct *fd) {
  if(fd) {
    fd->fs = 0; } }

uint8_t fat_read_byte(struct fat_file_struct *fd) {
  if(!fd || fd->pos + 1 > fd->dir_entry.file_size) {
    return 0xff; }

  uint16_t cluster_size = fd->fs->header.cluster_size;
  cluster_t cluster_num = fd->pos_cluster;
  uint16_t first_cluster_offset = (uint16_t)(fd->pos & (cluster_size - 1));

  if(!cluster_num) {
    cluster_num = fd->dir_entry.cluster;

    if(!cluster_num) {
      return 0xff; }

    if(fd->pos) {
      uint32_t pos = fd->pos;
      while(pos >= cluster_size) {
        pos -= cluster_size;
        cluster_num = fat_get_next_cluster(fd->fs, cluster_num);
        if(!cluster_num) {
          return 0xff; } } } }

  offset_t cluster_offset = fat_cluster_offset(fd->fs, cluster_num) + first_cluster_offset;

  uint8_t buffer[1];
  if(!fd->fs->partition->device_read(cluster_offset, buffer, 1)) {
    return 0xff; }

  fd->pos++;

  if(first_cluster_offset + 1 >= cluster_size) {
    if((cluster_num = fat_get_next_cluster(fd->fs, cluster_num))) {
      first_cluster_offset = 0;
    } else {
      fd->pos_cluster = 0;
      return buffer[0]; } }

  fd->pos_cluster = cluster_num;

  return buffer[0]; }

struct fat_dir_struct* fat_open_dir(struct fat_fs_struct *fs, const struct fat_dir_entry_struct *dir_entry) {
  if(!fs || !dir_entry || !(dir_entry->attributes & FAT_ATTRIB_DIR)) {
    return 0; }

  struct fat_dir_struct* dd = fat_dir_handles;
  uint8_t i;
  for(i = 0; i < 1; ++i) {
    if(!dd->fs) {
      break; }

    ++dd; }
  if(i >= 1) {
    return 0; }

  memcpy(&dd->dir_entry, dir_entry, sizeof(*dir_entry));
  dd->fs = fs;
  dd->entry_cluster = dir_entry->cluster;
  dd->entry_offset = 0;

  return dd; }

void fat_close_dir(struct fat_dir_struct *dd) {
  if(dd) {
    dd->fs = 0; } }

uint8_t fat_read_dir(struct fat_dir_struct *dd, struct fat_dir_entry_struct *dir_entry) {
  if(!dd || !dir_entry) {
    return 0; }

  struct fat_fs_struct* fs = dd->fs;
  const struct fat_header_struct* header = &fs->header;
  uint16_t cluster_size = header->cluster_size;
  cluster_t cluster_num = dd->entry_cluster;
  uint16_t cluster_offset = dd->entry_offset;
  struct fat_read_dir_callback_arg arg;

  if(cluster_offset >= cluster_size) {
    fat_reset_dir(dd);
    return 0; }

  memset(&arg, 0, sizeof(arg));
  memset(dir_entry, 0, sizeof(*dir_entry));
  arg.dir_entry = dir_entry;

  if(cluster_num == 0) {
    if(fs->partition->type == PARTITION_TYPE_FAT32) {
      cluster_num = header->root_dir_cluster;
    } else {
      cluster_size = header->cluster_zero_offset - header->root_dir_offset; } }

  uint8_t buffer[32];
  while(!arg.finished) {
    uint16_t cluster_left = cluster_size - cluster_offset;
    offset_t pos = cluster_offset;
    if(cluster_num == 0) {
      pos += header->root_dir_offset;
    } else {
      pos += fat_cluster_offset(fs, cluster_num); }

    arg.bytes_read = 0;
    if(!fs->partition->device_read_interval(pos, buffer, sizeof(buffer), cluster_left, fat_dir_entry_read_callback, &arg)) {
      return 0; }

    cluster_offset += arg.bytes_read;

    if(cluster_offset >= cluster_size) {
      if((cluster_num = fat_get_next_cluster(fs, cluster_num)) != 0) {
        cluster_offset = 0;
        continue; }
      if(!arg.finished) {
        fat_reset_dir(dd);
        return 0; }
      break; } }

  dd->entry_cluster = cluster_num;
  dd->entry_offset = cluster_offset;

  return arg.finished; }

uint8_t fat_reset_dir(struct fat_dir_struct *dd) {
  if(!dd) {
    return 0; }

  dd->entry_cluster = dd->dir_entry.cluster;
  dd->entry_offset = 0;
  return 1; }

uint8_t fat_dir_entry_read_callback(uint8_t *buffer, offset_t offset, void *p) {
  struct fat_read_dir_callback_arg *arg = p;
  struct fat_dir_entry_struct *dir_entry = arg->dir_entry;

  arg->bytes_read += 32;

  if(buffer[0] == FAT_DIRENTRY_DELETED || !buffer[0]) {
    return 1; }

  if(buffer[11] == 0x0f) {
    return 1; }

  char *long_name = dir_entry->long_name;
  
  memset(dir_entry, 0, sizeof(*dir_entry));
  dir_entry->entry_offset = offset;

  uint8_t i;
  for(i = 0; i < 8; ++i) {
    if(buffer[i] == ' ') {
      break; }
    long_name[i] = buffer[i];

    if((buffer[12] & 0x08) && buffer[i] >= 'A' && buffer[i] <= 'Z') {
      long_name[i] += 'a' - 'A'; } }

  if(long_name[0] == 0x05) {
    long_name[0] = (char) FAT_DIRENTRY_DELETED; }

  if(buffer[8] != ' ') {
    long_name[i++] = '.';

    uint8_t j = 8;
    for(; j < 11; ++j) {
      if(buffer[j] == ' ') {
        break; }
      long_name[i] = buffer[j];

      if((buffer[12] & 0x10) && buffer[j] >= 'A' && buffer[j] <= 'Z') {
        long_name[i] += 'a' - 'A'; }

      ++i; } }

  long_name[i] = '\0';

  dir_entry->attributes = buffer[11];
  dir_entry->cluster = read16(&buffer[26]);
  dir_entry->cluster |= ((cluster_t) read16(&buffer[20])) << 16;
  dir_entry->file_size = read32(&buffer[28]);

  arg->finished = 1;
  return 0; }
