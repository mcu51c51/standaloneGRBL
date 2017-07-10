#ifndef FAT_H
#define FAT_H

#include <stdint.h>
#include "sd_raw_config.h"

#define FAT_ATTRIB_READONLY (1<<0)
#define FAT_ATTRIB_HIDDEN (1<<1)
#define FAT_ATTRIB_SYSTEM (1<<2)
#define FAT_ATTRIB_VOLUME (1<<3)
#define FAT_ATTRIB_DIR (1<<4)
#define FAT_ATTRIB_ARCHIVE (1<<5)

typedef uint32_t cluster_t;

struct partition_struct;
struct fat_fs_struct;
struct fat_file_struct;
struct fat_dir_struct;

struct fat_dir_entry_struct {
  char long_name[13];
  uint8_t attributes;
  cluster_t cluster;
  uint32_t file_size;
  offset_t entry_offset; };

struct fat_fs_struct *fat_open(struct partition_struct *partition);
void fat_close(struct fat_fs_struct *fs);

struct fat_file_struct *fat_open_file(struct fat_fs_struct *fs, const struct fat_dir_entry_struct *dir_entry);
void fat_close_file(struct fat_file_struct *fd);
uint8_t fat_read_byte(struct fat_file_struct *fd);

struct fat_dir_struct *fat_open_dir(struct fat_fs_struct *fs, const struct fat_dir_entry_struct *dir_entry);
void fat_close_dir(struct fat_dir_struct *dd);
uint8_t fat_read_dir(struct fat_dir_struct *dd, struct fat_dir_entry_struct *dir_entry);
uint8_t fat_reset_dir(struct fat_dir_struct *dd);

uint8_t fat_get_dir_entry_of_path(struct fat_fs_struct *fs, const char *path, struct fat_dir_entry_struct *dir_entry);

#endif
