#ifndef FAT_H
#define FAT_H

#define PARTITION_TYPE_FREE 0x00
#define PARTITION_TYPE_FAT16 0x06
#define PARTITION_TYPE_FAT32 0x0b
#define PARTITION_TYPE_UNKNOWN 0xff

#define FAT_ATTRIB_READONLY (1<<0)
#define FAT_ATTRIB_HIDDEN (1<<1)
#define FAT_ATTRIB_SYSTEM (1<<2)
#define FAT_ATTRIB_VOLUME (1<<3)
#define FAT_ATTRIB_DIR (1<<4)
#define FAT_ATTRIB_ARCHIVE (1<<5)

typedef uint8_t (*device_read_t)(uint64_t offset, uint8_t *buffer, uintptr_t length);
typedef uint8_t (*device_read_callback_t)(uint8_t *buffer, uint64_t offset, void *p);
typedef uint8_t (*device_read_interval_t)(uint64_t offset, uint8_t *buffer, uintptr_t interval, uintptr_t length, device_read_callback_t callback, void *p);

struct partition_struct;
struct fat_fs_struct;
struct fat_file_struct;
struct fat_dir_struct;

struct partition_struct {
  device_read_t device_read;
  device_read_interval_t device_read_interval;
  uint8_t type;
  uint32_t offset;
  uint32_t length; };

struct fat_dir_entry_struct {
  char long_name[13];
  uint8_t attributes;
  uint32_t cluster;
  uint32_t file_size;
  uint64_t entry_offset; };

struct partition_struct *partition_open(device_read_t device_read, device_read_interval_t device_read_interval, int8_t index);
void partition_close(struct partition_struct *partition);

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
