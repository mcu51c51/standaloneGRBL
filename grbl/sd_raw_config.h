#ifndef SD_RAW_CONFIG_H
#define SD_RAW_CONFIG_H

#include <stdint.h>

#define SD_RAW_SDHC 1

#if SD_RAW_SDHC
  typedef uint64_t offset_t;
#else
  typedef uint32_t offset_t;
#endif

#endif
