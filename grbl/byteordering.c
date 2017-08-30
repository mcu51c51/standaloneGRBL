#include "grbl.h"

uint16_t read16(const uint8_t *p) {
  return (((uint16_t)p[1]) << 8) | (((uint16_t)p[0]) << 0); }

uint32_t read32(const uint8_t *p) {
    return (((uint32_t)p[3]) << 24) | (((uint32_t)p[2]) << 16) | (((uint32_t)p[1]) << 8) | (((uint32_t)p[0]) << 0); }
