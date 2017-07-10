#ifndef SD_RAW_H
#define SD_RAW_H

#include <stdint.h>
#include "sd_raw_config.h"

#define select_card() PORTB &= ~(1<<PORTB2)
#define unselect_card() PORTB |= (1<<PORTB2)

typedef uint8_t (*sd_raw_read_interval_handler_t)(uint8_t *buffer, offset_t offset, void *p);

uint8_t sd_raw_init();

uint8_t sd_raw_read(offset_t offset, uint8_t *buffer, uintptr_t length);
uint8_t sd_raw_read_interval(offset_t offset, uint8_t* buffer, uintptr_t interval, uintptr_t length, sd_raw_read_interval_handler_t callback, void *p);

#endif
