#ifndef SD_RAW_H
#define SD_RAW_H

#define select_card() PORTB &= ~(1<<PORTB2)
#define unselect_card() PORTB |= (1<<PORTB2)

typedef uint8_t (*sd_raw_read_interval_handler_t)(uint8_t *buffer, uint64_t offset, void *p);

uint8_t sd_raw_init();

uint8_t sd_raw_read(uint64_t offset, uint8_t *buffer, uintptr_t length);
uint8_t sd_raw_read_interval(uint64_t offset, uint8_t* buffer, uintptr_t interval, uintptr_t length, sd_raw_read_interval_handler_t callback, void *p);

#endif
