#ifndef serial_h
#define serial_h

#define SERIAL_NO_DATA 0xff

void serial_init();
void serial_write(uint8_t data);
uint8_t serial_read();
void serial_reset_read_buffer();

#endif
