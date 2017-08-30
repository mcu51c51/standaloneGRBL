#ifndef serial_h
#define serial_h

#define BAUD_RATE 115200

#define RX_BUFFER_SIZE 30
#define TX_BUFFER_SIZE 20

#define SERIAL_NO_DATA 0xff

#define CMD_RESET 0x18
#define CMD_CYCLE_START '~'
#define CMD_FEED_HOLD '!'

void serial_init();
void serial_write(uint8_t data);
uint8_t serial_read();
void serial_reset_read_buffer();

#endif
