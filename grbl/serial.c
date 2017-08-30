#include "grbl.h"

#define RX_RING_BUFFER (RX_BUFFER_SIZE+1)
#define TX_RING_BUFFER (TX_BUFFER_SIZE+1)

uint8_t serial_rx_buffer[RX_RING_BUFFER];
uint16_t serial_rx_buffer_head = 0;
volatile uint16_t serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_RING_BUFFER];
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;

void serial_init() {
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2;
    UCSR0A &= ~(1 << U2X0);
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;

  UCSR0B |= (1<<RXEN0 | 1<<TXEN0 | 1<<RXCIE0); }

void serial_write(uint8_t data) {
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_RING_BUFFER) { next_head = 0; }

  while (next_head == serial_tx_buffer_tail) {}

  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  UCSR0B |= (1 << UDRIE0); }

ISR(USART_UDRE_vect) {
  uint8_t tail = serial_tx_buffer_tail;

  UDR0 = serial_tx_buffer[tail];

  tail++;
  if (tail == TX_RING_BUFFER) { tail = 0; }

  serial_tx_buffer_tail = tail;

  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); } }

uint8_t serial_read() {
  uint16_t tail = serial_rx_buffer_tail;
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER) { tail = 0; }
    serial_rx_buffer_tail = tail;

    return data; } }

ISR(USART_RX_vect) {
  uint8_t data = UDR0;
  uint16_t next_head;

  switch (data) {
    case CMD_RESET: system_set_exec_state_flag(EXEC_RESET); break;
    case CMD_FEED_HOLD: system_set_exec_state_flag(EXEC_FEED_HOLD); break;
    case CMD_CYCLE_START: system_set_exec_state_flag(EXEC_CYCLE_START); break;

    default:
      next_head = serial_rx_buffer_head + 1;
      if (next_head == RX_RING_BUFFER) { next_head = 0; }

      if (next_head != serial_rx_buffer_tail) {
        serial_rx_buffer[serial_rx_buffer_head] = data;
        serial_rx_buffer_head = next_head; } } }

void serial_reset_read_buffer() {
  serial_rx_buffer_tail = serial_rx_buffer_head; }
