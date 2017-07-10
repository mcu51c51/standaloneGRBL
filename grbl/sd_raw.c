#include <string.h>
#include <avr/io.h>
#include "sd_raw.h"

#define CMD_GO_IDLE_STATE 0x00
#define CMD_SEND_OP_COND 0x01
#define CMD_SEND_IF_COND 0x08
#define CMD_SET_BLOCKLEN 0x10
#define CMD_READ_SINGLE_BLOCK 0x11
#define CMD_SD_SEND_OP_COND 0x29
#define CMD_APP 0x37
#define CMD_READ_OCR 0x3a

#define R1_IDLE_STATE 0
#define R1_ILL_COMMAND 2

#define SD_RAW_SPEC_1 0
#define SD_RAW_SPEC_2 1
#define SD_RAW_SPEC_SDHC 2

static uint8_t raw_block[512];
static offset_t raw_block_address;

static uint8_t sd_raw_card_type;

static void sd_raw_send_byte(uint8_t b);
static uint8_t sd_raw_rec_byte();
static uint8_t sd_raw_send_command(uint8_t command, uint32_t arg);

uint8_t sd_raw_init() {
  DDRB |= (1<<DDB3);
  DDRB |= (1<<DDB5);
  DDRB |= (1<<DDB2);
  DDRB &= ~(1<<DDB4);
  PORTB |= (1<<DDB4);

  unselect_card();

  SPCR = (0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR1)|(1<<SPR0);
  SPSR &= ~(1<<SPI2X);

  sd_raw_card_type = 0;

  for(uint8_t i = 0; i < 10; ++i) {
    sd_raw_rec_byte(); }

  select_card();

  uint8_t response;
  for(uint16_t i = 0; ; ++i) {
    response = sd_raw_send_command(CMD_GO_IDLE_STATE, 0);
    if(response == (1 << R1_IDLE_STATE)) {
      break; }

    if(i == 0x1ff) {
      unselect_card();
      return 0; } }

#if SD_RAW_SDHC
  response = sd_raw_send_command(CMD_SEND_IF_COND, 0x100 | 0xaa);
  if((response & (1 << R1_ILL_COMMAND)) == 0) {
    sd_raw_rec_byte();
    sd_raw_rec_byte();
    if((sd_raw_rec_byte() & 0x01) == 0) {
      return 0; }
    if(sd_raw_rec_byte() != 0xaa) {
      return 0; }
    sd_raw_card_type |= (1 << SD_RAW_SPEC_2);
  } else
#endif
  {
    sd_raw_send_command(CMD_APP, 0);
    response = sd_raw_send_command(CMD_SD_SEND_OP_COND, 0);
    if((response & (1 << R1_ILL_COMMAND)) == 0) {
      sd_raw_card_type |= (1 << SD_RAW_SPEC_1); } }

  for(uint16_t i = 0; ; ++i) {
    if(sd_raw_card_type & ((1 << SD_RAW_SPEC_1) | (1 << SD_RAW_SPEC_2))) {
      uint32_t arg = 0;
#if SD_RAW_SDHC
      if(sd_raw_card_type & (1 << SD_RAW_SPEC_2)) {
        arg = 0x40000000; }
#endif
      sd_raw_send_command(CMD_APP, 0);
      response = sd_raw_send_command(CMD_SD_SEND_OP_COND, arg);
    } else {
      response = sd_raw_send_command(CMD_SEND_OP_COND, 0); }

    if((response & (1 << R1_IDLE_STATE)) == 0) {
      break; }

    if(i == 0x7fff) {
      unselect_card();
      return 0; } }

#if SD_RAW_SDHC
  if(sd_raw_card_type & (1 << SD_RAW_SPEC_2)) {
    if(sd_raw_send_command(CMD_READ_OCR, 0)) {
      unselect_card();
      return 0; }
    if(sd_raw_rec_byte() & 0x40) {
      sd_raw_card_type |= (1 << SD_RAW_SPEC_SDHC); }

    sd_raw_rec_byte();
    sd_raw_rec_byte();
    sd_raw_rec_byte(); }
#endif

  if(sd_raw_send_command(CMD_SET_BLOCKLEN, 512)) {
    unselect_card();
    return 0; }

  unselect_card();

  SPCR &= ~((1<<SPR1)|(1<<SPR0));
  SPSR |= (1<<SPI2X);

  raw_block_address = (offset_t) - 1;
  if(!sd_raw_read(0, raw_block, sizeof(raw_block))) {
    return 0; }

  return 1; }

void sd_raw_send_byte(uint8_t b) {
  SPDR = b;
  while(!(SPSR & (1<<SPIF)));
  SPSR &= ~(1<<SPIF); }

uint8_t sd_raw_rec_byte() {
  SPDR = 0xff;
  while(!(SPSR & (1 << SPIF)));
  SPSR &= ~(1 << SPIF);

  return SPDR; }

uint8_t sd_raw_send_command(uint8_t command, uint32_t arg) {
  uint8_t response;

  sd_raw_rec_byte();

  sd_raw_send_byte(0x40 | command);
  sd_raw_send_byte((arg >> 24) & 0xff);
  sd_raw_send_byte((arg >> 16) & 0xff);
  sd_raw_send_byte((arg >> 8) & 0xff);
  sd_raw_send_byte((arg >> 0) & 0xff);
  switch(command) {
    case CMD_GO_IDLE_STATE:
      sd_raw_send_byte(0x95);
      break;
    case CMD_SEND_IF_COND:
      sd_raw_send_byte(0x87);
      break;
    default:
      sd_raw_send_byte(0xff);
      break; }

  for(uint8_t i = 0; i < 10; ++i) {
    response = sd_raw_rec_byte();
    if(response != 0xff) {
      break; } }

  return response; }

uint8_t sd_raw_read(offset_t offset, uint8_t *buffer, uintptr_t length) {
  offset_t block_address;
  uint16_t block_offset;
  uint16_t read_length;
  while(length > 0) {
    block_offset = offset & 0x01ff;
    block_address = offset - block_offset;
    read_length = 512 - block_offset;
    if(read_length > length) {
      read_length = length; }

    if(block_address != raw_block_address) {
      select_card();
#if SD_RAW_SDHC
      if(sd_raw_send_command(CMD_READ_SINGLE_BLOCK, (sd_raw_card_type & (1 << SD_RAW_SPEC_SDHC) ? block_address / 512 : block_address)))
#else
      if(sd_raw_send_command(CMD_READ_SINGLE_BLOCK, block_address))
#endif
      {
        unselect_card();
        return 0; }

      while(sd_raw_rec_byte() != 0xfe);

      uint8_t* cache = raw_block;
      for(uint16_t i = 0; i < 512; ++i) {
        *cache++ = sd_raw_rec_byte(); }
      raw_block_address = block_address;

      memcpy(buffer, raw_block + block_offset, read_length);
      buffer += read_length;

      sd_raw_rec_byte();
      sd_raw_rec_byte();

      unselect_card();

      sd_raw_rec_byte();
    } else {
      memcpy(buffer, raw_block + block_offset, read_length);
      buffer += read_length; }
    length -= read_length;
    offset += read_length; }

  return 1; }

uint8_t sd_raw_read_interval(offset_t offset, uint8_t *buffer, uintptr_t interval, uintptr_t length, sd_raw_read_interval_handler_t callback, void *p) {
  if(!buffer || interval == 0 || length < interval || !callback) {
    return 0; }

  while(length >= interval) {
    if(!sd_raw_read(offset, buffer, interval)) {
      return 0; }
    if(!callback(buffer, offset, p)) {
      break; }
    offset += interval;
    length -= interval; }

  return 1; }
