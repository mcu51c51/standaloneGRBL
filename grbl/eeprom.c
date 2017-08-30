#include "grbl.h"

#define EEPM1 5
#define EEPM0 4

unsigned char eeprom_get_char(unsigned int addr) {
  while(EECR & (1<<EEPE)) {}
  EEAR = addr;
  EECR = (1<<EERE);
  return EEDR; }

void eeprom_put_char(unsigned int addr, unsigned char new_value) {
  char old_value;
  char diff_mask;

  cli();

  while(EECR & (1<<EEPE)) {}
  #ifndef EEPROM_IGNORE_SELFPROG
    while(SPMCSR & (1<<SELFPRGEN)) {}
  #endif

  EEAR = addr;
  EECR = (1<<EERE);
  old_value = EEDR;
  diff_mask = old_value ^ new_value;

  if(diff_mask & new_value) {

    if(new_value != 0xff) {
      EEDR = new_value;
      EECR = (1<<EEMPE) | (0<<EEPM1) | (0<<EEPM0);
      EECR |= (1<<EEPE);
    } else {
      EECR = (1<<EEMPE) | (1<<EEPM0);
      EECR |= (1<<EEPE); }

  } else {

    if( diff_mask ) {
    EEDR = new_value;
    EECR = (1<<EEMPE) | (1<<EEPM1);
    EECR |= (1<<EEPE); } }

  sei(); }

void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;

  for(; size > 0; size--) {
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); }

  eeprom_put_char(destination, checksum); }

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;

  for(; size > 0; size--) {
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;
    *(destination++) = data; }

  return(checksum == eeprom_get_char(source)); }
