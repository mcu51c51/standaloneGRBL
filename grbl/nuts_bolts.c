#include "grbl.h"

uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr) {
  char *ptr = line + *char_counter;
  unsigned char c;

  c = *ptr++;

  bool isnegative = 0;
  if (c == '-') {
    isnegative = 1;
    c = *ptr++; }

  uint32_t intval = 0;
  int8_t exp = 0;
  uint8_t ndigit = 0;
  bool isdecimal = 0;

  while(1) {
    c -= '0';
    if (c <= 9) {
      ndigit++;
      if (isdecimal) { exp--; }
      intval = (((intval << 2) + intval) << 1) + c;
    } else if (c == (('.'-'0') & 0xff) && !(isdecimal)) {
      isdecimal = 1;
    } else {
      break; }
    c = *ptr++; }

  if (!ndigit) { return(0); }

  float fval;
  fval = (float)intval;

  if (fval != 0) {
    while (exp <= -2) {
      fval *= 0.01;
      exp += 2; }
    if (exp < 0) {
      fval *= 0.1; } }

  if (isnegative) {
    *float_ptr = -fval;
  } else {
    *float_ptr = fval; }

  *char_counter = ptr - line - 1;

  return(1); }

void delay_ms(uint16_t ms) {
  while (ms--) { _delay_ms(1); } }

float convert_delta_vector_to_unit_vector(float *vector) {
  uint8_t idx;
  float magnitude = 0.0;

  for (idx=0; idx<N_AXIS; idx++) {
    if (vector[idx] != 0.0) {
      magnitude += vector[idx]*vector[idx]; } }

  magnitude = sqrt(magnitude);

  float inv_magnitude = 1.0/magnitude;

  for (idx=0; idx<N_AXIS; idx++) { vector[idx] *= inv_magnitude; }

  return(magnitude); }

float limit_value_by_axis_maximum(float *max_value, float *unit_vec) {
  uint8_t idx;
  float limit_value = SOME_LARGE_VALUE;
  for (idx=0; idx<N_AXIS; idx++) {
    if (unit_vec[idx] != 0) {
      limit_value = min(limit_value, fabs(max_value[idx]/unit_vec[idx])); } }
  return(limit_value); }
