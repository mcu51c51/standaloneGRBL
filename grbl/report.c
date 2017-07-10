#include "grbl.h"

static void report_util_line_feed() { printPgmString(PSTR("\n")); }

void report_status_message(uint8_t status_code) {
  if (status_code == STATUS_OK) {
    printPgmString(PSTR("ok"));
  } else {
    printPgmString(PSTR("error:"));
    print_uint8_base10(status_code); }
  report_util_line_feed(); }

void report_build_info() {
  serial_write('[');
  printPgmString(PSTR(GRBL_VERSION "." GRBL_VERSION_BUILD));
  serial_write(',');
  printInteger(RX_BUFFER_SIZE);
  serial_write(']');
  report_util_line_feed(); }

void report_grbl_settings() {
  serial_write('{');
  print_uint8_base10(settings.step_invert_mask);
  serial_write(',');
  print_uint8_base10(settings.dir_invert_mask);
  serial_write(',');
  printFloat(settings.steps_per_mm[X_AXIS], 3);
  serial_write(',');
  printFloat(settings.steps_per_mm[Y_AXIS], 3);
  serial_write(',');
  printFloat(settings.max_rate[X_AXIS]/60, 0);
  serial_write(',');
  printFloat(settings.max_rate[Y_AXIS]/60, 0);
  serial_write(',');
  printFloat(settings.acceleration[X_AXIS]/(60*60), 0);
  serial_write(',');
  printFloat(settings.acceleration[Y_AXIS]/(60*60), 0);
  serial_write(',');
  printFloat(settings.max_travel[X_AXIS], 1);
  serial_write(',');
  printFloat(settings.max_travel[Y_AXIS], 1);
  serial_write(',');
  print_uint8_base10(bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS));
  serial_write(',');
  printFloat(settings.junction_deviation, 3);
  serial_write(',');
  print_uint8_base10(bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE));
  serial_write('}');
  report_util_line_feed(); }
