#include "grbl.h"

settings_t settings;

void settings_read_coord_data() {
  if (!(memcpy_from_eeprom_with_checksum((char*)&wco, EEPROM_ADDR_PARAMETERS, sizeof(wco)))) {
    report_status_message(STATUS_SETTING_READ_FAIL);
    memset(wco, 0, sizeof(wco));
    settings_write_coord_data(); } }

void settings_write_coord_data() {
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_PARAMETERS, (char*)&wco, sizeof(wco)); }

void write_global_settings() {
  eeprom_put_char(0, SETTINGS_VERSION);
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_GLOBAL, (char*)&settings, sizeof(settings_t)); }

void settings_restore() {
  settings.step_invert_mask = DEFAULT_STEPPING_INVERT_MASK;
  settings.dir_invert_mask = DEFAULT_DIRECTION_INVERT_MASK;
  settings.junction_deviation = DEFAULT_JUNCTION_DEVIATION;
  settings.flags = 0;
  if (DEFAULT_HOMING_ENABLE) { settings.flags |= BITFLAG_HOMING_ENABLE; }
  if (DEFAULT_INVERT_LIMIT_PINS) { settings.flags |= BITFLAG_INVERT_LIMIT_PINS; }
  settings.steps_per_mm[X_AXIS] = DEFAULT_XY_STEPS_PER_MM;
  settings.steps_per_mm[Y_AXIS] = DEFAULT_XY_STEPS_PER_MM;
  settings.max_rate[X_AXIS] = DEFAULT_XY_MAX_VELOCITY;
  settings.max_rate[Y_AXIS] = DEFAULT_XY_MAX_VELOCITY;
  settings.acceleration[X_AXIS] = DEFAULT_XY_MAX_ACCELERATION;
  settings.acceleration[Y_AXIS] = DEFAULT_XY_MAX_ACCELERATION;
  settings.max_travel[X_AXIS] = DEFAULT_X_MAX_TRAVEL;
  settings.max_travel[Y_AXIS] = DEFAULT_Y_MAX_TRAVEL;
  write_global_settings();

  memset(wco, 0, sizeof(wco));
  settings_write_coord_data(); }

uint8_t read_global_settings() {
  uint8_t version = eeprom_get_char(0);
  if (version == SETTINGS_VERSION) {
    if (!(memcpy_from_eeprom_with_checksum((char*)&settings, EEPROM_ADDR_GLOBAL, sizeof(settings_t)))) {
      return(0); }
  } else {
    return(0); }
  return(1); }

void settings_init() {
  if(!read_global_settings()) {
    report_status_message(STATUS_SETTING_READ_FAIL);
    settings_restore(); } }

uint8_t get_step_pin_mask(uint8_t axis_idx) {
  if (axis_idx == X_AXIS) {
    return((1<<X_STEP_BIT));
  } else {
    return((1<<Y_STEP_BIT)); } }

uint8_t get_direction_pin_mask(uint8_t axis_idx) {
  if (axis_idx == X_AXIS) {
    return((1<<X_DIRECTION_BIT));
  } else {
    return((1<<Y_DIRECTION_BIT)); } }

uint8_t settings_store_global_setting(uint8_t parameter, float value) {
  if (value < 0.0) { value *= -1.0; }
  uint8_t int_value = trunc(value);
  switch(parameter) {
    case 0:
      settings.step_invert_mask = int_value;
      st_generate_step_dir_invert_masks();
      break;
    case 1:
      settings.dir_invert_mask = int_value;
      st_generate_step_dir_invert_masks();
      break;
    case 2:
      settings.steps_per_mm[X_AXIS] = value;
      break;
    case 3:
      settings.steps_per_mm[Y_AXIS] = value;
      break;
    case 4:
      settings.max_rate[X_AXIS] = value*60;
      break;
    case 5:
      settings.max_rate[Y_AXIS] = value*60;
      break;
    case 6:
      settings.acceleration[X_AXIS] = value*(60*60);
      break;
    case 7:
      settings.acceleration[Y_AXIS] = value*(60*60);
      break;
    case 8:
      settings.max_travel[X_AXIS] = value;
      break;
    case 9:
      settings.max_travel[Y_AXIS] = value;
      break;
    case 10:
      settings.junction_deviation = value;
      break;
    case 11:
      if (int_value) { settings.flags |= BITFLAG_INVERT_LIMIT_PINS; }
      else { settings.flags &= ~BITFLAG_INVERT_LIMIT_PINS; }
      break;
    case 12:
      if (int_value) { settings.flags |= BITFLAG_HOMING_ENABLE; }
      else {
        settings.flags &= ~BITFLAG_HOMING_ENABLE;
        memset(wco, 0, sizeof(wco));
        settings_write_coord_data(); }
      break;
    default:
      return(STATUS_INVALID_STATEMENT); }
  write_global_settings();
  return(STATUS_OK); }
