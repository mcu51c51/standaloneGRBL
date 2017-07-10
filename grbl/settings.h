#ifndef settings_h
#define settings_h

#include "grbl.h"

#define BITFLAG_HOMING_ENABLE     bit(1)
#define BITFLAG_INVERT_LIMIT_PINS bit(0)

typedef struct {
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  float junction_deviation;
  uint8_t flags;
} settings_t;
extern settings_t settings;

void settings_init();
void settings_restore();
uint8_t settings_store_global_setting(uint8_t parameter, float value);
void settings_write_coord_data();
void settings_read_coord_data();
uint8_t get_step_pin_mask(uint8_t i);
uint8_t get_direction_pin_mask(uint8_t i);

#endif
