#ifndef settings_h
#define settings_h

#define DEFAULT_XY_STEPS_PER_MM 160.0
#define DEFAULT_XY_MAX_VELOCITY (167.0*60)
#define DEFAULT_XY_MAX_ACCELERATION (3000.0*60*60)
#define DEFAULT_X_MAX_TRAVEL 297.0
#define DEFAULT_Y_MAX_TRAVEL 420.0
#define DEFAULT_STEPPING_INVERT_MASK 0
#define DEFAULT_DIRECTION_INVERT_MASK 0
#define DEFAULT_JUNCTION_DEVIATION 0.010
#define DEFAULT_INVERT_LIMIT_PINS 0
#define DEFAULT_HOMING_ENABLE 0

#define SETTINGS_VERSION 31

#define EEPROM_ADDR_GLOBAL 1U
#define EEPROM_ADDR_PARAMETERS 512U

#define BITFLAG_XY_HOME_PIN_AS_ST_ENABLE bit(2)
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
void settings_read_coord_data();
uint8_t get_step_pin_mask(uint8_t i);
uint8_t get_direction_pin_mask(uint8_t i);

#endif
