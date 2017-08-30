#ifndef limits_h
#define limits_h

#define HOMING_AXIS_SEARCH_SCALAR 1.1
#define HOMING_PULLOFF 10.0
#define HOMING_FEED_RATE 600.0
#define HOMING_DEBOUNCE_DELAY 200

#define limit_get_state() ((bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) ? ((LIMIT_PIN & (1<<XY_LIMIT_BIT))^(1<<XY_LIMIT_BIT)):(LIMIT_PIN & (1<<XY_LIMIT_BIT)))

uint8_t limits_go_home();

#endif
