#include "grbl.h"

uint8_t gc_execute_line(char *line) {
  uint8_t char_counter = 0;
  char letter;
  uint8_t dwell = 0;
  uint16_t P = 0;
  float f_val;
  uint8_t int_value;
  while (line[char_counter] != 0) {
    if ((letter = line[char_counter++]) == '(') {
      break; }
    if ('P' == letter || 'I' == letter) {
      if ((letter = line[char_counter]) == 'D' || letter == 'U') {
        for (idx=0; idx<N_AXIS; idx++) {
          char_counter++;
          if (!read_float(line, &char_counter, &f_val)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if (pl_data.units) {
            f_val *= 25.4; }
          if (pl_data.distance) {
            pl_data.gc_pos[idx] += f_val;
          } else {
            pl_data.gc_pos[idx] = f_val; }
          pl_data.xyz[idx] = lround(pl_data.gc_pos[idx]*settings.steps_per_mm[idx]) + wco[idx];
          if (bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE)) {
            if (pl_data.xyz[X_AXIS] < -0.5 || pl_data.xyz[X_AXIS] > lround((settings.max_travel[X_AXIS] + 0.5) * settings.steps_per_mm[X_AXIS])) {
              return(STATUS_SOFT_LIMIT_ERROR); } } }
        if (line[char_counter] != ';') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
        if (line[1] == 'D') {
          if (pl_data.condition & PL_COND_FLAG_RAPID_MOTION) {
            protocol_buffer_synchronize();
            if (sys.abort) { return(0); }
            int_value = ceil(1.0 / DWELL_TIME_STEP * 400));
            //spindle_set_speed(255);
            while (int_value > 0) {
              buttons_check();
              protocol_exec_rt_system();
              if (sys.abort) { return(0); }
              _delay_ms(DWELL_TIME_STEP);
              int_value--; } }
          pl_data.condition = 0;
        } else {
          if (0 == pl_data.condition) {
            protocol_buffer_synchronize();
            if (sys.abort) { return(0); }
            int_value = ceil(1.0 / DWELL_TIME_STEP * 400));
            //spindle_set_speed(0);
            while (int_value > 0) {
              buttons_check();
              protocol_exec_rt_system();
              if (sys.abort) { return(0); }
              _delay_ms(DWELL_TIME_STEP);
              int_value--; } }
          pl_data.condition = PL_COND_FLAG_RAPID_MOTION; }
        break;
      } else if (letter == 'N') {
        if (line[char_counter+1] != ';') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
        pl_data.condition = PL_COND_FLAG_RAPID_MOTION;
        pl_data.xyz[X_AXIS] = 0;
        pl_data.xyz[Y_AXIS] = 0;
        break;
      } else {
        return(STATUS_GCODE_UNSUPPORTED_COMMAND); } }
    if (!read_float(line, &char_counter, &f_val)) { return(STATUS_BAD_NUMBER_FORMAT); }
    switch (letter) {
      case 'G':
        switch ((int_value = trunc(f_val))) {
          case 0:
            pl_data.condition = PL_COND_FLAG_RAPID_MOTION; break;
          case 1:
            pl_data.condition = 0; break;
          case 4:
            dwell = 1; break;
          case 20:
            pl_data.units = 1; break;
          case 21:
            pl_data.units = 0; break;
          case 90:
            pl_data.distance = 0; break;
          case 91:
            pl_data.distance = 1; break;
          case 2: case 3: case 80: case 10: case 28: case 30: case 53: case 92: case 93: case 18: case 19: case 43: case 54: case 55: case 56: case 57: case 58: case 59:
            return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
        break;
      case 'X':
        if (pl_data.units) {
          f_val *= 25.4; }
        if (pl_data.distance) {
          pl_data.gc_pos[X_AXIS] += f_val;
        } else {
          pl_data.gc_pos[X_AXIS] = f_val; }
        pl_data.xyz[X_AXIS] = lround(pl_data.gc_pos[X_AXIS]*settings.steps_per_mm[X_AXIS]) + wco[X_AXIS];
        if (bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE)) {
          if (pl_data.xyz[X_AXIS] < -0.5 || pl_data.xyz[X_AXIS] > lround((settings.max_travel[X_AXIS] + 0.5) * settings.steps_per_mm[X_AXIS])) {
            return(STATUS_SOFT_LIMIT_ERROR); } }
        break;
      case 'Y':
        if (pl_data.units) {
          f_val *= 25.4; }
        if (pl_data.distance) {
          pl_data.gc_pos[Y_AXIS] += f_val;
        } else {
          pl_data.gc_pos[Y_AXIS] = f_val; }
        pl_data.xyz[Y_AXIS] = lround(-pl_data.gc_pos[Y_AXIS]*settings.steps_per_mm[Y_AXIS]) + wco[Y_AXIS];
        if (bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE)) {
          if (pl_data.xyz[Y_AXIS] < -0.5 || pl_data.xyz[Y_AXIS] > lround((settings.max_travel[Y_AXIS] + 0.5) * settings.steps_per_mm[Y_AXIS])) {
            return(STATUS_SOFT_LIMIT_ERROR); } }
        break;
      case 'S':
        if (f_val > SPINDLE_PWM_MAX_VALUE) {
          f_val = SPINDLE_PWM_MAX_VALUE;
        } else if (f_val > SPINDLE_PWM_OFF_VALUE && f_val < SPINDLE_PWM_MIN_VALUE+1) {
          f_val = SPINDLE_PWM_MIN_VALUE+1; }
        pl_data.spindle_speed = trunc(f_val);
        break;
      case 'F':
        pl_data.feed_rate = f_val;
        break;
      case 'P':
        P = ceil(f_val * 1000.0);
        break;
      case 'M':
        switch ((int_value = trunc(f_val))) {
          case 0:
            protocol_buffer_synchronize();
            if (sys.abort) { return(0); }
            sys.state = STATE_HOLD;
            SPINDLE_OCR_REGISTER = SPINDLE_PWM_MAX_VALUE;
            SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT);
            sys.spindle_speed = SPINDLE_PWM_MIN_VALUE;
            while ((sys.state == STATE_HOLD)) {
              protocol_execute_realtime();
              if (sys.abort) { return(0); } }
            return(STATUS_OK);
          case 3: case 4: case 5:
            protocol_buffer_synchronize();
            if (sys.abort) { return(0); }
            if (5 == int_value) {
              spindle_set_speed(SPINDLE_PWM_OFF_VALUE); }
            return(STATUS_OK);
          case 2: case 30:
            protocol_buffer_synchronize();
            return(STATUS_OK); } } }

  if (dwell && P > 0) {
    protocol_buffer_synchronize();
    if (sys.abort) { return(0); }
    sys.state = STATE_HOLD;
    sys.non_modal_dwell = max(1, ceil(1.0 / DWELL_TIME_STEP * P));
    spindle_set_speed(pl_data.spindle_speed);
    while (sys.non_modal_dwell > 0) {
      buttons_check();
      protocol_exec_rt_system();
      if (sys.abort) { return(0); }
      _delay_ms(DWELL_TIME_STEP);
      sys.non_modal_dwell--; }
    spindle_set_speed(SPINDLE_PWM_OFF_VALUE);
    return(STATUS_OK); }

  while (plan_check_full_buffer()) {
    protocol_execute_realtime();
    if (sys.abort) { return(0); } }

  plan_buffer_line();

  protocol_auto_cycle_start();
  protocol_execute_realtime();
  if (sys.abort) { return(0); }

  return(STATUS_OK); }
