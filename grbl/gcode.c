#include "grbl.h"

uint8_t gc_execute_line(char *line) {
  uint8_t char_counter = 1;
  uint8_t idx = 0;
  int32_t value;
  float f_val;

  switch(line[0]) {

    case '5': case '6': case '4': case '7':
      if (!read_float(line, &char_counter, &f_val)) { return(STATUS_BAD_NUMBER_FORMAT); }
      if (line[0] != '7') {
        pl_data.xyz[X_AXIS] = lround(f_val*settings.steps_per_mm[X_AXIS]) + wco[X_AXIS];
        if (line[0] == '6') {
          char_counter++;
          if (!read_float(line, &char_counter, &pl_data.feed_rate)) { return(STATUS_BAD_NUMBER_FORMAT); } }
        if (line[char_counter] == ',') {
          char_counter++;
          if (!read_int(line, &char_counter, &pl_data.spindle_speed)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if (line[0] == '5') { pl_data.laserOn = 0; } }
      } else {
        pl_data.xyz[Y_AXIS] = lround(f_val*settings.steps_per_mm[Y_AXIS]) + wco[Y_AXIS]; }
      if (line[char_counter] != ';') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
      if (line[0] == '5' || line[0] == '6') {
        pl_data.condition = 0;
        if (line[0] == '5') { pl_data.laserOn = !pl_data.laserOn; }
      } else {
        pl_data.condition = PL_COND_FLAG_RAPID_MOTION;
        pl_data.laserOn = 1; }
      break;

    case 'G':
      if (line[1] != 'F' && line[1] != 'R') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
      for (idx=0; idx<N_AXIS; idx++) {
        char_counter++;
        if (!read_float(line, &char_counter, &f_val)) { return(STATUS_BAD_NUMBER_FORMAT); }
        pl_data.xyz[idx] = lround(f_val*settings.steps_per_mm[idx]) + wco[idx]; }
      if (line[char_counter] != ';') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
      if (line[1] == 'F') {
        if (pl_data.condition & PL_COND_FLAG_RAPID_MOTION) {
          protocol_buffer_synchronize();
          if (sys.abort) { return(0); }
          sys.state = STATE_HOLD;
          sys.non_modal_dwell = max(1, ceil(1.0 / DWELL_TIME_STEP * pl_data.dwell));
          spindle_set_speed(pl_data.spindle_speed);
          while (sys.non_modal_dwell > 0) {
            buttons_check();
            protocol_exec_rt_system();
            if (sys.abort) { return(0); }
            _delay_ms(DWELL_TIME_STEP);
            sys.non_modal_dwell--; } }
        pl_data.condition = 0;
      } else {
        pl_data.condition = PL_COND_FLAG_RAPID_MOTION; }
      break;

    case 'P':
      if (line[1] != 'D' && line[1] != 'U') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
      for (idx=0; idx<N_AXIS; idx++) {
        char_counter++;
        if (!read_int(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
        pl_data.xyz[idx] = value + wco[idx]; }
      if (line[char_counter] != ';') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
      if (line[1] == 'D') {
        if (pl_data.condition & PL_COND_FLAG_RAPID_MOTION) {
          protocol_buffer_synchronize();
          if (sys.abort) { return(0); }
          sys.state = STATE_HOLD;
          sys.non_modal_dwell = max(1, ceil(1.0 / DWELL_TIME_STEP * pl_data.dwell));
          spindle_set_speed(pl_data.spindle_speed);
          while (sys.non_modal_dwell > 0) {
            buttons_check();
            protocol_exec_rt_system();
            if (sys.abort) { return(0); }
            _delay_ms(DWELL_TIME_STEP);
            sys.non_modal_dwell--; } }
        pl_data.condition = 0;
      } else {
        pl_data.condition = PL_COND_FLAG_RAPID_MOTION; }
      break;

    case 'D':
      if (line[1] != 'W') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
      char_counter = 2;
      if (!read_int(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
      if (line[char_counter] == ',') {
        char_counter++;
        if (!read_int(line, &char_counter, &pl_data.spindle_speed)) { return(STATUS_BAD_NUMBER_FORMAT); } }
      if (line[char_counter] != ';') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
      protocol_buffer_synchronize();
      if (sys.abort) { return(0); }
      sys.state = STATE_HOLD;
      sys.non_modal_dwell = max(1, ceil(1.0 / DWELL_TIME_STEP * value));
      spindle_set_speed(pl_data.spindle_speed);
      while (sys.non_modal_dwell > 0) {
        buttons_check();
        protocol_exec_rt_system();
        if (sys.abort) { return(0); }
        _delay_ms(DWELL_TIME_STEP);
        sys.non_modal_dwell--; }
      spindle_set_speed(SPINDLE_PWM_OFF_VALUE);
      return(STATUS_OK);

    case 'M':
      if (line[1] != '0' || line[2] != ';') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
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

    case 'N':
      if (line[1] != 'I' || line[2] != ';') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
      protocol_buffer_synchronize();
      if (sys.abort) { return(0); }
      spindle_set_speed(SPINDLE_PWM_OFF_VALUE);
      lcd_state2();
      return(STATUS_OK);

    case 'I':
      if (line[1] != 'N') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
      pl_data.condition = PL_COND_FLAG_RAPID_MOTION;
      if (line[2] == ';') {
        for (idx=0; idx<N_AXIS; idx++) {
          pl_data.xyz[idx] = wco[idx]; }
      } else {
        char_counter = 2;
        if (!read_float(line, &char_counter, &pl_data.feed_rate)) { return(STATUS_BAD_NUMBER_FORMAT); }
        char_counter++;
        if (!read_int(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
        if ((sys.f_override = value) < 50) { sys.f_override = 50; }
        char_counter++;
        if (!read_int(line, &char_counter, &pl_data.spindle_speed)) { return(STATUS_BAD_NUMBER_FORMAT); }
        char_counter++;
        if (!read_int(line, &char_counter, &pl_data.dwell)) { return(STATUS_BAD_NUMBER_FORMAT); }
        if (line[char_counter] != ';') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
        for (idx=0; idx<N_AXIS; idx++) {
          pl_data.xyz[idx] = plan_get_position(idx); }
        pl_data.laserOn = 1;
        spindle_set_speed(SPINDLE_PWM_OFF_VALUE);
        lcd_state2();
        return(STATUS_OK); }
      break;

    case 'C':
      if (line[1] != 'L') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
      if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) {
        char_counter = 2;
        if (read_int(line, &char_counter, &value)) {
          if ((value + wco[X_AXIS]) / settings.steps_per_mm[X_AXIS] < -0.5) { return(STATUS_SOFT_LIMIT_ERROR); }
          char_counter++;
          if (!read_int(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if ((value + wco[Y_AXIS]) / settings.steps_per_mm[Y_AXIS] < -0.5) { return(STATUS_SOFT_LIMIT_ERROR); }
          for (idx=0; idx<N_AXIS; idx++) {
            char_counter++;
            if (!read_int(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
            if ((value + wco[idx]) / settings.steps_per_mm[idx] > settings.max_travel[idx] + 0.5) { return(STATUS_SOFT_LIMIT_ERROR); } }
        } else {
          char_counter = 1;
          for (idx=0; idx<N_AXIS; idx++) {
            char_counter++;
            if (!read_float(line, &char_counter, &f_val)) { return(STATUS_BAD_NUMBER_FORMAT); }
            if (f_val + wco[idx]/settings.steps_per_mm[idx] < -0.5) { return(STATUS_SOFT_LIMIT_ERROR); } }
          for (idx=0; idx<N_AXIS; idx++) {
            char_counter++;
            if (!read_float(line, &char_counter, &f_val)) { return(STATUS_BAD_NUMBER_FORMAT); }
            if (f_val + wco[idx]/settings.steps_per_mm[idx] > settings.max_travel[idx] + 0.5) { return(STATUS_SOFT_LIMIT_ERROR); } } }
      } else {
        for (idx=0; idx<N_AXIS*2; idx++) {
          char_counter++;
          if (!read_float(line, &char_counter, &f_val)) { return(STATUS_BAD_NUMBER_FORMAT); } } }
      if (line[char_counter] != ';') { return(STATUS_GCODE_UNSUPPORTED_COMMAND); }
      return(STATUS_OK);

    default:
      return(STATUS_GCODE_UNSUPPORTED_COMMAND); }

  while (plan_check_full_buffer()) {
    protocol_execute_realtime();
    if (sys.abort) { return(0); } }

  plan_buffer_line();

  protocol_auto_cycle_start();
  protocol_execute_realtime();
  if (sys.abort) { return(0); }

  return(STATUS_OK); }

uint8_t gc_execute_line2(char *line) {
  uint8_t char_counter = 0;
  char letter;
  uint8_t dwell = 0;
  uint16_t P = 0;
  float f_val;
  uint8_t int_value;
  while (line[char_counter] != 0) {
    if ((letter = line[char_counter++]) == '(') {
      break; }
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
        break; } }

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
