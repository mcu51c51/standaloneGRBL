#include "grbl.h"

void jog_execute(uint8_t i) {
  sys.f_override = 100;
  pl_data.spindle_speed = SPINDLE_PWM_OFF_VALUE;
  pl_data.condition = 0;
  uint8_t idx;
  uint8_t j = N_AXIS;
  float f_pos, pos;
  for (idx=0; idx<N_AXIS; idx++) {
    if (bit_istrue(btn_state,bit(idx))) {
      f_pos = (plan_get_position(idx) - wco[idx]) / settings.steps_per_mm[idx];
      pos = round(f_pos * 10) / 10 + i/10.0;
      if (pos - f_pos > (0.8/settings.steps_per_mm[idx] + i/10.0)) {
        pos -= i/10.0; }
      pl_data.xyz[idx] = lround(pos*settings.steps_per_mm[idx]) + wco[idx];
      if (bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE)) {
        if (pl_data.xyz[idx] > lround(settings.max_travel[idx] * settings.steps_per_mm[idx])) {
          pl_data.xyz[idx] = plan_get_position(idx);
          j--; } }
    } else if (bit_istrue(btn_state,bit(N_AXIS+idx))) {
      f_pos = (plan_get_position(idx) - wco[idx]) / settings.steps_per_mm[idx];
      pos = round(f_pos * 10) / 10 - i/10.0;
      if (pos - f_pos < -(0.8/settings.steps_per_mm[idx] + i/10.0)) {
        pos += i/10.0; }
      pl_data.xyz[idx] = lround(pos*settings.steps_per_mm[idx]) + wco[idx];
      if (bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE)) {
        if (pl_data.xyz[idx] < 0) {
          pl_data.xyz[idx] = plan_get_position(idx);
          j--; } }
    } else {
      pl_data.xyz[idx] = plan_get_position(idx);
      j--; } }

  pl_data.feed_rate = 3000.0 * sqrt(j);

  if (settings.max_rate[X_AXIS] < 600.0 || settings.max_rate[Y_AXIS] < 600.0) {
    j = 1;
  } else if (settings.max_rate[X_AXIS] < 1200.0 || settings.max_rate[Y_AXIS] < 1200.0) {
    j = 5;
  } else {
    j = 10; }

  while (plan_get_block_buffer_count() > j) {
    buttons_check();
    protocol_exec_rt_system();
    if (sys.abort) { return; } }

  plan_buffer_line();

  if (sys.state == STATE_IDLE) {
    sys_rt_exec_state |= EXEC_STATUS_REPORT;
    sys.step_control = STEP_CONTROL_NORMAL_OP;
    if (plan_get_current_block() != NULL) {
      sys.state = STATE_JOG;
      st_prep_buffer();
      st_wake_up(); } } }
