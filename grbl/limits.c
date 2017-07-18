#include "grbl.h"

void limits_init() {
  LIMIT_DDR &= ~(1<<XY_LIMIT_BIT);
  LIMIT_PORT |= (1<<XY_LIMIT_BIT); }

uint8_t limit_get_state() {
  uint8_t pin = (LIMIT_PIN & (1<<XY_LIMIT_BIT));
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { pin ^= (1<<XY_LIMIT_BIT); }
  if (pin) {
    return(1); }
  return(0); }

uint8_t limits_go_home() {
  uint8_t l = sys.state;
  uint8_t idx, i;

  spindle_set_speed(SPINDLE_PWM_OFF_VALUE);
  lcd_state2();
  sys.f_override = 100;
  pl_data.spindle_speed = SPINDLE_PWM_OFF_VALUE;

  for (idx=0; idx<N_AXIS; idx++) {
    if (limit_get_state()) {
      sys.state = l;
      return(STATUS_HOMING_FAIL_APPROACH); }
    pl_data.condition = 0;
    pl_data.feed_rate = SOME_LARGE_VALUE;
    pl_data.xyz[X_AXIS] = plan_get_position(X_AXIS);
    pl_data.xyz[Y_AXIS] = plan_get_position(Y_AXIS);
    i = lround(settings.steps_per_mm[idx] / 5);
    do {
      if (plan_get_block_buffer_count() < 3) {
        pl_data.xyz[idx] -= i;
        plan_buffer_line();
        if (sys.state == STATE_IDLE || sys.state == STATE_ALARM || sys.state == STATE_NULL) {
          sys.step_control = STEP_CONTROL_NORMAL_OP;
          if (plan_get_current_block() != NULL) {
            sys.state = STATE_HOMING;
            st_prep_buffer();
            st_wake_up(); } } }

      if (limit_get_state()) { i = 0; }

      buttons_check();
      protocol_exec_rt_system();
      if (sys.abort) { i = 0; }
    } while (sys.state == STATE_HOMING);

    if (sys.abort) {
      sys.abort = 0;
      sys.state = l;
      return(0); }
    delay_ms(HOMING_DEBOUNCE_DELAY);

    pl_data.condition = PL_COND_FLAG_SYSTEM_MOTION;
    pl_data.feed_rate = HOMING_FEED_RATE;
    pl_data.xyz[idx] += lround(HOMING_PULLOFF * settings.steps_per_mm[idx]);
    plan_buffer_line();
    sys.step_control = STEP_CONTROL_EXECUTE_SYS_MOTION;
    sys.state = STATE_HOMING;
    lcd_position(); lcd_state();
    st_prep_buffer();
    st_wake_up();
    do {
      if ((i = sys_rt_exec_state) & EXEC_CYCLE_STOP) { break; }
      st_prep_buffer();
    } while (limit_get_state());

    st_reset();
    plan_sync_position();

    if (i & EXEC_CYCLE_STOP) {
      sys.state = STATE_ALARM;
      sys_rt_exec_state &= ~EXEC_CYCLE_STOP;
      return(STATUS_HOMING_FAIL_PULLOFF); }

    pl_data.xyz[idx] = plan_get_position(idx) + lround(settings.steps_per_mm[idx] / 11);
    plan_buffer_line();
    sys.step_control = STEP_CONTROL_EXECUTE_SYS_MOTION;
    sys.state = STATE_HOMING;
    st_prep_buffer();
    st_wake_up();
    do {
      st_prep_buffer();
    } while (!(sys_rt_exec_state & EXEC_CYCLE_STOP));

    st_reset();
    sys_rt_exec_state &= ~EXEC_CYCLE_STOP;
    sys_position[idx] = 0;
    plan_sync_position();

    sys.state = STATE_NULL;
    delay_ms(HOMING_DEBOUNCE_DELAY); }

  sys.step_control = STEP_CONTROL_NORMAL_OP;

  sys.state = STATE_IDLE;
  return(STATUS_OK); }
