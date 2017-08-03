#include "grbl.h"

system_t sys;
plan_line_data_t pl_data;
int32_t sys_position[N_AXIS];
float last_position[N_AXIS];
int32_t wco[N_AXIS];
volatile uint8_t sys_rt_exec_state;
volatile uint8_t btn_state;
volatile uint8_t btn;

int main(void) {
  lcd_init();
  system_init();
  serial_init();
  settings_init();
  stepper_init();
  spindle_init();
  limits_init();

  OCR2A = 255;
  TIMSK2 |= (1 << OCIE2A);

  memset(sys_position, 0, sizeof(sys_position));
  memset(last_position, 0, sizeof(last_position));
  #ifndef COREXY
    settings_read_coord_data();
  #else
    settings.flags &= ~BITFLAG_HOMING_ENABLE;
    memset(wco, 0, sizeof(wco));
  #endif
  sei();

  memset(&sys, 0, sizeof(system_t));
  memset(&pl_data, 0, sizeof(plan_line_data_t));
  if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) {
    sys.state = STATE_ALARM; }
  sys.f_override = 100;

  report_status_message(STATUS_RESET);
  for (;;) {
    serial_reset_read_buffer();

    plan_reset();
    st_reset();

    plan_sync_position();

    protocol_main_loop();

    spindle_set_speed(SPINDLE_PWM_OFF_VALUE);
    lcd_state2();
    sys.abort = 0;
    btn_state = 0;
    sys_rt_exec_state = EXEC_STATUS_REPORT; }

  return 0; }
