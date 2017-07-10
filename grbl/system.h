#ifndef system_h
#define system_h

#include "grbl.h"

#define EXEC_STATUS_REPORT  bit(0)
#define EXEC_CYCLE_START    bit(1)
#define EXEC_CYCLE_STOP     bit(2)
#define EXEC_FEED_HOLD      bit(3)
#define EXEC_RESET          bit(4)

#define STATE_IDLE          0
#define STATE_ALARM         bit(0)
#define STATE_NULL          bit(1)
#define STATE_HOMING        bit(2)
#define STATE_CYCLE         bit(3)
#define STATE_HOLD          bit(4)
#define STATE_JOG           bit(5)

#define STATE_UART          0
#define STATE_FILE          1
#define STATE_ORIGIN        2

#define STEP_CONTROL_NORMAL_OP            0
#define STEP_CONTROL_END_MOTION           bit(0)
#define STEP_CONTROL_EXECUTE_HOLD         bit(1)
#define STEP_CONTROL_EXECUTE_SYS_MOTION   bit(2)
#define STEP_CONTROL_UPDATE_SPINDLE_PWM   bit(3)

typedef struct {
  uint8_t state;
  uint8_t last_state;
  uint8_t state2;
  uint8_t abort;
  uint8_t suspend;
  uint8_t step_control;
  uint8_t f_override;
  uint8_t spindle_speed;
  uint8_t non_modal_dwell;
} system_t;
extern system_t sys;

extern volatile uint8_t btn_state;
extern volatile uint8_t btn;

extern int32_t sys_position[N_AXIS];
extern float last_position[N_AXIS];
extern int32_t wco[N_AXIS];

extern volatile uint8_t sys_rt_exec_state;

void system_init();
void buttons_check();
uint8_t adc_read(uint8_t pin);
uint8_t system_execute_line(char *line);
void system_set_exec_state_flag(uint8_t mask);
void system_clear_exec_state_flag(uint8_t mask);

#endif
