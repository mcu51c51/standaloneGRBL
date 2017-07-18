#ifndef planner_h
#define planner_h

#define PLAN_OK 1
#define PLAN_EMPTY_BLOCK 0

#define PL_COND_FLAG_RAPID_MOTION      bit(0)
#define PL_COND_FLAG_SYSTEM_MOTION     bit(1)
#define PL_COND_FLAG_PLUNGE_MOTION     bit(2)

typedef struct {
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
  uint8_t direction_bits;
  uint8_t condition;
  float entry_speed_sqr;
  float max_entry_speed_sqr;
  float acceleration;
  float millimeters;
  float max_junction_speed_sqr;
  float rapid_rate;
  float programmed_rate;
  uint8_t spindle_speed;
} plan_block_t;

typedef struct {
  int32_t xyz[N_AXIS];
  float gc_pos[N_AXIS];
  uint8_t spindle_speed;
  uint16_t dwell;
  bool laserOn;
  float feed_rate;
  uint8_t condition;
  uint8_t units;
  uint8_t distance;
} plan_line_data_t;
extern plan_line_data_t pl_data;

void plan_reset();
void plan_reset_buffer();
uint8_t plan_buffer_line();
void plan_discard_current_block();

plan_block_t *plan_get_system_motion_block();
plan_block_t *plan_get_current_block();

uint8_t plan_next_block_index(uint8_t block_index);
float plan_get_exec_block_exit_speed_sqr();
float plan_compute_profile_nominal_speed(plan_block_t *block);
void plan_update_velocity_profile_parameters();
void plan_sync_position();
void plan_cycle_reinitialize();
uint8_t plan_get_block_buffer_count();
uint8_t plan_check_full_buffer();
int32_t plan_get_position(uint8_t idx);

#endif
