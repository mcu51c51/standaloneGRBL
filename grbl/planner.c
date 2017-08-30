#include "grbl.h"

#define PLAN_OK 1
#define PLAN_EMPTY_BLOCK 0

static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];
static uint8_t block_buffer_tail;
static uint8_t block_buffer_head;
static uint8_t next_buffer_head;
static uint8_t block_buffer_planned;

typedef struct {
  int32_t position[N_AXIS];
  float previous_unit_vec[N_AXIS];
  float previous_nominal_speed;
} planner_t;
static planner_t pl;

uint8_t plan_next_block_index(uint8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index); }

static uint8_t plan_prev_block_index(uint8_t block_index) {
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index); }

static void planner_recalculate() {
  uint8_t block_index = plan_prev_block_index(block_buffer_head);

  if (block_index == block_buffer_planned) { return; }

  float entry_speed_sqr;
  plan_block_t *next;
  plan_block_t *current = &block_buffer[block_index];

  current->entry_speed_sqr = min(current->max_entry_speed_sqr, 2*current->acceleration*current->millimeters);

  block_index = plan_prev_block_index(block_index);
  if (block_index == block_buffer_planned) {
    if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }
  } else {
    while (block_index != block_buffer_planned) {
      next = current;
      current = &block_buffer[block_index];
      block_index = plan_prev_block_index(block_index);

      if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }

      if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
        entry_speed_sqr = next->entry_speed_sqr + 2*current->acceleration*current->millimeters;
        if (entry_speed_sqr < current->max_entry_speed_sqr) {
          current->entry_speed_sqr = entry_speed_sqr;
        } else {
          current->entry_speed_sqr = current->max_entry_speed_sqr; } } } }

  next = &block_buffer[block_buffer_planned];
  block_index = plan_next_block_index(block_buffer_planned);
  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];

    if (current->entry_speed_sqr < next->entry_speed_sqr) {
      entry_speed_sqr = current->entry_speed_sqr + 2*current->acceleration*current->millimeters;
      if (entry_speed_sqr < next->entry_speed_sqr) {
        next->entry_speed_sqr = entry_speed_sqr;
        block_buffer_planned = block_index; } }

    if (next->entry_speed_sqr == next->max_entry_speed_sqr) { block_buffer_planned = block_index; }
    block_index = plan_next_block_index(block_index); } }

void plan_reset() {
  memset(&pl, 0, sizeof(planner_t));
  block_buffer_tail = 0;
  block_buffer_head = 0;
  next_buffer_head = 1;
  block_buffer_planned = 0; }

void plan_discard_current_block() {
  if (block_buffer_head != block_buffer_tail) {
    uint8_t block_index = plan_next_block_index(block_buffer_tail);
    if (block_buffer_tail == block_buffer_planned) { block_buffer_planned = block_index; }
    block_buffer_tail = block_index; } }

plan_block_t *plan_get_current_block() {
  if (block_buffer_head == block_buffer_tail) { return(NULL); }
  return(&block_buffer[block_buffer_tail]); }

float plan_get_exec_block_exit_speed_sqr() {
  uint8_t block_index = plan_next_block_index(block_buffer_tail);
  if (block_index == block_buffer_head) { return(0); }
  return(block_buffer[block_index].entry_speed_sqr); }

uint8_t plan_check_full_buffer() {
  if (block_buffer_tail == next_buffer_head) { return(1); }
  return(0); }

float plan_compute_profile_nominal_speed(plan_block_t *block) {
  float nominal_speed = block->programmed_rate;
  if (!(block->condition & PL_COND_FLAG_RAPID_MOTION)) {
    nominal_speed *= (.01*sys.f_override);
    if (nominal_speed > block->rapid_rate) { nominal_speed = block->rapid_rate; } }
  if (nominal_speed > MINIMUM_FEED_RATE) { return(nominal_speed); }
  return(MINIMUM_FEED_RATE); }

static void plan_compute_profile_parameters(plan_block_t *block, float nominal_speed, float prev_nominal_speed) {
  if (nominal_speed > prev_nominal_speed) {
    block->max_entry_speed_sqr = prev_nominal_speed*prev_nominal_speed;
  } else {
    block->max_entry_speed_sqr = nominal_speed*nominal_speed; }
  if (block->max_entry_speed_sqr > block->max_junction_speed_sqr) {
    block->max_entry_speed_sqr = block->max_junction_speed_sqr; } }

void plan_update_velocity_profile_parameters() {
  uint8_t block_index = block_buffer_tail;
  plan_block_t *block;
  float nominal_speed;
  float prev_nominal_speed = SOME_LARGE_VALUE;
  while (block_index != block_buffer_head) {
    block = &block_buffer[block_index];
    nominal_speed = plan_compute_profile_nominal_speed(block);
    plan_compute_profile_parameters(block, nominal_speed, prev_nominal_speed);
    prev_nominal_speed = nominal_speed;
    block_index = plan_next_block_index(block_index); }
  pl.previous_nominal_speed = prev_nominal_speed; }

uint8_t plan_buffer_line() {
  plan_block_t *block = &block_buffer[block_buffer_head];
  memset(block, 0, sizeof(plan_block_t));
  block->condition = pl_data.condition;

  int32_t position_steps[N_AXIS];
  float unit_vec[N_AXIS], delta_mm;
  uint8_t idx;

  memcpy(position_steps, pl.position, sizeof(pl.position));

  #ifdef COREXY
    block->steps[A_MOTOR] = labs((pl_data.xyz[X_AXIS]-position_steps[X_AXIS]) + (pl_data.xyz[Y_AXIS]-position_steps[Y_AXIS]));
    block->steps[B_MOTOR] = labs((pl_data.xyz[X_AXIS]-position_steps[X_AXIS]) - (pl_data.xyz[Y_AXIS]-position_steps[Y_AXIS]));
  #endif

  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef COREXY
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      if (idx == A_MOTOR) {
        delta_mm = (pl_data.xyz[X_AXIS]-position_steps[X_AXIS] + pl_data.xyz[Y_AXIS]-position_steps[Y_AXIS])/settings.steps_per_mm[idx];
      } else {
        delta_mm = (pl_data.xyz[X_AXIS]-position_steps[X_AXIS] - pl_data.xyz[Y_AXIS]+position_steps[Y_AXIS])/settings.steps_per_mm[idx]; }
    #else
      block->steps[idx] = labs(pl_data.xyz[idx]-position_steps[idx]);
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      delta_mm = (pl_data.xyz[idx] - position_steps[idx])/settings.steps_per_mm[idx];
    #endif
    unit_vec[idx] = delta_mm;

    if (delta_mm < 0.0 ) { block->direction_bits |= get_direction_pin_mask(idx); } }

  if (block->step_event_count == 0) { return(PLAN_EMPTY_BLOCK); }

  block->millimeters = convert_delta_vector_to_unit_vector(unit_vec);
  block->acceleration = limit_value_by_axis_maximum(settings.acceleration, unit_vec);
  block->rapid_rate = limit_value_by_axis_maximum(settings.max_rate, unit_vec);

  if (block->condition & PL_COND_FLAG_RAPID_MOTION) {
    block->programmed_rate = block->rapid_rate;
  } else {
    block->programmed_rate = pl_data.feed_rate;
    block->spindle_speed = pl_data.spindle_speed;
  }

  if (block_buffer_head == block_buffer_tail) {

    block->entry_speed_sqr = 0;
    block->max_junction_speed_sqr = 0;

  } else {

    float junction_unit_vec[N_AXIS];
    float junction_cos_theta = 0;
    for (idx=0; idx<N_AXIS; idx++) {
      junction_cos_theta -= pl.previous_unit_vec[idx]*unit_vec[idx];
      junction_unit_vec[idx] = unit_vec[idx]-pl.previous_unit_vec[idx]; }

    if (junction_cos_theta > .999999) {
      block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED;
    } else {
      if (junction_cos_theta < -.999999) {
        block->max_junction_speed_sqr = SOME_LARGE_VALUE;
      } else {
        convert_delta_vector_to_unit_vector(junction_unit_vec);
        float junction_acceleration = limit_value_by_axis_maximum(settings.acceleration, junction_unit_vec);
        float sin_theta_d2 = sqrt(.5*(1-junction_cos_theta));
        block->max_junction_speed_sqr = max(MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED, (junction_acceleration * settings.junction_deviation * sin_theta_d2)/(1.0-sin_theta_d2)); } } }

  float nominal_speed = plan_compute_profile_nominal_speed(block);
  plan_compute_profile_parameters(block, nominal_speed, pl.previous_nominal_speed);
  pl.previous_nominal_speed = nominal_speed;

  memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec));
  memcpy(pl.position, pl_data.xyz, sizeof(pl_data.xyz));

  block_buffer_head = next_buffer_head;
  next_buffer_head = plan_next_block_index(block_buffer_head);

  planner_recalculate();

  return(PLAN_OK); }

void plan_sync_position() {
  #ifdef COREXY
    pl.position[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys_position);
    pl.position[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys_position);
  #else
    memcpy(pl.position, sys_position, sizeof(pl.position));
  #endif
}

int32_t plan_get_position(uint8_t idx) {
  return(pl.position[idx]); }

uint8_t plan_get_block_buffer_count() {
  if (block_buffer_head >= block_buffer_tail) { return(block_buffer_head-block_buffer_tail); }
  return(BLOCK_BUFFER_SIZE - (block_buffer_tail-block_buffer_head)); }

void plan_cycle_reinitialize() {
  st_update_plan_block_parameters();
  block_buffer_planned = block_buffer_tail;
  planner_recalculate(); }
