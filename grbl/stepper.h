#ifndef stepper_h
#define stepper_h

#define SEGMENT_BUFFER_SIZE 6

#define ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
#define ACCELERATION_TICKS_PER_SECOND 100

void stepper_init();
void st_wake_up();
void st_go_idle();
void st_generate_step_dir_invert_masks();
void st_reset();
void st_prep_buffer();
void st_update_plan_block_parameters();

#endif
