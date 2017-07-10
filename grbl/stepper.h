#ifndef stepper_h
#define stepper_h

void stepper_init();
void st_wake_up();
void st_go_idle();
void st_generate_step_dir_invert_masks();
void st_reset();
void st_prep_buffer();
void st_update_plan_block_parameters();

#endif
