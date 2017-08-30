#include "grbl.h"

#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0))
#define REQ_MM_INCREMENT_SCALAR 1.25
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2
#define RAMP_DECEL_OVERRIDE 3

#define PREP_FLAG_RECALCULATE bit(0)
#define PREP_FLAG_HOLD_PARTIAL_BLOCK bit(1)
#define PREP_FLAG_DECEL_OVERRIDE bit(2)

#define MAX_AMASS_LEVEL 3
#define AMASS_LEVEL1 (F_CPU/8000)
#define AMASS_LEVEL2 (F_CPU/4000)
#define AMASS_LEVEL3 (F_CPU/2000)

typedef struct {
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
  uint8_t direction_bits;
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

typedef struct {
  uint16_t n_step;
  uint16_t cycles_per_tick;
  uint8_t  st_block_index;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;
  #else
    uint8_t prescaler;
  #endif
  uint8_t spindle_pwm;
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

typedef struct {
  uint32_t counter_x, counter_y;
  uint8_t execute_step;
  uint8_t step_outbits;
  uint8_t dir_outbits;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif
  uint16_t step_count;
  uint8_t exec_block_index;
  st_block_t *exec_block;
  segment_t *exec_segment;
} stepper_t;
static stepper_t st;

static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

static uint8_t step_port_invert_mask;
static uint8_t dir_port_invert_mask;

static volatile uint8_t busy;

static plan_block_t *pl_block;
static st_block_t *st_prep_block;

typedef struct {
  uint8_t st_block_index;
  uint8_t recalculate_flag;
  float dt_remainder;
  float steps_remaining;
  float step_per_mm;
  float req_mm_increment;
  uint8_t ramp_type;
  float mm_complete;
  float current_speed;
  float maximum_speed;
  float exit_speed;
  float accelerate_until;
  float decelerate_after;
  uint8_t current_spindle_pwm;
} st_prep_t;
static st_prep_t prep;

void st_wake_up() {
  if (bit_istrue(settings.flags,BITFLAG_XY_HOME_PIN_AS_ST_ENABLE)) {
    if ((bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS) ? bit_isfalse(STEPPERS_DISABLE_PORT,(1<<STEPPERS_DISABLE_BIT)):bit_istrue(STEPPERS_DISABLE_PORT,(1<<STEPPERS_DISABLE_BIT)))) {
      if (bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); }
      else { STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); }
      delay_ms(200); } }
  st.step_outbits = step_port_invert_mask;

  TIMSK1 |= (1<<OCIE1A); }

void st_go_idle() {
  TIMSK1 &= ~(1<<OCIE1A);
  TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10);

  busy = 0; }

ISR(TIMER1_COMPA_vect) {
  if (busy) { return; }

  STEPPER_PORT = (STEPPER_PORT & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK);
  STEPPER_PORT = (STEPPER_PORT & ~STEP_MASK) | st.step_outbits;

  TCNT0 = -((10*TICKS_PER_MICROSECOND) >> 3);
  TCCR0B = (1<<CS01);

  busy = 1;
  sei();

  if (st.exec_segment == NULL) {
    if (segment_buffer_head != segment_buffer_tail) {
      st.exec_segment = &segment_buffer[segment_buffer_tail];

      #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (st.exec_segment->prescaler<<CS10);
      #endif

      OCR1A = st.exec_segment->cycles_per_tick;
      st.step_count = st.exec_segment->n_step;
      if (st.exec_block_index != st.exec_segment->st_block_index) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];

        st.counter_x = st.counter_y = (st.exec_block->step_event_count >> 1); }
      st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask;

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
        st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
      #endif

      if (sys.state == STATE_CYCLE && sys.spindle_speed != SPINDLE_PWM_MIN_VALUE) {
        spindle_set_speed(st.exec_segment->spindle_pwm); }
    } else {
      TIMSK1 &= ~(1<<OCIE1A);
      TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10);

      if (sys.spindle_speed > SPINDLE_PWM_MIN_VALUE) { spindle_set_speed(SPINDLE_PWM_OFF_VALUE); }
      system_set_exec_state_flag(EXEC_CYCLE_STOP);
      busy = 0;
      return; } }

  st.step_outbits = 0;

  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_x += st.steps[X_AXIS];
  #else
    st.counter_x += st.exec_block->steps[X_AXIS];
  #endif
  if (st.counter_x > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<X_STEP_BIT);
    st.counter_x -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<X_DIRECTION_BIT)) { sys_position[X_AXIS]--; }
    else { sys_position[X_AXIS]++; } }

  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_y += st.steps[Y_AXIS];
  #else
    st.counter_y += st.exec_block->steps[Y_AXIS];
  #endif
  if (st.counter_y > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Y_STEP_BIT);
    st.counter_y -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Y_DIRECTION_BIT)) { sys_position[Y_AXIS]--; }
    else { sys_position[Y_AXIS]++; } }

  st.step_count--;
  if (st.step_count == 0) {
    st.exec_segment = NULL;
    if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; } }

  st.step_outbits ^= step_port_invert_mask;
  busy = 0; }

ISR(TIMER0_OVF_vect) {
  STEPPER_PORT = (STEPPER_PORT & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK);
  TCCR0B = 0; }

void st_generate_step_dir_invert_masks() {
  uint8_t idx;
  step_port_invert_mask = 0;
  dir_port_invert_mask = 0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (bit_istrue(settings.step_invert_mask,bit(idx))) { step_port_invert_mask |= get_step_pin_mask(idx); }
    if (bit_istrue(settings.dir_invert_mask,bit(idx))) { dir_port_invert_mask |= get_direction_pin_mask(idx); } } }

void st_reset() {
  st_go_idle();

  memset(&prep, 0, sizeof(st_prep_t));
  memset(&st, 0, sizeof(stepper_t));
  st.exec_segment = NULL;
  pl_block = NULL;
  segment_buffer_tail = 0;
  segment_buffer_head = 0;
  segment_next_head = 1;
  busy = 0;

  st_generate_step_dir_invert_masks();
  st.dir_outbits = dir_port_invert_mask;

  STEPPER_PORT = (STEPPER_PORT & ~STEP_MASK) | step_port_invert_mask;
  STEPPER_PORT = (STEPPER_PORT & ~DIRECTION_MASK) | dir_port_invert_mask; }

void stepper_init() {
  STEPPER_DDR |= STEP_MASK | DIRECTION_MASK;

  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~((1<<WGM11) | (1<<WGM10));
  TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0));
  TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10);
  TIMSK1 &= ~(1<<OCIE1A);

  TIMSK0 &= ~((1<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0));
  TCCR0A = 0;
  TCCR0B = 0;
  TIMSK0 |= (1<<TOIE0); }

void st_update_plan_block_parameters() {
  if (pl_block != NULL) {
    prep.recalculate_flag |= PREP_FLAG_RECALCULATE;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed;
    pl_block = NULL; } }

static uint8_t st_next_block_index(uint8_t block_index) {
  block_index++;
  if (block_index == (SEGMENT_BUFFER_SIZE-1)) { return(0); }
  return(block_index); }

void st_prep_buffer() {
  if (bit_istrue(sys.step_control,STEP_CONTROL_END_MOTION)) { return; }

  while (segment_buffer_tail != segment_next_head) {

    if (pl_block == NULL) {

      pl_block = plan_get_current_block();
      if (pl_block == NULL) { return; }

      if (prep.recalculate_flag & PREP_FLAG_RECALCULATE) {

        prep.recalculate_flag = 0;

      } else {

        prep.st_block_index = st_next_block_index(prep.st_block_index);

        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;
        uint8_t idx;
        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = (pl_block->steps[idx] << 1); }
          st_prep_block->step_event_count = (pl_block->step_event_count << 1);
        #else
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL; }
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif

        prep.steps_remaining = (float)pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;
        prep.dt_remainder = 0.0;

        if ((sys.step_control & STEP_CONTROL_EXECUTE_HOLD) || (prep.recalculate_flag & PREP_FLAG_DECEL_OVERRIDE)) {
          prep.current_speed = prep.exit_speed;
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed;
          prep.recalculate_flag &= ~(PREP_FLAG_DECEL_OVERRIDE);
        } else {
          prep.current_speed = sqrt(pl_block->entry_speed_sqr); } }

      prep.mm_complete = 0;
      float inv_2_accel = .5/pl_block->acceleration;

      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {

        prep.ramp_type = RAMP_DECEL;

        float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
        if (decel_dist < 0) {
          prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
        } else {
          prep.mm_complete = decel_dist;
          prep.exit_speed = 0; }

      } else {

        prep.ramp_type = RAMP_ACCEL;
        prep.accelerate_until = pl_block->millimeters;

        float exit_speed_sqr;
        float nominal_speed;
        exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
        prep.exit_speed = sqrt(exit_speed_sqr);

        nominal_speed = plan_compute_profile_nominal_speed(pl_block);
        float nominal_speed_sqr = nominal_speed*nominal_speed;
        float intersect_distance = .5*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));
        if (pl_block->entry_speed_sqr > nominal_speed_sqr) {

          prep.accelerate_until = pl_block->millimeters - inv_2_accel*(pl_block->entry_speed_sqr-nominal_speed_sqr);
          if (prep.accelerate_until <= 0) {

            prep.ramp_type = RAMP_DECEL;
            //prep.decelerate_after = pl_block->millimeters;
            //prep.maximum_speed = prep.current_speed;

            prep.exit_speed = sqrt(pl_block->entry_speed_sqr - 2*pl_block->acceleration*pl_block->millimeters);
            prep.recalculate_flag |= PREP_FLAG_DECEL_OVERRIDE;

          } else {

            prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
            prep.maximum_speed = nominal_speed;
            prep.ramp_type = RAMP_DECEL_OVERRIDE; }

        } else if (intersect_distance > 0) {

          if (intersect_distance < pl_block->millimeters) {
            prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
            if (prep.decelerate_after < intersect_distance) {
              prep.maximum_speed = nominal_speed;
              if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
                prep.ramp_type = RAMP_CRUISE;
              } else {
                prep.accelerate_until -= inv_2_accel*(nominal_speed_sqr-pl_block->entry_speed_sqr); }
            } else {
              prep.accelerate_until = intersect_distance;
              prep.decelerate_after = intersect_distance;
              prep.maximum_speed = sqrt(2*pl_block->acceleration*intersect_distance+exit_speed_sqr); }
          } else {
            prep.ramp_type = RAMP_DECEL;
            //prep.decelerate_after = pl_block->millimeters;
            //prep.maximum_speed = prep.current_speed;
          }

        } else {

          prep.accelerate_until = 0;
          //prep.decelerate_after = 0;
          prep.maximum_speed = prep.exit_speed; } }

      bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
    }

    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    prep_segment->st_block_index = prep.st_block_index;

    float dt_max = DT_SEGMENT;
    float dt = 0;
    float time_var = dt_max;
    float mm_var;
    float speed_var;
    float mm_remaining = pl_block->millimeters;
    float minimum_mm = mm_remaining-prep.req_mm_increment;
    if (minimum_mm < 0) { minimum_mm = 0; }

    do {
      switch (prep.ramp_type) {
        case RAMP_DECEL_OVERRIDE:
          speed_var = pl_block->acceleration*time_var;
          if (prep.current_speed-prep.maximum_speed <= speed_var) {
            mm_remaining = prep.accelerate_until;
            time_var = 2*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            prep.ramp_type = RAMP_CRUISE;
            prep.current_speed = prep.maximum_speed;
          } else {
            mm_remaining -= time_var*(prep.current_speed - .5*speed_var);
            prep.current_speed -= speed_var; }
          break;
        case RAMP_ACCEL:
          speed_var = pl_block->acceleration*time_var;
          mm_remaining -= time_var*(prep.current_speed + .5*speed_var);
          if (mm_remaining < prep.accelerate_until) {
            mm_remaining = prep.accelerate_until;
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else {
            prep.current_speed += speed_var; }
          break;
        case RAMP_CRUISE:
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) {
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after;
            prep.ramp_type = RAMP_DECEL;
          } else {
            mm_remaining = mm_var; }
          break;
        default:
          speed_var = pl_block->acceleration*time_var;
          if (prep.current_speed > speed_var) {
            mm_var = mm_remaining - time_var*(prep.current_speed - .5*speed_var);
            if (mm_var > prep.mm_complete) {
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; } }
          time_var = 2*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete;
          prep.current_speed = prep.exit_speed; }
      dt += time_var;
      if (dt < dt_max) { time_var = dt_max - dt; }
      else {
        if (mm_remaining > minimum_mm) {
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else {
          break; } }
    } while (mm_remaining > prep.mm_complete);

    if (sys.step_control & STEP_CONTROL_UPDATE_SPINDLE_PWM) {
      prep.current_spindle_pwm = pl_block->spindle_speed;
      bit_false(sys.step_control,STEP_CONTROL_UPDATE_SPINDLE_PWM); }
    prep_segment->spindle_pwm = prep.current_spindle_pwm;

    float step_dist_remaining = prep.step_per_mm*mm_remaining;
    float n_steps_remaining = ceil(step_dist_remaining);
    float last_n_steps_remaining = ceil(prep.steps_remaining);
    prep_segment->n_step = last_n_steps_remaining-n_steps_remaining;

    if (prep_segment->n_step == 0) {
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        return; } }

    dt += prep.dt_remainder;
    float inv_rate = dt/(last_n_steps_remaining - step_dist_remaining);
    uint32_t cycles = ceil((TICKS_PER_MICROSECOND*1000000*60)*inv_rate);

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      if (cycles < AMASS_LEVEL1) { prep_segment->amass_level = 0; }
      else {
        if (cycles < AMASS_LEVEL2) { prep_segment->amass_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->amass_level = 2; }
        else { prep_segment->amass_level = 3; }
        cycles >>= prep_segment->amass_level;
        prep_segment->n_step <<= prep_segment->amass_level; }
      if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; }
      else { prep_segment->cycles_per_tick = 0xffff; }
    #else
      if (cycles < (1UL << 16)) {
        prep_segment->prescaler = 1;
        prep_segment->cycles_per_tick = cycles;
      } else if (cycles < (1UL << 19)) {
        prep_segment->prescaler = 2;
        prep_segment->cycles_per_tick = cycles >> 3;
      } else {
        prep_segment->prescaler = 3;
        if (cycles < (1UL << 22)) {
          prep_segment->cycles_per_tick =  cycles >> 6;
        } else {
          prep_segment->cycles_per_tick = 0xffff; } }
    #endif

    segment_buffer_head = segment_next_head;
    if (++segment_next_head == SEGMENT_BUFFER_SIZE) { segment_next_head = 0; }

    pl_block->millimeters = mm_remaining;
    prep.steps_remaining = n_steps_remaining;
    prep.dt_remainder = (n_steps_remaining - step_dist_remaining)*inv_rate;

    if (mm_remaining == prep.mm_complete) {
      if (mm_remaining > 0) {
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        return;
      } else {
        pl_block = NULL;
        plan_discard_current_block(); } } } }
