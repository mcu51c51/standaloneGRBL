#ifndef nuts_bolts_h
#define nuts_bolts_h

#define SOME_LARGE_VALUE 1.0E+38

#define N_AXIS 2
#define X_AXIS 0
#define Y_AXIS 1

#define A_MOTOR X_AXIS
#define B_MOTOR Y_AXIS

#define DWELL_TIME_STEP 10
#define MM_PER_INCH (25.4)
#define TICKS_PER_MICROSECOND (F_CPU/1000000)

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

#define bit(n) (1 << n)
#define bit_true(x,mask) (x) |= (mask)
#define bit_false(x,mask) (x) &= ~(mask)
#define bit_istrue(x,mask) (0 != (x & mask))
#define bit_isfalse(x,mask) (0 == (x & mask))

uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);
void delay_ms(uint16_t ms);
float convert_delta_vector_to_unit_vector(float *vector);
float limit_value_by_axis_maximum(float *max_value, float *unit_vec);

#endif
