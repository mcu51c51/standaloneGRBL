#ifndef cpu_map_h
#define cpu_map_h

#define SERIAL_RX         USART_RX_vect
#define SERIAL_UDRE       USART_UDRE_vect

#define STEPPER_DDR       DDRD
#define STEPPER_PORT      PORTD
#define X_STEP_BIT        5
#define X_DIRECTION_BIT   4
#define Y_STEP_BIT        7
#define Y_DIRECTION_BIT   6
#define STEP_MASK         ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT))
#define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT))

#define LCD_RS_PORT      PORTD
#define LCD_RS_BIT       2

#define LCD_RW_PORT      PORTB
#define LCD_RW_BIT       0

#define LCD_ENABLE_PORT  PORTC
#define LCD_ENABLE_BIT   4

#define LCD_DATA_PORT    PORTC

#define LIMIT_DDR         DDRB
#define LIMIT_PIN         PINB
#define LIMIT_PORT        PORTB
#define XY_LIMIT_BIT      1

#define CONTROL_DDR       DDRC
#define CONTROL_PORT      PORTC
#define CONTROL_OTHER     5
#define CONTROL_JOG_X     6
#define CONTROL_JOG_Y     7
#define CONTROL_MASK      ((1<<CONTROL_OTHER)|(1<<CONTROL_JOG_X)|(1<<CONTROL_JOG_Y))

#define SPINDLE_PWM_DDR   DDRD
#define SPINDLE_PWM_BIT   3

#define SPINDLE_PWM_MAX_VALUE     255
#define SPINDLE_PWM_MIN_VALUE     1
#define SPINDLE_PWM_OFF_VALUE     0
#define SPINDLE_TCCRA_REGISTER    TCCR2A
#define SPINDLE_TCCRB_REGISTER    TCCR2B
#define SPINDLE_OCR_REGISTER      OCR2B
#define SPINDLE_COMB_BIT          COM2B1
#define SPINDLE_TCCRA_INIT_MASK   ((1<<WGM20)|(1<<WGM21))

// 62.5kHz
//#define SPINDLE_TCCRB_INIT_MASK   (1<<CS20)

// 7.8kHz
//#define SPINDLE_TCCRB_INIT_MASK   (1<<CS21)

// 1.96kHz
//#define SPINDLE_TCCRB_INIT_MASK   ((1<<CS21)|(1<<CS20))

// 0.98kHz
#define SPINDLE_TCCRB_INIT_MASK   (1<<CS22)

#endif
