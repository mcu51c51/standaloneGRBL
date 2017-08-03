#include "grbl.h"
#include "lcd.h"

#define DDR(x) (*(&x - 1))
#define PIN(x) (*(&x - 2))

#define lcd_e_delay()   _delay_us(LCD_DELAY_ENABLE_PULSE)
#define lcd_e_high()    LCD_ENABLE_PORT |=  _BV(LCD_ENABLE_BIT);
#define lcd_e_low()     LCD_ENABLE_PORT &= ~_BV(LCD_ENABLE_BIT);
#define lcd_e_toggle()  toggle_e()
#define lcd_rw_high()   LCD_RW_PORT |=  _BV(LCD_RW_BIT)
#define lcd_rw_low()    LCD_RW_PORT &= ~_BV(LCD_RW_BIT)
#define lcd_rs_high()   LCD_RS_PORT |=  _BV(LCD_RS_BIT)
#define lcd_rs_low()    LCD_RS_PORT &= ~_BV(LCD_RS_BIT)

#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES 

static void toggle_e(void);

#define delay(us) _delay_us(us)
uint8_t counter = 0;

static void toggle_e(void) {
  lcd_e_high();
  lcd_e_delay();
  lcd_e_low(); }

static void lcd_write(uint8_t data, uint8_t rs) {
  unsigned char dataBits ;

  if (rs) {
     lcd_rs_high();
  } else {
     lcd_rs_low(); }
  lcd_rw_low();

  DDR(LCD_DATA_PORT) |= 0x0F;

  dataBits = LCD_DATA_PORT & 0xF0;
  LCD_DATA_PORT = dataBits |((data>>4)&0x0F);
  lcd_e_toggle();

  LCD_DATA_PORT = dataBits | (data&0x0F);
  lcd_e_toggle();

  LCD_DATA_PORT = dataBits | 0x0F; }

static uint8_t lcd_read() {
  uint8_t data;

  lcd_rs_low();
  lcd_rw_high();

  DDR(LCD_DATA_PORT) &= 0xF0;

  lcd_e_high();
  lcd_e_delay();
  data = PIN(LCD_DATA_PORT) << 4;
  lcd_e_low();

  lcd_e_delay();

  lcd_e_high();
  lcd_e_delay();
  data |= PIN(LCD_DATA_PORT)&0x0F;
  lcd_e_low();
  return data; }

static void lcd_waitbusy() {
  while (lcd_read() & (1<<LCD_BUSY)) {}
  delay(LCD_DELAY_BUSY_FLAG); }

void lcd_command(uint8_t cmd) {
  lcd_waitbusy();
  lcd_write(cmd,0); }

void lcd_data(uint8_t data) {
  lcd_waitbusy();
  lcd_write(data,1); }

void lcd_gotoxy(uint8_t x, uint8_t y) {
  if (y == 0) {
    lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
  } else {
    lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x); } }

void lcd_clrscr(void) {
  lcd_command(1<<LCD_CLR); }

void lcd_putc(char c) {
  lcd_waitbusy();
  lcd_write(c, 1); }

/*void lcd_puts(const char *s) {
  register char c;
  while ((c = *s++)) {
    lcd_putc(c); } }*/

void lcd_puts_p(const char *progmem_s) {
  register char c;
  while ((c = pgm_read_byte(progmem_s++))) {
    lcd_putc(c); } }

void lcd_position() {
  uint8_t idx, i;
  bool isnegative;
  float n; uint32_t a;
  unsigned char buf[8];
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef COREXY
      if (idx == A_MOTOR) {
        n = (system_convert_corexy_to_x_axis_steps(sys_position) - wco[idx]) / settings.steps_per_mm[idx];
      } else {
        n = (system_convert_corexy_to_y_axis_steps(sys_position) - wco[idx]) / settings.steps_per_mm[idx]; }
    #else
      n = (sys_position[idx] - wco[idx]) / settings.steps_per_mm[idx];
    #endif
    if (n != last_position[idx]) {
      last_position[idx] = n;
      isnegative = 0;
      if (n < 0) {
        isnegative = 1;
        n = -n; }
      n *= 1000; n += 0.5;
      i = 0; a = (long)n;
      while(a > 0) {
        buf[i++] = (a % 10) + '0';
        a /= 10; }
      while (i < 3) {
         buf[i++] = '0'; }
      if (i == 3) {
        buf[i++] = '0'; }
      if (isnegative) {
        buf[i++] = '-'; }
      while (i < 8) {
         buf[i++] = ' '; }
      lcd_gotoxy(2,idx);
      for (; i > 0; i--) {
        if (i == 3) { lcd_gotoxy(8,idx); }
        lcd_putc(buf[i-1]); } } } }

void lcd_state() {
  if (sys.state != sys.last_state) {
    sys.last_state = sys.state;
    lcd_gotoxy(12,0);
    switch (sys.state) {
      case STATE_CYCLE: lcd_puts_P(" RUN"); break;
      case STATE_IDLE: lcd_puts_P("IDLE"); break;
      case STATE_HOLD:
        if (sys.non_modal_dwell) {
          lcd_puts_P(" RUN");
        } else {
          lcd_puts_P("PAUS"); }
        break;
      case STATE_JOG: lcd_puts_P("MANL"); break;
      case STATE_HOMING: lcd_puts_P("HOME"); break;
      case STATE_ALARM: lcd_puts_P("ALRM"); break; }
    lcd_state2(); } }

void lcd_state2() {
  lcd_gotoxy(12,1);
  if (sys.state == STATE_CYCLE || sys.state == STATE_HOLD) {
    if (sys.state2 == STATE_UART) {
      lcd_puts_P("UART");
    } else if (sys.state2 == STATE_FILE) {
      lcd_puts_P("FILE");
    } else if (sys.state2 == STATE_ORIGIN) {
      lcd_puts_P("ORGN"); }
  } else {
    if (sys.spindle_speed == SPINDLE_PWM_OFF_VALUE) {
      lcd_puts_P("LOFF");
    } else {
      lcd_puts_P("SAFE"); } } }

void lcd_clear() {
  last_position[X_AXIS] = 0; last_position[Y_AXIS] = 0;
  lcd_gotoxy(0,0); lcd_puts_P("X     0.000 ");
  lcd_gotoxy(0,1); lcd_puts_P("Y     0.000 ");
  sys.last_state = STATE_NULL; lcd_state();
  sys_rt_exec_state = EXEC_STATUS_REPORT; }

void lcd_file(const char *s) {
  register char c;
  uint8_t i = 0;
  while ((c = *s++)) {
    if (c >= 'a' && c <= 'z') {
      lcd_putc(c-'a'+'A');
    } else {
      lcd_putc(c); }
    i++; }
  while (i < 12) {
    lcd_putc(' '); i++; } }

void lcd_error(uint8_t status_code) {
  lcd_clrscr(); lcd_gotoxy(0,0);
  switch(status_code) {
    case 100:
      lcd_puts_P("Insert SD card!");
      break;
    case 101:
      lcd_puts_P("Unknown SD card!");
      break;
    case 102:
      lcd_puts_P("No U-disk files!");
      break;
    default:
      lcd_puts_P("error:");
      uint8_t digit_a = 0, digit_b = 0;
      if (status_code >= 100) {
        digit_a = '0' + status_code % 10;
        status_code /= 10; }
      if (status_code >= 10) {
        digit_b = '0' + status_code % 10;
        status_code /= 10; }
      lcd_putc('0' + status_code);
      if (digit_b) { lcd_putc(digit_b); }
      if (digit_a) { lcd_putc(digit_a); }
      break; } }

void lcd_init() {
  DDR(LCD_DATA_PORT)   |= 0x0F;
  DDR(LCD_RS_PORT)     |= _BV(LCD_RS_BIT);
  DDR(LCD_RW_PORT)     |= _BV(LCD_RW_BIT);
  DDR(LCD_ENABLE_PORT) |= _BV(LCD_ENABLE_BIT);
  delay(LCD_DELAY_BOOTUP);

  LCD_DATA_PORT |= _BV(1);
  LCD_DATA_PORT |= _BV(0);
  lcd_e_toggle();
  delay(LCD_DELAY_INIT);

  lcd_e_toggle();
  delay(LCD_DELAY_INIT_REP);

  lcd_e_toggle();
  delay(LCD_DELAY_INIT_REP);

  LCD_DATA_PORT &= ~_BV(0);
  lcd_e_toggle();
  delay(LCD_DELAY_INIT_4BIT);

  lcd_command(LCD_FUNCTION_DEFAULT);
  lcd_clrscr();
  lcd_command(LCD_MODE_DEFAULT);
  lcd_command(LCD_DISP_ON);

  lcd_clear(); }

ISR(TIMER2_COMPA_vect) {
  if (counter++ > 100) {
    sys_rt_exec_state |= EXEC_STATUS_REPORT;
    counter = 0; } }
