#include "grbl.h"
#include "fat.h"
#include "partition.h"
#include "sd_raw.h"
#include "sd_raw_config.h"

#define LINE_FLAG_OVERFLOW bit(0)

static char line[LINE_BUFFER_SIZE];

void protocol_main_loop() {
  uint8_t line_flags = 0;
  uint8_t char_counter = 0;
  uint8_t c, idx, val, f_cnt;
  struct partition_struct *partition;
  struct fat_fs_struct *fs;
  struct fat_dir_struct *dd;
  struct fat_file_struct *fd;
  struct fat_dir_entry_struct dir_entry;
  for (;;) {
    if (btn_state && !(btn_state & (1<<4))) {
      if (btn_state & (1<<7)) {
        btn_state = 0;
        #ifndef COREXY
        if ((val = limits_go_home()) != STATUS_OK) {
          lcd_error(val);
          while(1) {
            if (abs(adc_read(CONTROL_OTHER) - (val = adc_read(CONTROL_OTHER))) <= 2) {
              if (val < 58) {
                if (btn) {
                  btn = 0;
                  break; }
              } else {
                btn = 1; } }
            _delay_ms(10); }
          lcd_clear(); }
        #endif
      } else if (btn_state & (1<<6)) {
        if (sd_raw_init()) {
          btn_state = 0;
          sys.state2 = STATE_FILE;
          if (!(partition = partition_open(sd_raw_read,sd_raw_read_interval,0))) {
            if (!(partition = partition_open(sd_raw_read,sd_raw_read_interval,-1))) {
              lcd_error(101); } }
          if (partition) {
            if ((fs = fat_open(partition))) {
              fat_get_dir_entry_of_path(fs, "/", &dir_entry);
              if ((dd = fat_open_dir(fs, &dir_entry))) {
                idx = 0; f_cnt = 0;
                while (fat_read_dir(dd, &dir_entry)) {
                  if (!(dir_entry.attributes & FAT_ATTRIB_DIR)) {
                    f_cnt++; } }
                if (f_cnt > 0) {
                  lcd_clrscr(); val = 0;
                  fat_reset_dir(dd);
                  while (fat_read_dir(dd, &dir_entry)) {
                    if (!(dir_entry.attributes & FAT_ATTRIB_DIR)) {
                      lcd_gotoxy(0,val);
                      if (val == 0) {
                        lcd_puts_P("~"); lcd_file(dir_entry.long_name);
                      } else {
                        lcd_puts_P(" "); lcd_file(dir_entry.long_name); break; }
                      val++; } }
                  while(1) {
                    if (f_cnt > 1) {
                      if ((val = adc_read(CONTROL_JOG_Y)) < 63) {
                        if (!(btn_state & (1<<4))) {
                          btn_state |= (1<<4);
                          val = 0; idx++;
                          fat_reset_dir(dd);
                          if (idx < f_cnt) {
                            while (fat_read_dir(dd, &dir_entry)) {
                              if (!(dir_entry.attributes & FAT_ATTRIB_DIR)) {
                                if (val == idx-1) {
                                  lcd_gotoxy(0,0);
                                  lcd_puts_P(" "); lcd_file(dir_entry.long_name);
                                } else if (val == idx) {
                                  lcd_gotoxy(0,1);
                                  lcd_puts_P("~"); lcd_file(dir_entry.long_name); break; }
                                val++; } }
                          } else {
                            idx = 0;
                            while (fat_read_dir(dd, &dir_entry)) {
                              if (!(dir_entry.attributes & FAT_ATTRIB_DIR)) {
                                if (val == idx) {
                                  lcd_gotoxy(0,0);
                                  lcd_puts_P("~"); lcd_file(dir_entry.long_name);
                                } else if (val == idx+1) {
                                  lcd_gotoxy(0,1);
                                  lcd_puts_P(" "); lcd_file(dir_entry.long_name); break; }
                                val++; } } } }
                      } else if (val < 191) {
                        if (!(btn_state & (1<<4))) {
                          btn_state |= (1<<4);
                          val = 0;
                          fat_reset_dir(dd);
                          if (idx > 0) {
                            idx--;
                            while (fat_read_dir(dd, &dir_entry)) {
                              if (!(dir_entry.attributes & FAT_ATTRIB_DIR)) {
                                if (val == idx) {
                                  lcd_gotoxy(0,0);
                                  lcd_puts_P("~"); lcd_file(dir_entry.long_name);
                                } else if (val == idx+1) {
                                  lcd_gotoxy(0,1);
                                  lcd_puts_P(" "); lcd_file(dir_entry.long_name); break; }
                                val++; } }
                          } else {
                            idx = f_cnt-1;
                            while (fat_read_dir(dd, &dir_entry)) {
                              if (!(dir_entry.attributes & FAT_ATTRIB_DIR)) {
                                if (val == idx-1) {
                                  lcd_gotoxy(0,0);
                                  lcd_puts_P(" "); lcd_file(dir_entry.long_name);
                                } else if (val == idx) {
                                  lcd_gotoxy(0,1);
                                  lcd_puts_P("~"); lcd_file(dir_entry.long_name); break; }
                                val++; } } } }
                      } else {
                        btn_state &= ~(1<<4); } }
                    if (btn_state & (1<<4)) { continue; }
                    if (abs(adc_read(CONTROL_OTHER) - (val = adc_read(CONTROL_OTHER))) <= 2) {
                      if (val < 58) {
                        if (btn) {
                          btn = 0;
                          if (val < 21) {
                            break; }
                          fat_reset_dir(dd); val = 0;
                          while (fat_read_dir(dd, &dir_entry)) {
                            if (!(dir_entry.attributes & FAT_ATTRIB_DIR)) {
                              if (val == idx) {
                                fd = fat_open_file(fs, &dir_entry);
                                break; }
                              val++; } }
                          break; }
                      } else {
                        btn = 1; } }
                    _delay_ms(10); }
                  if (fd) {
                    lcd_clear();

                    sys.f_override = 100;
                    pl_data.feed_rate = 1201.2;
                    pl_data.spindle_speed = 255;
                    pl_data.dwell = 0;
                    pl_data.units = 0;
                    pl_data.distance = 0;
                    pl_data.laserOn = 1;
                    pl_data.condition = PL_COND_FLAG_RAPID_MOTION;
                    pl_data.gc_pos[X_AXIS] = ((pl_data.xyz[X_AXIS] = plan_get_position(X_AXIS)) - wco[X_AXIS]) / settings.steps_per_mm[X_AXIS];
                    pl_data.gc_pos[Y_AXIS] = ((pl_data.xyz[Y_AXIS] = plan_get_position(Y_AXIS)) - wco[Y_AXIS]) / settings.steps_per_mm[Y_AXIS];
                    spindle_set_speed(SPINDLE_PWM_OFF_VALUE); lcd_state2();
                    char_counter = 0; val = STATUS_OK;
                    while ((c = fat_read_byte(fd)) != 0xff) {
                      if (c == ';') {
                        if (sys.abort) { break; }

                        if (sys.state != STATE_HOLD) {
                          protocol_execute_realtime();
                          if (sys.abort) { break; } }

                        line[char_counter] = ';';

                        if ((val = gc_execute_line(line)) != STATUS_OK) {
                          break; }

                        char_counter = 0;

                      } else if (c == '\n' || c == '\r') {
                        if (char_counter > 0) {
                          if (sys.abort) { break; }

                          if (sys.state != STATE_HOLD) {
                            protocol_execute_realtime();
                            if (sys.abort) { break; } }

                          line[char_counter] = 0;

                          if ((val = gc_execute_line2(line)) != STATUS_OK) {
                            break; }

                          char_counter = 0; }

                      } else if (c != ' ') {

                        if (char_counter >= (LINE_BUFFER_SIZE-1)) {
                          val = STATUS_OVERFLOW; break;
                        } else {
                          line[char_counter++] = c; } } }

                    protocol_buffer_synchronize();
                    if (sys.abort) { val = STATUS_OK; }
                    if (val != STATUS_OK) {
                      lcd_error(val);
                      while(1) {
                        if (abs(adc_read(CONTROL_OTHER) - (val = adc_read(CONTROL_OTHER))) <= 2) {
                          if (val < 58) {
                            if (btn) {
                              btn = 0;
                              break; }
                          } else {
                            btn = 1; } }
                        _delay_ms(10); }
                      lcd_clear(); }

                    fat_close_file(fd);
                    fat_close_dir(dd);
                    fat_close(fs);
                    partition_close(partition);
                    return; }
                } else {
                  lcd_error(102); }
              } else {
                lcd_error(101); }
            } else {
              lcd_error(101); } }
        } else {
          lcd_error(100); }
        fat_close_dir(dd);
        fat_close(fs);
        partition_close(partition);
        if (f_cnt == 0) {
          while(1) {
            if (abs(adc_read(CONTROL_OTHER) - (val = adc_read(CONTROL_OTHER))) <= 2) {
              if (val < 58) {
                if (btn) {
                  if (val < 21) {
                    btn_state = 0; }
                  btn = 0;
                  break; }
              } else {
                btn = 1; } }
            _delay_ms(10); } }
        if (btn_state) {
          continue;
        } else {
          lcd_clear(); }
      } else if (btn_state & (1<<5)) {
        btn_state = 0;
        val = 35;
        while (!btn && val > 0) {
          _delay_ms(10);
          buttons_check();
          val--; }
        if (!btn) {
          if (bit_isfalse(settings.flags,BITFLAG_HOMING_ENABLE)) {
            memset(sys_position, 0, sizeof(sys_position));
            plan_sync_position();
          } else {
            memcpy(wco, sys_position, sizeof(wco));
            settings_write_coord_data(); }
        } else {
          sys.state2 = STATE_ORIGIN;
          gc_execute_line("IN;"); }
      } else {
        val = 35;
        jog_execute_line(1);
        while (btn_state && val > 0) {
          _delay_ms(10);
          buttons_check();
          protocol_exec_rt_system();
          if (sys.abort) { return; }
          val--; }
        while (btn_state) {
          protocol_execute_realtime();
          if (sys.abort) { return; }
          jog_execute_line(2); }
        if (sys.abort) { return; } }
      serial_reset_read_buffer(); }

    while ((c = serial_read()) != SERIAL_NO_DATA) {
      if (c == ';') {
        if (sys.abort) { return; }

        if (sys.state != STATE_HOLD) {
          protocol_execute_realtime();
          if (sys.abort) { return; } }

        line[char_counter] = ';';

        if (line_flags & LINE_FLAG_OVERFLOW) {
          report_status_message(STATUS_OVERFLOW);
        } else if (line[0] == '$') {
          report_status_message(system_execute_line(line));
        } else if (sys.state & (STATE_ALARM | STATE_HOMING | STATE_JOG)) {
          report_status_message(STATUS_SYSTEM_GC_LOCK);
        } else {
          report_status_message(gc_execute_line(line)); }

        line_flags = 0;
        char_counter = 0;

      } else if (c == '#') {

        line_flags = 0;
        char_counter = 0;

      } else {

        if (!line_flags) {
          if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            line_flags |= LINE_FLAG_OVERFLOW;
          } else {
            line[char_counter++] = c; } } } }
    if (sys.abort) { return; }

    if (sys.state != STATE_HOLD) {
      protocol_execute_realtime();
      if (sys.abort) { return; } } }

  return; }

void protocol_buffer_synchronize() {
  while (plan_get_current_block() || (sys.state == STATE_CYCLE)) {
    protocol_execute_realtime();
    if (sys.abort) { return; } } }

void protocol_auto_cycle_start() {
  if (plan_get_current_block() != NULL) {
    system_set_exec_state_flag(EXEC_CYCLE_START); } }

void protocol_execute_realtime() {
  buttons_check();
  protocol_exec_rt_system();
  while (sys.suspend) {
    buttons_check();
    protocol_exec_rt_system(); } }

void protocol_exec_rt_system() {
  uint8_t rt_exec = sys_rt_exec_state;

  if (rt_exec) {

    if (rt_exec & (EXEC_RESET | EXEC_FEED_HOLD)) {
      if (sys.state == STATE_CYCLE) {
        sys.step_control = STEP_CONTROL_EXECUTE_HOLD;
        st_update_plan_block_parameters();
        if (rt_exec & EXEC_RESET) { sys.suspend = 1; }
      } else if (sys.state == STATE_HOLD || sys.state == STATE_IDLE) {
        if (rt_exec & EXEC_RESET) {
          sys.state = STATE_IDLE;
          sys.abort = 1;
          sys.suspend = 0;
          sys.non_modal_dwell = 0;
          return; }
      } else if (sys.state == STATE_HOMING) {
        if (rt_exec & EXEC_RESET) {
          sys.abort = 1; } }
      system_clear_exec_state_flag((EXEC_RESET | EXEC_FEED_HOLD)); }

    if (rt_exec & EXEC_CYCLE_START) {
      if (sys.state == STATE_IDLE || sys.state == STATE_HOLD) {
        rt_exec |= EXEC_STATUS_REPORT;
        sys.step_control = STEP_CONTROL_NORMAL_OP;
        sys.suspend = 0;
        if (plan_get_current_block()) {
          sys.state = STATE_CYCLE;
          st_prep_buffer();
          st_wake_up();
        } else {
          if (!sys.non_modal_dwell) {
            sys.state = STATE_IDLE; } } }
      system_clear_exec_state_flag(EXEC_CYCLE_START); }

    if (rt_exec & EXEC_STATUS_REPORT) {
      lcd_position(); lcd_state();
      system_clear_exec_state_flag(EXEC_STATUS_REPORT); }

    if (rt_exec & EXEC_CYCLE_STOP) {
      sys_rt_exec_state |= EXEC_STATUS_REPORT;
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
        plan_cycle_reinitialize();
        bit_false(sys.step_control, STEP_CONTROL_EXECUTE_HOLD);
        if (sys.suspend) {
          sys.state = STATE_IDLE;
          sys.abort = 1;
          sys.suspend = 0;
          return;
        } else {
          sys.state = STATE_HOLD;
          sys.suspend = 1; }
      } else {
        sys.state = STATE_IDLE;
        protocol_auto_cycle_start(); }
      system_clear_exec_state_flag(EXEC_CYCLE_STOP); } }

  if (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_JOG)) {
    st_prep_buffer(); } }
