#ifndef protocol_h
#define protocol_h

#define LINE_BUFFER_SIZE 30

void protocol_main_loop();
void protocol_buffer_synchronize();
void protocol_execute_realtime();
void protocol_exec_rt_system();
void protocol_auto_cycle_start();

#endif
