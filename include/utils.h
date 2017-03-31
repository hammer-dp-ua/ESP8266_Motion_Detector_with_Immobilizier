#include "esp_common.h"

#ifndef true // needed only for Eclipse
   typedef unsigned char bool;
   #define true 1
   #define false 0
#endif

void set_flag(unsigned int *flags, unsigned int flag_value);
void reset_flag(unsigned int *flags, unsigned int flag_value);
bool read_flag(unsigned int flags, unsigned int flag_value);
void *set_string_parameters(char string[], char *parameters[]);
char *generate_post_request(char *request);
char *get_string_from_rom(const char *rom_string);
void pin_output_set(unsigned int pin);
void pin_output_reset(unsigned int pin);
bool read_output_pin_state(unsigned int pin);
bool read_input_pin_state(unsigned int pin);
