#ifndef GENERAL_UTILS
#define GENERAL_UTILS

#include "esp_common.h"
#include "global_definitions.h"
#include "malloc_logger.h"
#include "device_settings.h"

#define HEXADECIMAL_ADDRESS_FORMAT "%08x"

LOCAL char RESET_REASON_TEMPLATE[] ICACHE_RODATA_ATTR = "<1>\\n"
      " Fatal exception (<2>):\\n"
      " epc1=0x<3>, epc2=0x<4>, epc3=0x<5>, excvaddr=0x<6>, depc=0x<7>, rtn_addr=0x<8>\\n"
      " used software: <9>";

void set_flag(unsigned int *flags, unsigned int flag_value);
void reset_flag(unsigned int *flags, unsigned int flag_value);
bool read_flag(unsigned int flags, unsigned int flag_value);
void *set_string_parameters(char string[], char *parameters[]);
char *generate_post_request(char *request);
char *get_string_from_rom(const char *rom_string);
bool compare_strings(char *string1, char *string2);
void pin_output_set(unsigned int pin);
void pin_output_reset(unsigned int pin);
bool read_output_pin_state(unsigned int pin);
bool read_input_pin_state(unsigned int pin);
char *generate_reset_reason();
void set_default_wi_fi_settings();

#endif
