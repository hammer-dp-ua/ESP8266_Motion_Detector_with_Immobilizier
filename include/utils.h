#include "esp_common.h"
#include "global_definitions.h"
#include "malloc_logger.h"

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
