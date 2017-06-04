#include "esp_common.h"
#include "global_definitions.h"

#ifndef MALLOC_LOGGER
#define MALLOC_LOGGER

#define MALLOC_LOGGER_LIST_SIZE 50

struct malloc_logger_element {
   unsigned int variable_line;
   void * allocated_element_address;
};

void *zalloc_logger(unsigned int element_size, unsigned int variable_line);
char *malloc_logger(unsigned int string_size, unsigned int variable_line);
unsigned char get_malloc_logger_list_elements_amount();
void free_logger(void *allocated_address_element_to_free);
struct malloc_logger_element get_last_element_in_logger_list();
void print_not_empty_elements_lines();

#endif
