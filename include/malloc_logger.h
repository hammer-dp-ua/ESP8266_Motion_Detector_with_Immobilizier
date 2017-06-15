#include "esp_common.h"
#include "global_definitions.h"

#ifdef USE_MALLOC_LOGGER
   #define FREE(allocated_address_element_to_free)          free_logger(allocated_address_element_to_free)
   #define MALLOC(element_length, line_no, allocated_time)  malloc_logger(element_length, line_no, allocated_time)
   #define ZALLOC(element_length, line_no, allocated_time)  zalloc_logger(element_length, line_no, allocated_time)
#else
   #define FREE(allocated_address_element_to_free)          free(allocated_address_element_to_free)
   #define MALLOC(element_length, line_no, allocated_time)  malloc(element_length)
   #define ZALLOC(element_length, line_no, allocated_time)  zalloc(element_length)
#endif

#ifndef MALLOC_LOGGER
#define MALLOC_LOGGER

#define MALLOC_LOGGER_LIST_SIZE 50

struct malloc_logger_element {
   unsigned int variable_line;
   void * allocated_element_address;
   unsigned int allocated_time;
};

void *zalloc_logger(unsigned int element_size, unsigned int variable_line, unsigned int allocated_time);
char *malloc_logger(unsigned int string_size, unsigned int variable_line, unsigned int allocated_time);
unsigned char get_malloc_logger_list_elements_amount();
void free_logger(void *allocated_address_element_to_free);
struct malloc_logger_element get_last_element_in_logger_list();
void print_not_empty_elements_lines();

#endif
