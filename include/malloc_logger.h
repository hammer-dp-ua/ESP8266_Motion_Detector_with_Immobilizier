#include "esp_common.h"
#include "global_definitions.h"

#ifdef USE_MALLOC_LOGGER
   #define FREE(allocated_address_element_to_free)    free_logger(allocated_address_element_to_free)
   #define MALLOC(element_length, line_no)            malloc_logger(element_length, line_no)
   #define ZALLOC(element_length, line_no)            zalloc_logger(element_length, line_no)
#else
   #define FREE(allocated_address_element_to_free) free(allocated_address_element_to_free)
   #define MALLOC(element_length, line_no)         malloc(element_length)
   #define ZALLOC(element_length, line_no)         zalloc(element_length)
#endif

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
