#include "malloc_logger.h"

#ifdef MALLOC_LOGGER

struct malloc_logger_element malloc_logger_list[MALLOC_LOGGER_LIST_SIZE];

LOCAL void check_is_full(unsigned char current_amount) {
   #ifdef ALLOW_USE_PRINTF
   if (current_amount >= MALLOC_LOGGER_LIST_SIZE) {
      printf("\n !malloc_logger_list is full\n");
   }
   #endif
}

void *zalloc_logger(unsigned int element_size, unsigned int variable_line, unsigned int allocated_time) {
   void *allocated_address = zalloc(element_size);
   unsigned char i;

   for (i = 0; i < MALLOC_LOGGER_LIST_SIZE; i++) {
      if (malloc_logger_list[i].allocated_element_address == NULL) {
         malloc_logger_list[i].allocated_element_address = allocated_address;
         malloc_logger_list[i].variable_line = variable_line;
         malloc_logger_list[i].allocated_time = allocated_time;
         break;
      }
   }

   check_is_full(i);
   return allocated_address;
}

char *malloc_logger(unsigned int string_size, unsigned int variable_line, unsigned int allocated_time) {
   char *allocated_string = malloc(string_size);
   unsigned char i;

   for (i = 0; i < MALLOC_LOGGER_LIST_SIZE; i++) {
      if (malloc_logger_list[i].allocated_element_address == NULL) {
         malloc_logger_list[i].allocated_element_address = allocated_string;
         malloc_logger_list[i].variable_line = variable_line;
         malloc_logger_list[i].allocated_time = allocated_time;
         break;
      }
   }

   check_is_full(i);
   return allocated_string;
}

void free_logger(void *allocated_address_element_to_free) {
   unsigned char i;

   for (i = 0; i < MALLOC_LOGGER_LIST_SIZE; i++) {
      if (malloc_logger_list[i].allocated_element_address == allocated_address_element_to_free) {
         free(allocated_address_element_to_free);
         malloc_logger_list[i].allocated_element_address = NULL;
         malloc_logger_list[i].variable_line = 0;
         malloc_logger_list[i].allocated_time = 0;
         break;
      }
   }
}

unsigned char get_malloc_logger_list_elements_amount() {
   unsigned char i = 0;
   unsigned char amount = 0;

   while (i < MALLOC_LOGGER_LIST_SIZE) {
      if (malloc_logger_list[i].allocated_element_address != NULL) {
         amount++;
      }
      i++;
   }
   return amount;
}

struct malloc_logger_element get_last_element_in_logger_list() {
   unsigned char i;

   for(i = MALLOC_LOGGER_LIST_SIZE - 1; i != 0xFF; i--) {
      if (malloc_logger_list[i].allocated_element_address != NULL) {
         return malloc_logger_list[i];
         break;
      }
   }
   return malloc_logger_list[MALLOC_LOGGER_LIST_SIZE - 1];
}

void print_not_empty_elements_lines() {
   unsigned int i;

   for (i = 0; i < MALLOC_LOGGER_LIST_SIZE; i++) {
      if (malloc_logger_list[i].allocated_element_address != NULL) {
         printf(" element's variable line: %u, allocated time: %u\n",
               malloc_logger_list[i].variable_line, malloc_logger_list[i].allocated_time);
      }
   }
}

#endif
