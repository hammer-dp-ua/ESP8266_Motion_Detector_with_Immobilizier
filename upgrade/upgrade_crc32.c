/*****************************************************
 ** Name         : crc32.c
 ** Author       : tianzx
 ** Version      : 1.0
 ** Date         : 2016-1
 ** Description  : CRC32 Checking
 ******************************************************/
#include "esp_common.h"

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/mem.h"
#include <stdlib.h>
#include "global_definitions.h"

#define BUFSIZE     512
#define CRC_BLOCK_SIZE 512
uint16 start_sec;
static unsigned int *crc_table;

#ifdef MEMLEAK_DEBUG
static const char mem_debug_file[] ICACHE_RODATA_ATTR = __FILE__;
#endif

int init_crc_table(void);
static unsigned int crc32(unsigned int crc, unsigned char * buffer, unsigned int size);

int ICACHE_FLASH_ATTR init_crc_table(void) {
   unsigned int c;
   unsigned int i, j;

   crc_table = (unsigned int*) zalloc(256 * 4);
   if (crc_table == NULL) {
      #ifdef ALLOW_USE_PRINTF
      printf("malloc crc table failed\n");
      #endif

      return -1;
   }

   for (i = 0; i < 256; i++) {
      c = i;

      for (j = 0; j < 8; j++) {
         if (c & 1) {
            c = 0xedb88320 ^ (c >> 1);
         } else {
            c = c >> 1;
         }
      }
      crc_table[i] = c;
   }
   return 0;
}

static unsigned int ICACHE_FLASH_ATTR crc32(unsigned int crc, unsigned char *buffer, unsigned int size) {
   unsigned int i;

   for (i = 0; i < size; i++) {
      crc = crc_table[(crc ^ buffer[i]) & 0xff] ^ (crc >> 8);
   }
   return crc;
}

static int ICACHE_FLASH_ATTR calc_img_crc(unsigned int sumlength, int *img_crc) {
   int fd;
   int ret;
   int i = 0;
   uint8 error = 0;
   unsigned char *buf = (char *) zalloc(CRC_BLOCK_SIZE);

   if (buf == NULL) {
      #ifdef ALLOW_USE_PRINTF
      printf("malloc crc buf failed\n");
      #endif

      free(crc_table);
      return -1;
   }

   unsigned int crc = 0xffffffff;

   uint16 sec_block = sumlength / CRC_BLOCK_SIZE;
   uint32 sec_last = sumlength % CRC_BLOCK_SIZE;

   for (i = 0; i < sec_block; i++) {
      if (0 != (error = spi_flash_read(start_sec * SPI_FLASH_SEC_SIZE + i * CRC_BLOCK_SIZE, (uint32 *) buf, CRC_BLOCK_SIZE))) {
         free(crc_table);
         free(buf);

         #ifdef ALLOW_USE_PRINTF
         printf("spi_flash_read error %d\n", error);
         #endif

         return -1;
      }
      crc = crc32(crc, buf, CRC_BLOCK_SIZE);
   }

   if (sec_last) {
      if (0 != (error = spi_flash_read(start_sec * SPI_FLASH_SEC_SIZE + i * CRC_BLOCK_SIZE, (uint32 *) buf, sec_last))) {
         free(crc_table);
         free(buf);

         #ifdef ALLOW_USE_PRINTF
         printf("spi_flash_read error %d\n", error);
         #endif

         return -1;
      }
      crc = crc32(crc, buf, sec_last);
   }

   *img_crc = abs(crc);
   free(crc_table);
   free(buf);
   return 0;
}

int ICACHE_FLASH_ATTR upgrade_crc_check(uint16 fw_bin_sec, unsigned int sumlength) {
   int ret;
   int img_crc;
   int expected_crc = 0xFF;

   start_sec = fw_bin_sec;

   if (0 != init_crc_table()) {
      return false;
   }

   ret = calc_img_crc(sumlength - 4, &img_crc);
   if (ret < 0) {
      return false;
   }

   #ifdef ALLOW_USE_PRINTF
   printf("Wrote firmware CRC: %d = 0x%x\n", img_crc, img_crc);
   #endif

   spi_flash_read(start_sec * SPI_FLASH_SEC_SIZE + sumlength - 4, &expected_crc, 4);
   expected_crc = abs(expected_crc);

   #ifdef ALLOW_USE_PRINTF
   printf("Expected CRC: %d = 0x%x\n", expected_crc, expected_crc);
   #endif

   if (img_crc == expected_crc) {
      return 0;
   } else {
      return -1;
   }
}

