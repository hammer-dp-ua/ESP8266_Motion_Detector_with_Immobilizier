/***************************************************************************** *
 * Copyright (C) 2014 -2016  Espressif System
 *
 * FileName: user_upgrade.c
 *
 * Description: downlaod upgrade userbin file from upgrade server
 *
 * Modification history:
 * 2015/7/3, v1.0 create this file.
 *******************************************************************************/
//#include "version.h"
//#include "user_config.h"
#include "esp_common.h"
#include "lwip/mem.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "upgrade.h"

#ifndef true // needed only for Eclipse
   typedef unsigned char bool;
   #define true 1
   #define false 0
#endif

/*the size cannot be bigger than below*/
#define UPGRADE_DATA_SEG_LEN 1460
#define UPGRADE_RETRY_TIMES 10

LOCAL uint32 totallength = 0;
LOCAL uint32 sumlength = 0;
LOCAL BOOL flash_erased = 0;

char *precv_buf = NULL;
os_timer_t upgrade_timer;
xTaskHandle *pxCreatedTask = NULL;

#ifdef UPGRADE_SSL_ENABLE

#endif
/******************************************************************************
 * FunctionName : upgrade_deinit
 * Description  :
 * Parameters   :
 * Returns      : none
 *******************************************************************************/
void
LOCAL upgrade_deinit(void) {
   if (system_upgrade_flag_check() != UPGRADE_FLAG_START) {
      system_upgrade_deinit();
      //system_upgrade_reboot();
   }
}

/******************************************************************************
 * FunctionName : upgrade_data_load
 * Description  : parse the data from server,send fw data to system interface 
 * Parameters   : pusrdata--data from server,
 *              : length--length of the pusrdata
 * Returns      : none
 *  
 * first data from server:
 * HTTP/1.1 200 OK
 * Server: nginx/1.6.2
 * Date: Tue, 14 Jul 2015 09:15:51 GMT
 * Content-Type: application/octet-stream
 * Content-Length: 282448
 * Connection: keep-alive
 * Content-Disposition: attachment;filename=user2.bin
 * Vary: Cookie
 * X-RateLimit-Remaining: 3599
 * X-RateLimit-Limit: 3600
 * X-RateLimit-Reset: 1436866251
 *******************************************************************************/
BOOL upgrade_data_load(char *pusrdata, unsigned short length) {
   char *ptr = NULL;
   char *ptmp2 = NULL;
   char lengthbuffer[32]; // For a value of Content-Length header

   if (totallength == 0
         && (ptr = (char *) strstr(pusrdata, "\r\n\r\n")) != NULL
         && (ptr = (char *) strstr(pusrdata, "Content-Length")) != NULL) {

#ifdef ALLOW_USE_PRINTF
      printf("\n pusrdata: %s\n", pusrdata);
#endif

      ptr = (char *) strstr(pusrdata, "Content-Length: ");
      if (ptr != NULL) {
         ptr += 16; // Jumps to Content-Length value
         ptmp2 = (char *) strstr(ptr, "\r\n");

         if (ptmp2 != NULL) {
            memset(lengthbuffer, 0, sizeof(lengthbuffer));

            if ((ptmp2 - ptr) <= 32) {
               memcpy(lengthbuffer, ptr, ptmp2 - ptr);
            } else {
#ifdef ALLOW_USE_PRINTF
               printf("ERR1: arr_overflow, %u, %d\n", __LINE__, ptmp2 - ptr);
#endif
            }

            sumlength = atoi(lengthbuffer); // Value of Content-Length header

#ifdef ALLOW_USE_PRINTF
            printf("userbin sumlength: %d\n", sumlength);
#endif

            ptr = (char *) strstr(pusrdata, "\r\n\r\n"); // End of request
            length -= ptr - pusrdata;
            length -= 4; // \r\n\r\n
            totallength = length; // Received bytes headers excluded

            /*
             * At the beginning of the upgrade, we get the sumlength and erase all the target flash sectors, return false
             * to close the connection and start upgrade again.
             *
             * There is file content after "ptr + 4"
             */
            if (FALSE == flash_erased) {
               flash_erased = system_upgrade(ptr + 4, sumlength);
               return flash_erased;
            } else {
               system_upgrade(ptr + 4, length);
            }
         } else {
#ifdef ALLOW_USE_PRINTF
            printf("ERROR: get sumlength failed\n");
#endif

            return false;
         }
      } else {
#ifdef ALLOW_USE_PRINTF
         printf("ERROR: get Content-Length failed\n");
#endif

         return false;
      }
   } else {
      if (totallength != 0) {
         totallength += length;

         if (totallength > sumlength) {
#ifdef ALLOW_USE_PRINTF
            printf("strip the 400 error mesg\n");
#endif

            length = length - (totallength - sumlength);
         }

         //printf(">>>recv %dB, %dB left\n", totallength, sumlength - totallength);
         system_upgrade(pusrdata, length);
      } else {
#ifdef ALLOW_USE_PRINTF
         printf("server response with something else, check it!\n");
#endif

         return false;
      }
   }

   return true;
}
#ifdef UPGRADE_SSL_ENABLE

#else
/******************************************************************************
 * FunctionName : upgrade_task
 * Description  : task to connect with target server and get firmware data 
 * Parameters   : pvParameters--save the server address\port\request frame for
 *              : the upgrade server\call back functions to tell the userapp
 *              : the result of this upgrade task
 * Returns      : none
 *******************************************************************************/
void upgrade_task(void *pvParameters) {
   int recbytes;
   int sta_socket;
   int retry_count = 0;
   struct ip_info ipconfig;
   struct upgrade_server_info *server = pvParameters;

   flash_erased = FALSE;
   precv_buf = (char*) malloc(UPGRADE_DATA_SEG_LEN);

#ifdef ALLOW_USE_PRINTF
   if (NULL == precv_buf) {
      printf("upgrade_task, memory exhausted, check it\n");
   }
#endif

   while (retry_count++ < UPGRADE_RETRY_TIMES) {
      wifi_get_ip_info(STATION_IF, &ipconfig);

      /* check the ip address or net connection state*/
      while (ipconfig.ip.addr == 0) {
         vTaskDelay(1000 / portTICK_RATE_MS);
         wifi_get_ip_info(STATION_IF, &ipconfig);
      }

      sta_socket = socket(PF_INET, SOCK_STREAM, 0);
      if (-1 == sta_socket) {
         close(sta_socket);
         vTaskDelay(1000 / portTICK_RATE_MS);

#ifdef ALLOW_USE_PRINTF
         printf("socket fail!\n");
#endif

         continue;
      }

      /*for upgrade connection debug*/
      //server->sockaddrin.sin_addr.s_addr= inet_addr("192.168.1.170");
      if (0 != connect(sta_socket, (struct sockaddr * ) (&server->sockaddrin), sizeof(struct sockaddr))) {
         close(sta_socket);
         vTaskDelay(1000 / portTICK_RATE_MS);

#ifdef ALLOW_USE_PRINTF
         printf("connect fail!\n");
#endif

         continue;
      }

#ifdef ALLOW_USE_PRINTF
      printf("Connect OK!\n");
#endif

      system_upgrade_init();
      system_upgrade_flag_set(UPGRADE_FLAG_START);

      if (write(sta_socket, server->url, strlen(server->url) + 1 ) < 0) {
         close(sta_socket);
         vTaskDelay(1000 / portTICK_RATE_MS);

#ifdef ALLOW_USE_PRINTF
         printf("send fail!\n");
#endif
         continue;
      }

#ifdef ALLOW_USE_PRINTF
      printf("Request send success\n");
#endif

      while ((recbytes = read(sta_socket, precv_buf, UPGRADE_DATA_SEG_LEN)) > 0) {
         if (FALSE == flash_erased) {
            close(sta_socket);

#ifdef ALLOW_USE_PRINTF
            printf("pre erase flash!\n");
#endif

            upgrade_data_load(precv_buf, recbytes);
            break;
         }

         if (false == upgrade_data_load(precv_buf, recbytes)) {
#ifdef ALLOW_USE_PRINTF
            printf("upgrade data error!\n");
#endif

            close(sta_socket);
            flash_erased = FALSE;
            vTaskDelay(1000 / portTICK_RATE_MS);
            break;
         }

         /*
          * This two length data should be equal, if totallength is bigger,
          * maybe data wrong or server send extra info, drop it anyway
          */
         if (totallength >= sumlength) {
#ifdef ALLOW_USE_PRINTF
            printf("upgrade data load finish\n");
#endif

            close(sta_socket);
            goto finish;
         }

#ifdef ALLOW_USE_PRINTF
         //printf("upgrade_task %d word left\n", uxTaskGetStackHighWaterMark(NULL));
#endif
      }

      if (recbytes <= 0) {
         close(sta_socket);
         flash_erased = FALSE;
         vTaskDelay(1000 / portTICK_RATE_MS);

#ifdef ALLOW_USE_PRINTF
         printf("ERROR: read data fail!\n");
#endif
      }

      totallength = 0;
      sumlength = 0;
   }

finish:
   os_timer_disarm(&upgrade_timer);

   if (upgrade_crc_check(system_get_fw_start_sec(), sumlength) != 0) {
#ifdef ALLOW_USE_PRINTF
      printf("upgrade crc check failed!\n");
#endif

      server->upgrade_flag = false;
      system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
   } else {
      if (retry_count == UPGRADE_RETRY_TIMES) {
         /*retry too many times, fail*/
         server->upgrade_flag = false;
         system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
      } else {
         server->upgrade_flag = true;
         system_upgrade_flag_set(UPGRADE_FLAG_FINISH);
      }
   }

   if (NULL != precv_buf) {
      free(precv_buf);
   }

   totallength = 0;
   sumlength = 0;
   flash_erased = FALSE;

   upgrade_deinit();

#ifdef ALLOW_USE_PRINTF
   printf("\n Exit upgrade task\n");
#endif

   if (server->check_cb != NULL) {
      server->check_cb(server);
   }
   vTaskDelay(100 / portTICK_RATE_MS);
   vTaskDelete(NULL);
}
#endif
/******************************************************************************
 * FunctionName : upgrade_check
 * Description  : check the upgrade process, if not finished in 300S,exit
 * Parameters   : pvParameters--save the server address\port\request frame for
 * Returns      : none
 *******************************************************************************/
LOCAL void upgrade_check(struct upgrade_server_info *server) {
   /*network not stable, upgrade data lost, this may be called*/
   vTaskDelete(pxCreatedTask);
   os_timer_disarm(&upgrade_timer);

   if (NULL != precv_buf) {
      free(precv_buf);
   }

   totallength = 0;
   sumlength = 0;
   flash_erased = FALSE;

   /*take too long to finish,fail*/
   server->upgrade_flag = false;
   system_upgrade_flag_set(UPGRADE_FLAG_IDLE);

   upgrade_deinit();

#ifdef ALLOW_USE_PRINTF
   printf("\n upgrade fail, exit\n");
#endif

   if (server->check_cb != NULL) {
      server->check_cb(server);
   }
}

#ifdef UPGRADE_SSL_ENABLE
#else
/******************************************************************************
 * FunctionName : system_upgrade_start
 * Description  : task to connect with target server and get firmware data 
 * Parameters   : pvParameters--save the server address\port\request frame for
 *              : the upgrade server\call back functions to tell the userapp
 *              : the result of this upgrade task
 * Returns      : true if task created successfully, false failed.
 *******************************************************************************/

BOOL system_upgrade_start(struct upgrade_server_info *server) {
   portBASE_TYPE ret = 0;

   if (NULL == pxCreatedTask) {
      ret = xTaskCreate(upgrade_task, "upgrade_task", 224, server, 5, pxCreatedTask); //1024, 890 left

      if (pdPASS == ret) {
         os_timer_disarm(&upgrade_timer);
         os_timer_setfn(&upgrade_timer, (os_timer_func_t *) upgrade_check, server);
         os_timer_arm(&upgrade_timer, 1200000, 0);
      }
   }
   return (pdPASS == ret);
}
#endif

