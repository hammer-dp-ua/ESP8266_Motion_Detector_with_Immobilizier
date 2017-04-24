/**
 * Pins 4 and 5 on some ESP8266-07 are exchanged on silk screen!!!
 *
 * SPI SPI_CPOL & SPI_CPHA:
 *    SPI_CPOL - (0) Clock is low when inactive
 *               (1) Clock is high when inactive
 *    SPI_CPHA - (0) Data is valid on clock leading edge
 *               (1) Data is valid on clock trailing edge
 */

#include "esp_common.h"

// driver libs
#include "uart.h"
#include "gpio.h"
#include "spi_interface.h"

#include "esp_sta.h"
#include "esp_wifi.h"
#include "upgrade.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "device_settings.h"
#include "espconn.h"
#include "utils.h"
#include "lwip/sys.h"
#include "lwip/inet.h"
#include "user_main.h"
#include "global_printf_usage.h"

unsigned int milliseconds_g;
int signal_strength_g;
unsigned short errors_counter_g;
LOCAL os_timer_t millisecons_time_serv_g;

struct _esp_tcp user_tcp;

unsigned char responses_index;
char *responses[10];
unsigned int general_flags;

struct motion_sensor *alarm_sources_g[ALARM_SOURCES_AMOUNT];

xSemaphoreHandle status_request_semaphore_g;
xSemaphoreHandle requests_mutex_g;
xSemaphoreHandle buzzer_semaphore_g;

xTimerHandle alarm_timers[ALARM_TIMERS_MAX_AMOUNT];

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
 *******************************************************************************/
uint32 user_rf_cal_sector_set(void) {
   flash_size_map size_map = system_get_flash_size_map();
   uint32 rf_cal_sec = 0;

   switch (size_map) {
      case FLASH_SIZE_4M_MAP_256_256:
         rf_cal_sec = 128 - 5;
         break;

      case FLASH_SIZE_8M_MAP_512_512:
         rf_cal_sec = 256 - 5;
         break;

      case FLASH_SIZE_16M_MAP_512_512:
      case FLASH_SIZE_16M_MAP_1024_1024:
         rf_cal_sec = 512 - 5;
         break;

      case FLASH_SIZE_32M_MAP_512_512:
      case FLASH_SIZE_32M_MAP_1024_1024:
         rf_cal_sec = 1024 - 5;
         break;

      default:
         rf_cal_sec = 0;
         break;
   }
   return rf_cal_sec;
}

LOCAL void milliseconds_counter() {
   milliseconds_g++;
}

void start_100millisecons_counter() {
   os_timer_disarm(&millisecons_time_serv_g);
   os_timer_setfn(&millisecons_time_serv_g, (os_timer_func_t *) milliseconds_counter, NULL);
   os_timer_arm(&millisecons_time_serv_g, 100, 1); // 100 ms
}

void stop_milliseconds_counter() {
   os_timer_disarm(&millisecons_time_serv_g);
}

// Callback function when AP scanning is completed
void get_ap_signal_strength(void *arg, STATUS status) {
   if (status == OK) {
      struct bss_info *got_bss_info = (struct bss_info *) arg;

      signal_strength_g = got_bss_info->rssi;
      //got_bss_info = got_bss_info->next.stqe_next;
   }
}

void scan_access_point_task(void *pvParameters) {
   long rescan_when_connected_task_delay = 10 * 60 * 1000 / portTICK_RATE_MS; // 10 mins
   long rescan_when_not_connected_task_delay = 10 * 1000 / portTICK_RATE_MS; // 10 secs

   for (;;) {
      STATION_STATUS status = wifi_station_get_connect_status();

      if (status == STATION_GOT_IP) {
         struct scan_config ap_scan_config;
         char *default_access_point_name = get_string_from_rom(ACCESS_POINT_NAME);

         ap_scan_config.ssid = default_access_point_name;
         wifi_station_scan(&ap_scan_config, get_ap_signal_strength);
         free(default_access_point_name);

         vTaskDelay(rescan_when_connected_task_delay);
      } else {
         vTaskDelay(rescan_when_not_connected_task_delay);
      }
   }
}

void autoconnect_task(void *pvParameters) {
   long task_delay = 10000 / portTICK_RATE_MS;

   for (;;) {
      STATION_STATUS status = wifi_station_get_connect_status();
      read_output_pin_state(AP_CONNECTION_STATUS_LED_PIN);
      if (status != STATION_GOT_IP && status != STATION_CONNECTING) {
         wifi_station_connect(); // Do not call this API in user_init
      }
      vTaskDelay(task_delay);
   }
}

void ICACHE_FLASH_ATTR testing_task(void *pvParameters) {
   for (;;) {
      vTaskDelay(1000 / portTICK_RATE_MS);
   }
}

void beep_task() {
   #ifdef ALLOW_USE_PRINTF
   printf("beep_task has been created. Time: %u\n", milliseconds_g);
   #endif

   vSemaphoreCreateBinary(buzzer_semaphore_g);

   pin_output_set(BUZZER_PIN);
   vTaskDelay(80 / portTICK_RATE_MS);
   pin_output_reset(BUZZER_PIN);

   vTaskDelay(500 / portTICK_RATE_MS);

   if (xSemaphoreTake(buzzer_semaphore_g, REQUEST_MAX_DURATION_TIME) == pdPASS) {
      pin_output_set(BUZZER_PIN);
      vTaskDelay(100 / portTICK_RATE_MS);
      pin_output_reset(BUZZER_PIN);
   }

   vSemaphoreDelete(buzzer_semaphore_g);
   buzzer_semaphore_g = NULL;
   vTaskDelete(NULL);
}

void successfull_connected_tcp_handler_callback(void *arg) {
   struct espconn *connection = arg;
   struct connection_user_data *user_data = connection->reserve;
   char *request = user_data->request;
   unsigned short request_length = strnlen(request, 0xFFFF);

   espconn_set_opt(connection, ESPCONN_REUSEADDR);
   // Keep-Alive timeout doesn't work yet
   //espconn_set_opt(connection, ESPCONN_KEEPALIVE); // ESPCONN_REUSEADDR |
   //uint32 espconn_keepidle_value = 5; // seconds
   //unsigned char keepalive_error_code = espconn_set_keepalive(connection, ESPCONN_KEEPIDLE, &espconn_keepidle_value);
   //uint32 espconn_keepintvl_value = 2; // seconds
   // If there is no response, retry ESPCONN_KEEPCNT times every ESPCONN_KEEPINTVL
   //keepalive_error_code |= espconn_set_keepalive(connection, ESPCONN_KEEPINTVL, &espconn_keepintvl_value);
   //uint32 espconn_keepcnt_value = 2; // count
   //keepalive_error_code |= espconn_set_keepalive(connection, ESPCONN_KEEPCNT, &espconn_keepcnt_value);

   int sent_status = espconn_send(connection, request, request_length);
   free(request);
   user_data->request = NULL;

   if (sent_status != 0) {
      void (*execute_on_error)(struct espconn *connection) = user_data->execute_on_error;
      if (execute_on_error) {
         execute_on_error(connection);
      }
   }
}

void successfull_disconnected_tcp_handler_callback(void *arg) {
   struct espconn *connection = arg;
   struct connection_user_data *user_data = connection->reserve;
   bool response_received = user_data->response_received;

   #ifdef ALLOW_USE_PRINTF
   printf("Disconnected callback beginning. Response %s received. Time: %u\n", response_received ? "has been" : "has not been", milliseconds_g);
   #endif

   void (*execute_on_succeed)(struct espconn *connection) = user_data->execute_on_succeed;
   if (execute_on_succeed) {
      execute_on_succeed(connection);
   }
}

void tcp_connection_error_handler_callback(void *arg, sint8 err) {
   #ifdef ALLOW_USE_PRINTF
   printf("Connection error callback. Error code: %d. Time: %u\n", err, milliseconds_g);
   #endif

   struct espconn *connection = arg;
   struct connection_user_data *user_data = connection->reserve;
   void (*execute_on_error)(struct espconn *connection) = user_data->execute_on_error;
   if (execute_on_error) {
      execute_on_error(connection);
   }
}

void tcp_response_received_handler_callback(void *arg, char *pdata, unsigned short len) {
   struct espconn *connection = arg;
   struct connection_user_data *user_data = connection->reserve;
   bool response_received = user_data->response_received;

   if (!response_received) {
      char *server_sent = get_string_from_rom(RESPONSE_SERVER_SENT_OK);

      if (strstr(pdata, server_sent)) {
         user_data->response_received = true;
         char *response = malloc(len);
         memcpy(response, pdata, len);
         user_data->response = response;

         #ifdef ALLOW_USE_PRINTF
         printf("Response received: %sTime: %u\n", pdata, milliseconds_g);
         #endif
      }
      free(server_sent);
   }

   // Don't call this API in any espconn callback. If needed, please use system task to trigger espconn_disconnect.
   //espconn_disconnect(connection);
}

void tcp_request_successfully_sent_handler_callback() {
   //printf("Request sent callback\n");
}

void tcp_request_successfully_written_into_buffer_handler_callback() {
   //printf("Request written into buffer callback\n");
}

void status_request_on_error_callback(struct espconn *connection) {
   #ifdef ALLOW_USE_PRINTF
   printf("status_request_on_error_callback. Time: %u\n", milliseconds_g);
   #endif

   struct connection_user_data *user_data = connection->reserve;
   char *request = user_data->request;

   errors_counter_g++;
   pin_output_reset(SERVER_AVAILABILITY_STATUS_LED_PIN);
   set_flag(&general_flags, REQUEST_ERROR_OCCURRED_FLAG);
   xSemaphoreHandle semaphores_to_give[] = {status_request_semaphore_g, requests_mutex_g, NULL};
   request_finish_action(connection, semaphores_to_give);
}

void general_request_on_error_callback(struct espconn *connection) {
   #ifdef ALLOW_USE_PRINTF
   printf("general_request_on_error_callback. Time: %u\n", milliseconds_g);
   #endif

   struct connection_user_data *user_data = connection->reserve;
   char *request = user_data->request;

   errors_counter_g++;
   pin_output_reset(SERVER_AVAILABILITY_STATUS_LED_PIN);
   set_flag(&general_flags, REQUEST_ERROR_OCCURRED_FLAG);
   xSemaphoreHandle semaphores_to_give[] = {requests_mutex_g, NULL};
   request_finish_action(connection, semaphores_to_give);
}

void check_for_update_firmware(char *response) {
   char *update_firmware_json_element = get_string_from_rom(UPDATE_FIRMWARE);

   if (strstr(response, update_firmware_json_element)) {
      set_flag(&general_flags, UPDATE_FIRMWARE_FLAG);
   }
   free(update_firmware_json_element);
}

void status_request_on_succeed_callback(struct espconn *connection) {
   #ifdef ALLOW_USE_PRINTF
   printf("status_request_on_succeed_callback, Time: %u\n", milliseconds_g);
   #endif

   struct connection_user_data *user_data = connection->reserve;

   if (!user_data->response_received && user_data->execute_on_error) {
      user_data->execute_on_error(connection);
      return;
   }

   check_for_update_firmware(user_data->response);

   pin_output_set(SERVER_AVAILABILITY_STATUS_LED_PIN);
   xSemaphoreHandle semaphores_to_give[] = {requests_mutex_g, NULL};
   request_finish_action(connection, semaphores_to_give);

   xTaskCreate(activate_status_requests_task_task, "activate_status_requests_task_task", 256, NULL, 1, NULL);

   if (read_flag(general_flags, UPDATE_FIRMWARE_FLAG)) {
      upgrade_firmware();
   }
}

void general_request_on_succeed_callback(struct espconn *connection) {
   #ifdef ALLOW_USE_PRINTF
   printf("general_request_on_succeed_callback. Time: %u\n", milliseconds_g);
   #endif

   struct connection_user_data *user_data = connection->reserve;
   xTaskHandle parent_task = user_data->parent_task;

   if (!user_data->response_received && user_data->execute_on_error) {
      user_data->execute_on_error(connection);
      return;
   }

   check_for_update_firmware(user_data->response);

   pin_output_set(SERVER_AVAILABILITY_STATUS_LED_PIN);

   if (parent_task) {
      #ifdef ALLOW_USE_PRINTF
      printf("parent task is to be deleted...\n");
      #endif

      vTaskDelete(parent_task);
   }

   if (buzzer_semaphore_g != NULL) {
      xSemaphoreGive(buzzer_semaphore_g);
   }

   xSemaphoreHandle semaphores_to_give[] = {requests_mutex_g, NULL};
   request_finish_action(connection, semaphores_to_give);
}

void request_finish_action(struct espconn *connection, xSemaphoreHandle semaphores_to_give[]) {
   struct connection_user_data *user_data = connection->reserve;
   char *request = user_data->request;

   if (request != NULL) {
      free(request);
      user_data->request = NULL;
   }
   if (user_data->response != NULL) {
      free(user_data->response);
      user_data->response = NULL;
   }

   if (user_data->timeout_request_supervisor_task != NULL) {
      vTaskDelete(user_data->timeout_request_supervisor_task);

      #ifdef ALLOW_USE_PRINTF
      printf("timeout_request_supervisor_task still exists\n");
      #endif
   }
   free(user_data);

   char error_code = espconn_delete(connection);
   if (error_code != 0) {
      #ifdef ALLOW_USE_PRINTF
      printf("ERROR! Connection is still in progress\n");
      #endif
   }
   free(connection);

   if (semaphores_to_give) {
      unsigned char i;
      for (i = 0; semaphores_to_give[i] != NULL; i++) {
         xSemaphoreHandle semaphore_to_give = semaphores_to_give[i];
         xSemaphoreGive(semaphore_to_give);
      }
   }
}

bool compare_pins_names(char *activated_pin, char *rom_pin_name) {
   char *comparable_pin = get_string_from_rom(rom_pin_name);
   bool names_matched = strcmp(activated_pin, comparable_pin) == 0;

   free(comparable_pin);
   return names_matched;
}

void fill_motion_sensor_request_data(struct request_data *request_data, GeneralRequestType request_type, MotionSensorUnit msu, char *motion_sensor_pin) {
   struct motion_sensor *ms = (struct motion_sensor *) zalloc(sizeof(struct motion_sensor));

   ms->unit = msu;
   ms->alarm_source = motion_sensor_pin;

   request_data->ms = ms;
   request_data->request_type = request_type;
}

void input_pins_analyzer_task(void *pvParameters) {
   char *activated_pin_with_prefix = pvParameters;
   char *prefix = "pin:";
   unsigned char prefix_length = strnlen(prefix, 0xFF);
   char *found_pin_prefix = strstr(activated_pin_with_prefix, prefix);

   if (found_pin_prefix == NULL) {
      #ifdef ALLOW_USE_PRINTF
      printf("pin prefix is not found\n");
      #endif
      free(activated_pin_with_prefix);
      vTaskDelete(NULL);
   }

   unsigned char pin_name_length = 0;
   while (*(activated_pin_with_prefix + prefix_length + pin_name_length) != '\0') {
      pin_name_length++;
   }

   char *activated_pin = malloc(pin_name_length);
   unsigned char i = 0;

   for (i = 0; i < pin_name_length; i++) {
      *(activated_pin + i) = *(activated_pin_with_prefix + prefix_length + i);
   }
   *(activated_pin + i) = '\0';
   free(activated_pin_with_prefix);

   #ifdef ALLOW_USE_PRINTF
   printf("pin without prefix: %s\n", activated_pin);
   #endif

   struct request_data *request_data_param = (struct request_data *) zalloc(sizeof(struct request_data));

   if (compare_pins_names(activated_pin, MOTION_SENSOR_1_PIN)) {
      fill_motion_sensor_request_data(request_data_param, ALARM, MOTION_SENSOR_1, activated_pin);
      send_general_request(request_data_param, 2);
   } else if (compare_pins_names(activated_pin, MOTION_SENSOR_2_PIN)) {
      fill_motion_sensor_request_data(request_data_param, ALARM, MOTION_SENSOR_2, activated_pin);
      send_general_request(request_data_param, 2);
   } else if (compare_pins_names(activated_pin, MOTION_SENSOR_3_PIN)) {
      fill_motion_sensor_request_data(request_data_param, ALARM, MOTION_SENSOR_3, activated_pin);
      send_general_request(request_data_param, 2);
   } else if (compare_pins_names(activated_pin, PIR_LED_1_PIN) || compare_pins_names(activated_pin, MW_LED_1_PIN)) {
      fill_motion_sensor_request_data(request_data_param, FALSE_ALARM, MOTION_SENSOR_1, activated_pin);
      send_general_request(request_data_param, 1);
   } else if (compare_pins_names(activated_pin, PIR_LED_3_PIN) || compare_pins_names(activated_pin, MW_LED_3_PIN)) {
      fill_motion_sensor_request_data(request_data_param, FALSE_ALARM, MOTION_SENSOR_3, activated_pin);
      send_general_request(request_data_param, 1);
   } else if (compare_pins_names(activated_pin, IMMOBILIZER_LED_PIN)) {
      request_data_param->request_type = IMMOBILIZER_ACTIVATION;
      send_general_request(request_data_param, 1);
   } else {
      free(activated_pin);
      free(request_data_param);
   }

   vTaskDelete(NULL);
}

void timeout_request_supervisor_task(void *pvParameters) {
   struct espconn *connection = pvParameters;
   struct connection_user_data *user_data = connection->reserve;

   vTaskDelay(user_data->request_max_duration_time);

   #ifdef ALLOW_USE_PRINTF
   printf("Request timeout. Time: %u\n", milliseconds_g);
   #endif

   if (connection->state == ESPCONN_CONNECT) {
      #ifdef ALLOW_USE_PRINTF
      printf("Was connected\n");
      #endif

      espconn_disconnect(connection);
   } else {
      #ifdef ALLOW_USE_PRINTF
      printf("Some another connection timeout error\n");
      #endif

      // To not delete this task in other functions
      user_data->timeout_request_supervisor_task = NULL;

      void (*execute_on_error)(struct espconn *connection) = user_data->execute_on_error;
      if (execute_on_error != NULL) {
         execute_on_error(connection);
      }
   }

   vTaskDelete(NULL);
}

void ota_finished_callback(void *arg) {
   struct upgrade_server_info *update = arg;

   if (update->upgrade_flag == true) {
      #ifdef ALLOW_USE_PRINTF
      printf("[OTA] success; rebooting! Time: %u\n", milliseconds_g);
      #endif

      system_upgrade_flag_set(UPGRADE_FLAG_FINISH);
      system_upgrade_reboot();
   } else {
      #ifdef ALLOW_USE_PRINTF
      printf("[OTA] failed! Time: %u\n", milliseconds_g);
      #endif

      system_restart();
   }

   free(&update->sockaddrin);
   free(update->url);
   free(update);
}

void blink_leds_while_updating_task(void *pvParameters) {
   for (;;) {
      if (read_output_pin_state(AP_CONNECTION_STATUS_LED_PIN)) {
         pin_output_reset(AP_CONNECTION_STATUS_LED_PIN);
         pin_output_set(SERVER_AVAILABILITY_STATUS_LED_PIN);
      } else {
         pin_output_set(AP_CONNECTION_STATUS_LED_PIN);
         pin_output_reset(SERVER_AVAILABILITY_STATUS_LED_PIN);
      }

      vTaskDelay(100 / portTICK_RATE_MS);
   }
}

void upgrade_firmware() {
   #ifdef ALLOW_USE_PRINTF
   printf("\nUpdating firmware... Time: %u\n", milliseconds_g);
   #endif

   turn_motion_sensors_off();
   ETS_UART_INTR_DISABLE(); // To not receive data from UART RX

   xTaskCreate(blink_leds_while_updating_task, "blink_leds_while_updating_task", 256, NULL, 1, NULL);

   struct upgrade_server_info *upgrade_server = (struct upgrade_server_info *) zalloc(sizeof(struct upgrade_server_info));
   struct sockaddr_in *sockaddrin = (struct sockaddr_in *) zalloc(sizeof(struct sockaddr_in));

   upgrade_server->sockaddrin = *sockaddrin;
   upgrade_server->sockaddrin.sin_family = AF_INET;
   struct in_addr sin_addr;
   char *server_ip = get_string_from_rom(SERVER_IP_ADDRESS);
   sin_addr.s_addr = inet_addr(server_ip);
   upgrade_server->sockaddrin.sin_addr = sin_addr;
   upgrade_server->sockaddrin.sin_port = htons(SERVER_PORT);
   upgrade_server->sockaddrin.sin_len = sizeof(upgrade_server->sockaddrin);
   upgrade_server->check_cb = ota_finished_callback;
   upgrade_server->check_times = 10;

   char *url_pattern = get_string_from_rom(FIRMWARE_UPDATE_GET_REQUEST);
   unsigned char user_bin = system_upgrade_userbin_check();
   char *file_to_download = user_bin == UPGRADE_FW_BIN1 ? "user2.bin" : "user1.bin";
   char *url_parameters[] = {file_to_download, server_ip, NULL};
   char *url = set_string_parameters(url_pattern, url_parameters);

   free(url_pattern);
   free(server_ip);
   upgrade_server->url = url;
   system_upgrade_start(upgrade_server);
}

void establish_connection(struct espconn *connection) {
   if (!connection) {
      #ifdef ALLOW_USE_PRINTF
      printf("Create connection first\n");
      #endif

      return;
   }

   int connection_status = espconn_connect(connection);

   #ifdef ALLOW_USE_PRINTF
   printf("Connection status: ");
   #endif

   switch (connection_status) {
      case ESPCONN_OK:
      {
         struct connection_user_data *user_data = connection->reserve;

         if (user_data && user_data->timeout_request_supervisor_task) {
            xTaskCreate(timeout_request_supervisor_task, "timeout_request_supervisor_task", 256, connection, 1, user_data->timeout_request_supervisor_task);
         }

         #ifdef ALLOW_USE_PRINTF
         printf("Connected\n");
         #endif

         break;
      }
      case ESPCONN_RTE:
         #ifdef ALLOW_USE_PRINTF
         printf("Routing problem\n");
         #endif

         break;
      case ESPCONN_MEM:
         #ifdef ALLOW_USE_PRINTF
         printf("Out of memory\n");
         #endif

         break;
      case ESPCONN_ISCONN:
         #ifdef ALLOW_USE_PRINTF
         printf("Already connected\n");
         #endif

         break;
      case ESPCONN_ARG:
         #ifdef ALLOW_USE_PRINTF
         printf("Illegal argument\n");
         #endif

         break;
   }

   if (connection_status != ESPCONN_OK) {
      struct connection_user_data *user_data = connection->reserve;

      if (user_data && user_data->execute_on_error) {
         user_data->execute_on_error(connection);
      }
   }
}

void activate_status_requests_task_task(void *pvParameters) {
   vTaskDelay(STATUS_REQUESTS_SEND_INTERVAL);
   xSemaphoreGive(status_request_semaphore_g);
   vTaskDelete(NULL);
}

void send_status_requests_task(void *pvParameters) {
   #ifdef ALLOW_USE_PRINTF
   printf("send_status_requests_task has been created\n");
   #endif
   vTaskDelay(5000 / portTICK_RATE_MS);

   for (;;) {
      xSemaphoreTake(status_request_semaphore_g, portMAX_DELAY);
      xSemaphoreTake(requests_mutex_g, portMAX_DELAY);

      if (read_flag(general_flags, UPDATE_FIRMWARE_FLAG)) {
         vTaskDelete(NULL);
      }

      #ifdef ALLOW_USE_PRINTF
      printf("send_status_requests_task started. Time: %u\n", milliseconds_g);
      #endif

      if (!read_output_pin_state(AP_CONNECTION_STATUS_LED_PIN)) {
         #ifdef ALLOW_USE_PRINTF
         printf("Can't send status request, because not connected to AP. Time: %u\n", milliseconds_g);
         #endif

         xSemaphoreGive(requests_mutex_g);
         xSemaphoreGive(status_request_semaphore_g);
         vTaskDelay(2000 / portTICK_RATE_MS);
         continue;
      }

      if (read_flag(general_flags, REQUEST_ERROR_OCCURRED_FLAG)) {
         reset_flag(&general_flags, REQUEST_ERROR_OCCURRED_FLAG);

         xSemaphoreGive(requests_mutex_g);
         xSemaphoreGive(status_request_semaphore_g);
         vTaskDelay(REQUEST_IDLE_TIME_ON_ERROR);
         continue;
      }

      char signal_strength[4];
      sprintf(signal_strength, "%d", signal_strength_g);
      char *device_name = get_string_from_rom(DEVICE_NAME);
      char errors_counter[5];
      sprintf(errors_counter, "%d", errors_counter_g);
      char build_timestamp[30];
      sprintf(build_timestamp, "%s", __TIMESTAMP__);
      char *status_info_request_payload_template_parameters[] = {signal_strength, device_name, errors_counter, build_timestamp, NULL};
      char *status_info_request_payload_template = get_string_from_rom(STATUS_INFO_REQUEST_PAYLOAD);
      char *request_payload = set_string_parameters(status_info_request_payload_template, status_info_request_payload_template_parameters);

      free(device_name);
      free(status_info_request_payload_template);

      char *request_template = get_string_from_rom(STATUS_INFO_POST_REQUEST);
      unsigned short request_payload_length = strnlen(request_payload, 0xFFFF);
      char request_payload_length_string[3];
      sprintf(request_payload_length_string, "%d", request_payload_length);
      char *server_ip_address = get_string_from_rom(SERVER_IP_ADDRESS);
      char *request_template_parameters[] = {request_payload_length_string, server_ip_address, request_payload, NULL};
      char *request = set_string_parameters(request_template, request_template_parameters);

      free(request_payload);
      free(request_template);
      free(server_ip_address);

      #ifdef ALLOW_USE_PRINTF
      printf("Request created:\n<<<\n%s>>>\n", request);
      #endif

      struct espconn *connection = (struct espconn *) zalloc(sizeof(struct espconn));
      struct connection_user_data *user_data = (struct connection_user_data *) zalloc(sizeof(struct connection_user_data));

      user_data->response_received = false;
      user_data->timeout_request_supervisor_task = NULL;
      user_data->request = request;
      user_data->response = NULL;
      user_data->execute_on_succeed = status_request_on_succeed_callback;
      user_data->execute_on_error = status_request_on_error_callback;
      user_data->request_max_duration_time = REQUEST_MAX_DURATION_TIME;
      connection->reserve = user_data;
      connection->type = ESPCONN_TCP;
      connection->state = ESPCONN_NONE;

      // remote IP of TCP server
      unsigned char tcp_server_ip[] = {SERVER_IP_ADDRESS_1, SERVER_IP_ADDRESS_2, SERVER_IP_ADDRESS_3, SERVER_IP_ADDRESS_4};

      connection->proto.tcp = &user_tcp;
      memcpy(&connection->proto.tcp->remote_ip, tcp_server_ip, 4);
      connection->proto.tcp->remote_port = SERVER_PORT;
      connection->proto.tcp->local_port = espconn_port(); // local port of ESP8266

      espconn_regist_connectcb(connection, successfull_connected_tcp_handler_callback);
      espconn_regist_disconcb(connection, successfull_disconnected_tcp_handler_callback);
      espconn_regist_reconcb(connection, tcp_connection_error_handler_callback);
      espconn_regist_sentcb(connection, tcp_request_successfully_sent_handler_callback);
      espconn_regist_recvcb(connection, tcp_response_received_handler_callback);
      //espconn_regist_write_finish(&connection, tcp_request_successfully_written_into_buffer_handler_callback);

      establish_connection(connection);
   }
}

bool is_alarm_being_ignored(struct motion_sensor *ms, GeneralRequestType request_type) {
   if (ms == NULL) {
      return false;
   }

   unsigned char i;

   if (request_type == FALSE_ALARM) {
      for (i = 0; i < ALARM_SOURCES_AMOUNT; i++) {
         struct motion_sensor *found_motion_sensor = alarm_sources_g[i];

         if (found_motion_sensor->unit == ms->unit) {
            // Because of this "false alarm" will be ignored even when "alarm" is being ignored if "alarm" timeout is longer than "false alarm" timeout
            return true;
         }
     }
   } else if (request_type == ALARM) {
      for (i = 0; i < ALARM_SOURCES_AMOUNT; i++) {
         struct motion_sensor *found_motion_sensor = alarm_sources_g[i];

         if (found_motion_sensor->unit == ms->unit && strcmp(found_motion_sensor->alarm_source, ms->alarm_source) == 0) {
            return true;
         }
     }
   }
   return false;
}

void ignore_alarm(struct motion_sensor *ms, unsigned int timeout_ms) {
   unsigned char i;

   for (i = 0; i < ALARM_SOURCES_AMOUNT; i++) {
      if (alarm_sources_g[i] == NULL) {
         alarm_sources_g[i] = ms;
         break;
      }
   }

   for (i = 0; i < ALARM_TIMERS_MAX_AMOUNT; i++) {
      if (alarm_timers[i] == NULL) {
         xTimerHandle created_timer = xTimerCreate(NULL, timeout_ms/ portTICK_RATE_MS, pdFALSE, ms, stop_ignoring_alarm);
         alarm_timers[i] = created_timer;
         break;
      }
   }
}

void stop_ignoring_alarm(xTimerHandle xTimer) {
   struct motion_sensor *ms = pvTimerGetTimerID(xTimer);

   if (ms == NULL) {
      return;
   }

   unsigned char i;
   for (i = 0; i < ALARM_SOURCES_AMOUNT; i++) {
      struct motion_sensor *found_motion_sensor = alarm_sources_g[i];

      if (found_motion_sensor == ms) {
         free(found_motion_sensor->alarm_source);
         free(found_motion_sensor);
         alarm_sources_g[i] = NULL;
         break;
      }
   }

   for (i = 0; i < ALARM_TIMERS_MAX_AMOUNT; i++) {
      if (alarm_timers[i] == xTimer) {
         alarm_timers[i] = NULL;
         xTimerDelete(xTimer, 0);
         break;
      }
   }
}

void send_general_request(struct request_data *request_data_param, unsigned char task_priority) {
   if (read_flag(general_flags, UPDATE_FIRMWARE_FLAG)) {
      if (request_data_param->ms != NULL) {
         free(request_data_param->ms->alarm_source);
         free(request_data_param->ms);
      }
      free(request_data_param);
      return;
   }

   if ((request_data_param->request_type == FALSE_ALARM || request_data_param->request_type == ALARM) &&
         is_alarm_being_ignored(request_data_param->ms, request_data_param->request_type)) {
      #ifdef ALLOW_USE_PRINTF
      printf("%s alarm is being ignored. Time: %u\n", request_data_param->ms->alarm_source, milliseconds_g);
      #endif

      free(request_data_param->ms->alarm_source);
      free(request_data_param->ms);
      free(request_data_param);
      return;
   }

   if (request_data_param->request_type == IMMOBILIZER_ACTIVATION) {
      xTaskCreate(beep_task, "beep_task", 176, NULL, 1, NULL);
   }

   xTaskCreate(send_general_request_task, "send_general_request_task", 256, request_data_param, task_priority, NULL);
}

void send_general_request_task(void *pvParameters) {
   #ifdef ALLOW_USE_PRINTF
   printf("send_general_request_task has been created. Time: %u\n", milliseconds_g);
   #endif

   struct request_data *request_data_param = pvParameters;

   if (request_data_param->request_type == ALARM) {
      ignore_alarm(request_data_param->ms, IGNORE_ALARMS_TIMEOUT_SEC * 1000);
   } else if (request_data_param->request_type == FALSE_ALARM) {
      vTaskDelay((IGNORE_FALSE_ALARMS_TIMEOUT_SEC * 1000 + 500) / portTICK_RATE_MS);

      if (is_alarm_being_ignored(request_data_param->ms, request_data_param->request_type)) {
         #ifdef ALLOW_USE_PRINTF
         printf("%s is being ignored after timeout. Time: %u\n", request_data_param->ms->alarm_source, milliseconds_g);
         #endif

         // Alarm occurred after false alarm within interval
         free(request_data_param->ms->alarm_source);
         free(request_data_param->ms);
         free(request_data_param);
         vTaskDelete(NULL);
      }

      ignore_alarm(request_data_param->ms, IGNORE_FALSE_ALARMS_TIMEOUT_SEC * 1000);
   }

   GeneralRequestType request_type = request_data_param->request_type;
   unsigned char alarm_source_length = (request_type == ALARM || request_type == FALSE_ALARM) ? (strlen(request_data_param->ms->alarm_source) + 1) : 0;
   char alarm_source[alarm_source_length];

   memcpy(alarm_source, request_data_param->ms->alarm_source, alarm_source_length);

   for (;;) {
      xSemaphoreTake(requests_mutex_g, portMAX_DELAY);

      #ifdef ALLOW_USE_PRINTF
      printf("send_general_request_task started. Time: %u\n", milliseconds_g);
      #endif

      if (!read_output_pin_state(AP_CONNECTION_STATUS_LED_PIN)) {
         #ifdef ALLOW_USE_PRINTF
         printf("Can't send alarm request, because not connected to AP. Time: %u\n", milliseconds_g);
         #endif

         vTaskDelay(2000 / portTICK_RATE_MS);
         continue;
      }

      if (read_flag(general_flags, REQUEST_ERROR_OCCURRED_FLAG)) {
         reset_flag(&general_flags, REQUEST_ERROR_OCCURRED_FLAG);

         vTaskDelay(REQUEST_IDLE_TIME_ON_ERROR);
      }

      char *server_ip_address = get_string_from_rom(SERVER_IP_ADDRESS);
      char *request_template;
      char *request;

      if (request_type == ALARM) {
         request_template = get_string_from_rom(ALARM_GET_REQUEST);
         char *request_template_parameters[] = {alarm_source, server_ip_address, NULL};
         request = set_string_parameters(request_template, request_template_parameters);
      } else if (request_type == FALSE_ALARM) {
         request_template = get_string_from_rom(FALSE_ALARM_GET_REQUEST);
         char *request_template_parameters[] = {alarm_source, server_ip_address, NULL};
         request = set_string_parameters(request_template, request_template_parameters);
      } else if (request_type == IMMOBILIZER_ACTIVATION) {
         request_template = get_string_from_rom(IMMOBILIZER_ACTIVATION_REQUEST);
         char *request_template_parameters[] = {server_ip_address, NULL};
         request = set_string_parameters(request_template, request_template_parameters);
      }

      free(request_template);
      free(server_ip_address);

      #ifdef ALLOW_USE_PRINTF
      printf("Request created:\n<<<\n%s>>>\n", request);
      #endif

      struct espconn *connection = (struct espconn *) zalloc(sizeof(struct espconn));
      struct connection_user_data *user_data = (struct connection_user_data *) zalloc(sizeof(struct connection_user_data));

      user_data->response_received = false;
      user_data->timeout_request_supervisor_task = NULL;
      user_data->request = request;
      user_data->response = NULL;
      user_data->execute_on_succeed = general_request_on_succeed_callback;
      user_data->execute_on_error = general_request_on_error_callback;
      user_data->parent_task = xTaskGetCurrentTaskHandle();
      user_data->request_max_duration_time = REQUEST_MAX_DURATION_TIME;
      connection->reserve = user_data;
      connection->type = ESPCONN_TCP;
      connection->state = ESPCONN_NONE;

      // remote IP of TCP server
      unsigned char tcp_server_ip[] = {SERVER_IP_ADDRESS_1, SERVER_IP_ADDRESS_2, SERVER_IP_ADDRESS_3, SERVER_IP_ADDRESS_4};

      connection->proto.tcp = &user_tcp;
      memcpy(&connection->proto.tcp->remote_ip, tcp_server_ip, 4);
      connection->proto.tcp->remote_port = SERVER_PORT;
      connection->proto.tcp->local_port = espconn_port(); // local port of ESP8266

      espconn_regist_connectcb(connection, successfull_connected_tcp_handler_callback);
      espconn_regist_disconcb(connection, successfull_disconnected_tcp_handler_callback);
      espconn_regist_reconcb(connection, tcp_connection_error_handler_callback);
      espconn_regist_sentcb(connection, tcp_request_successfully_sent_handler_callback);
      espconn_regist_recvcb(connection, tcp_response_received_handler_callback);
      //espconn_regist_write_finish(&connection, tcp_request_successfully_written_into_buffer_handler_callback);

      establish_connection(connection);
   }
}

void wifi_event_handler_callback(System_Event_t *event) {
   switch (event->event_id) {
      case EVENT_STAMODE_CONNECTED:
         pin_output_set(AP_CONNECTION_STATUS_LED_PIN);
         turn_motion_sensors_on();
         break;
      case EVENT_STAMODE_DISCONNECTED:
         pin_output_reset(AP_CONNECTION_STATUS_LED_PIN);
         pin_output_reset(SERVER_AVAILABILITY_STATUS_LED_PIN);
         break;
   }
}

void set_default_wi_fi_settings() {
   wifi_station_set_auto_connect(false);
   wifi_station_set_reconnect_policy(false);
   wifi_station_dhcpc_stop();
   wifi_set_opmode(STATION_MODE);

   STATION_STATUS station_status = wifi_station_get_connect_status();
   if (station_status == STATION_GOT_IP) {
      wifi_station_disconnect();
   }

   struct station_config station_config_settings;

   wifi_station_get_config_default(&station_config_settings);

   char *default_access_point_name = get_string_from_rom(ACCESS_POINT_NAME);
   char *default_access_point_password = get_string_from_rom(ACCESS_POINT_PASSWORD);

   if (strncmp(default_access_point_name, station_config_settings.ssid, 32) != 0
         || strncmp(default_access_point_password, station_config_settings.password, 64) != 0) {
      struct station_config station_config_settings_to_save;

      memcpy(&station_config_settings_to_save.ssid, default_access_point_name, 32);
      memcpy(&station_config_settings_to_save.password, default_access_point_password, 64);
      wifi_station_set_config(&station_config_settings_to_save);
   }
   free(default_access_point_name);
   free(default_access_point_password);

   struct ip_info current_ip_info;
   wifi_get_ip_info(STATION_IF, &current_ip_info);
   char *current_ip = ipaddr_ntoa(&current_ip_info.ip);
   char *own_ip_address = get_string_from_rom(OWN_IP_ADDRESS);

   if (strncmp(current_ip, own_ip_address, 15) != 0) {
      char *own_netmask = get_string_from_rom(OWN_NETMASK);
      char *own_getaway_address = get_string_from_rom(OWN_GETAWAY_ADDRESS);
      struct ip_info ip_info_to_set;

      ip_info_to_set.ip.addr = ipaddr_addr(own_ip_address);
      ip_info_to_set.netmask.addr = ipaddr_addr(own_netmask);
      ip_info_to_set.gw.addr = ipaddr_addr(own_getaway_address);
      wifi_set_ip_info(STATION_IF, &ip_info_to_set);
      free(own_netmask);
      free(own_getaway_address);
   }
   free(current_ip);
   free(own_ip_address);
}

void pins_config() {
   GPIO_ConfigTypeDef output_pins;
   output_pins.GPIO_Mode = GPIO_Mode_Output;
   output_pins.GPIO_Pin = AP_CONNECTION_STATUS_LED_PIN | SERVER_AVAILABILITY_STATUS_LED_PIN | BUZZER_PIN |
         MOTION_SENSOR_1_ENABLE_PIN | MOTION_SENSOR_2_ENABLE_PIN | MOTION_SENSOR_3_ENABLE_PIN;
   pin_output_reset(AP_CONNECTION_STATUS_LED_PIN);
   pin_output_reset(SERVER_AVAILABILITY_STATUS_LED_PIN);
   pin_output_reset(BUZZER_PIN);

   gpio_config(&output_pins);
}

void uart_config() {
   UART_WaitTxFifoEmpty(UART0);

   UART_ConfigTypeDef uart_config;
   uart_config.baud_rate         = 115200;
   uart_config.data_bits         = UART_WordLength_8b;
   uart_config.parity            = USART_Parity_None;
   uart_config.stop_bits         = USART_StopBits_1;
   uart_config.flow_ctrl         = USART_HardwareFlowControl_None;
   uart_config.UART_RxFlowThresh = 120;
   uart_config.UART_InverseMask  = UART_None_Inverse;
   UART_ParamConfig(UART0, &uart_config);

   UART_IntrConfTypeDef uart_intr;
   uart_intr.UART_IntrEnMask = UART_RXFIFO_TOUT_INT_ENA | UART_FRM_ERR_INT_ENA | UART_RXFIFO_FULL_INT_ENA;
   uart_intr.UART_RX_FifoFullIntrThresh = 30;
   uart_intr.UART_RX_TimeOutIntrThresh = 2;
   uart_intr.UART_TX_FifoEmptyIntrThresh = 20;
   UART_IntrConfig(UART0, &uart_intr);

   UART_SetPrintPort(UART0);
   UART_intr_handler_register(uart_rx_intr_handler, NULL);
   ETS_UART_INTR_ENABLE();
}

void uart_rx_intr_handler(void *params) {
   char received_character;

   unsigned int uart_intr_status = READ_PERI_REG(UART_INT_ST(UART0));

   while (uart_intr_status != 0x0) {
      if (UART_FRM_ERR_INT_ST == (uart_intr_status & UART_FRM_ERR_INT_ST)) {
         WRITE_PERI_REG(UART_INT_CLR(UART0), UART_FRM_ERR_INT_CLR);
      } else if (UART_RXFIFO_FULL_INT_ST == (uart_intr_status & UART_RXFIFO_FULL_INT_ST)) {
         unsigned char buf_idx = 0;
         unsigned char fifo_len = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT;

         while (buf_idx < fifo_len) {
            received_character = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
            buf_idx++;
         }

         WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR);
      } else if (UART_RXFIFO_TOUT_INT_ST == (uart_intr_status & UART_RXFIFO_TOUT_INT_ST)) {
         unsigned char buf_idx = 0;
         unsigned char fifo_len = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT;
         char *received_data = malloc(fifo_len + 1);

         while (buf_idx < fifo_len) {
            received_character = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
            *(received_data + buf_idx) = received_character;
            buf_idx++;

            if (buf_idx >= UART_RX_BUFFER_SIZE) {
               buf_idx = 0;
            }
         }
         *(received_data + fifo_len) = '\0';

         #ifdef ALLOW_USE_PRINTF
         printf("Received pin info: %s\n", received_data);
         #endif
         xTaskCreate(input_pins_analyzer_task, "input_pins_analyzer_task", 256, received_data, 1, NULL);

         WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_TOUT_INT_CLR);
      } else {
         //skip
      }

      uart_intr_status = READ_PERI_REG(UART_INT_ST(UART0));
   }
}

void turn_motion_sensors_on() {
   pin_output_reset(MOTION_SENSOR_1_ENABLE_PIN);
   pin_output_reset(MOTION_SENSOR_2_ENABLE_PIN);
   pin_output_reset(MOTION_SENSOR_3_ENABLE_PIN);
}

void turn_motion_sensors_off() {
   pin_output_set(MOTION_SENSOR_1_ENABLE_PIN);
   pin_output_set(MOTION_SENSOR_2_ENABLE_PIN);
   pin_output_set(MOTION_SENSOR_3_ENABLE_PIN);
}

bool are_motion_sensors_turned_on() {
   return read_output_pin_state(MOTION_SENSOR_1_ENABLE_PIN) && read_output_pin_state(MOTION_SENSOR_2_ENABLE_PIN) && read_output_pin_state(MOTION_SENSOR_3_ENABLE_PIN);
}

void user_init(void) {
   vTaskDelay(5000 / portTICK_RATE_MS);

   pins_config();
   uart_config();

   #ifdef ALLOW_USE_PRINTF
   printf("\nSoftware is running from: %s\n", system_upgrade_userbin_check() ? "user2.bin" : "user1.bin");
   #endif

   wifi_set_event_handler_cb(wifi_event_handler_callback);
   set_default_wi_fi_settings();
   espconn_init();

   xTaskCreate(autoconnect_task, "autoconnect_task", 256, NULL, 1, NULL);
   xTaskCreate(scan_access_point_task, "scan_access_point_task", 256, NULL, 1, NULL);

   vSemaphoreCreateBinary(status_request_semaphore_g);
   xSemaphoreGive(status_request_semaphore_g);
   requests_mutex_g = xSemaphoreCreateMutex();

   xTaskCreate(send_status_requests_task, "send_status_requests_task", 256, NULL, 1, NULL);

   #ifdef ALLOW_USE_PRINTF
   start_100millisecons_counter();
   #endif
}
