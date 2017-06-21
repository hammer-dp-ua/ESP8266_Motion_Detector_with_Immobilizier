#ifndef MAIN_HEADER
#define MAIN_HEADER

#define enable_pins_interrupt()  _xt_isr_unmask(1<<ETS_GPIO_INUM)
#define disable_pins_interrupt() _xt_isr_mask(1<<ETS_GPIO_INUM)

#define AP_CONNECTION_STATUS_LED_PIN         GPIO_Pin_5
#define SERVER_AVAILABILITY_STATUS_LED_PIN   GPIO_Pin_4
#define BUZZER_PIN                           GPIO_Pin_2
#define MOTION_SENSORS_ENABLE_PIN            GPIO_Pin_12

#ifndef true // needed only for Eclipse
   typedef unsigned char bool;
   #define true 1
   #define false 0
#endif

#define LONG_POLLING_REQUEST_ERROR_OCCURRED_FLAG   1
#define SERVER_IS_AVAILABLE_FLAG                   2
#define UPDATE_FIRMWARE_FLAG                       4
#define REQUEST_ERROR_OCCURRED_FLAG                8
#define IGNORE_ALARMS_FLAG                         16
#define IGNORE_FALSE_ALARMS_FLAG                   32
#define IGNORE_MOTION_DETECTORS_FLAG               64
#define IGNORE_IMMOBILIZER_BEEPER_FLAG             128
#define IGNORE_IMMOBILIZER_FLAG                    256
#define FIRST_STATUS_INFO_SENT_FLAG                512

#define REQUEST_IDLE_TIME_ON_ERROR        (10 * 1000 / portTICK_RATE_MS) // 10 sec
#define REQUEST_MAX_DURATION_TIME         (10 * 1000 / portTICK_RATE_MS) // 10 sec
#define STATUS_REQUESTS_SEND_INTERVAL_MS  (30 * 1000)
#define STATUS_REQUESTS_SEND_INTERVAL     (STATUS_REQUESTS_SEND_INTERVAL_MS / portTICK_RATE_MS) // 30 sec

#define IGNORE_ALARMS_TIMEOUT_SEC                           60
#define IGNORE_FALSE_ALARMS_TIMEOUT_SEC                     30
#define IGNORE_MOTION_DETECTORS_TIMEOUT_AFTER_TURN_ON_SEC   60
#define IGNORE_IMMOBILIZER_BEEPER_SEC                       300
#define IGNORE_IMMOBILIZER_SEC                              60

#define MILLISECONDS_COUNTER_DIVIDER 10

#define UART_RX_BUFFER_SIZE 30

#define ALARM_SOURCES_AMOUNT 7
#define ALARM_TIMERS_MAX_AMOUNT 5 // 3 alarms + 2 false alarms

char RESPONSE_SERVER_SENT_OK[] ICACHE_RODATA_ATTR = "\"statusCode\":\"OK\"";
char STATUS_INFO_POST_REQUEST[] ICACHE_RODATA_ATTR =
      "POST /server/esp8266/statusInfo HTTP/1.1\r\n"
      "Content-Length: <1>\r\n"
      "Host: <2>\r\n"
      "User-Agent: ESP8266\r\n"
      "Content-Type: application/json\r\n"
      "Connection: close\r\n"
      "Accept: application/json\r\n\r\n"
      "<3>\r\n";
char STATUS_INFO_REQUEST_PAYLOAD_TEMPLATE[] ICACHE_RODATA_ATTR =
      "{\"gain\":\"<1>\","
      "\"deviceName\":\"<2>\","
      "\"errors\":<3>,"
      "\"pendingConnectionErrors\":<4>,"
      "\"uptime\":<5>,"
      "\"buildTimestamp\":\"<6>\","
      "\"freeHeapSpace\":<7>,"
      "\"resetReason\":\"<8>\"}";
char ALARM_GET_REQUEST[] ICACHE_RODATA_ATTR =
      "GET /server/esp8266/alarm?alarmSource=<1> HTTP/1.1\r\n"
      "Host: <2>\r\n"
      "User-Agent: ESP8266\r\n"
      "Connection: close\r\n"
      "Accept: application/json\r\n\r\n";
char FALSE_ALARM_GET_REQUEST[] ICACHE_RODATA_ATTR =
      "GET /server/esp8266/falseAlarm?alarmSource=<1> HTTP/1.1\r\n"
      "Host: <2>\r\n"
      "User-Agent: ESP8266\r\n"
      "Connection: close\r\n"
      "Accept: application/json\r\n\r\n";
char IMMOBILIZER_ACTIVATION_REQUEST[] ICACHE_RODATA_ATTR =
      "GET /server/esp8266/immobilizerActivated HTTP/1.1\r\n"
      "Host: <1>\r\n"
      "User-Agent: ESP8266\r\n"
      "Accept: application/json\r\n"
      "Connection: close\r\n\r\n";
char UPDATE_FIRMWARE[] ICACHE_RODATA_ATTR = "\"updateFirmware\":true";
char FIRMWARE_UPDATE_GET_REQUEST[] ICACHE_RODATA_ATTR =
      "GET /esp8266_fota/<1> HTTP/1.1\r\n"
      "Host: <2>\r\n"
      "User-Agent: ESP8266\r\n"
      "Connection: close\r\n\r\n";
char MOTION_SENSOR_1_PIN[] ICACHE_RODATA_ATTR = "MOTION_SENSOR_1";
char MOTION_SENSOR_2_PIN[] ICACHE_RODATA_ATTR = "MOTION_SENSOR_2";
char MOTION_SENSOR_3_PIN[] ICACHE_RODATA_ATTR = "MOTION_SENSOR_3";
char IMMOBILIZER_LED_PIN[] ICACHE_RODATA_ATTR = "IMMOBILIZER_LED";
char PIR_LED_1_PIN[] ICACHE_RODATA_ATTR = "PIR_LED_1";
char MW_LED_1_PIN[] ICACHE_RODATA_ATTR = "MW_LED_1";
char PIR_LED_3_PIN[] ICACHE_RODATA_ATTR = "PIR_LED_3";
char MW_LED_3_PIN[] ICACHE_RODATA_ATTR = "MW_LED_3";

struct connection_user_data {
   bool response_received;
   char *request;
   char *response;
   void (*execute_on_disconnect) (struct espconn *connection);
   void (*execute_on_error) (struct espconn *connection);
   xTaskHandle timeout_request_supervisor_task;
   xTaskHandle parent_task;
   portTickType request_max_duration_time;
};

typedef enum {
   ALARM,
   FALSE_ALARM,
   IMMOBILIZER_ACTIVATION
} GeneralRequestType;

typedef enum {
   MOTION_SENSOR_1,
   MOTION_SENSOR_2,
   MOTION_SENSOR_3
} MotionSensorUnit;

struct motion_sensor {
   MotionSensorUnit unit;
   char *alarm_source;
};

struct request_data {
   struct motion_sensor *ms;
   GeneralRequestType request_type;
};

void scan_access_point_task(void *pvParameters);
void send_long_polling_requests_task(void *pvParameters);
void autoconnect_task(void *pvParameters);
void activate_status_requests_task_task(void *pvParameters);
void send_status_info_task(void *pvParameters);
void send_general_request_task(void *pvParameters);
void beep_task();
void successfull_connected_tcp_handler_callback(void *arg);
void successfull_disconnected_tcp_handler_callback();
void tcp_connection_error_handler_callback(void *arg, sint8 err);
void tcp_response_received_handler_callback(void *arg, char *pdata, unsigned short len);
void tcp_request_successfully_sent_handler_callback();
void tcp_request_successfully_written_into_buffer_handler_callback();
void long_polling_request_on_succeed_callback(struct espconn *connection);
void long_polling_request_on_error_callback(struct espconn *connection);
void long_polling_request_finish_action(struct espconn *connection);
void upgrade_firmware();
void establish_connection(struct espconn *connection);
void request_finish_action(struct espconn *connection);
void pins_interrupt_handler();
void uart_rx_intr_handler(void *params);
void uart_config();
void turn_motion_sensors_on();
void turn_motion_sensors_off();
bool are_motion_sensors_turned_on();
void input_pins_analyzer_task(void *pvParameters);
void send_general_request(struct request_data *request_data_param, unsigned char task_priority);
bool is_alarm_being_ignored(struct motion_sensor *ms, GeneralRequestType request_type);
void ignore_alarm(struct motion_sensor *ms, unsigned int timeout_ms);
void stop_ignoring_alarm(xTimerHandle xTimer);
void schedule_sending_status_info();
void check_for_update_firmware(char *response);
void ota_finished_callback(void *arg);
void disconnect_connection_task(void *pvParameters);
#endif
