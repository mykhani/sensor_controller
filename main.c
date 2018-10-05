#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <espressif/esp_common.h>
#include <ssid_config.h>

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

#include <i2c/i2c.h>

#include <esp/uart.h>
#include <esp/uart_regs.h>

#include "cJSON.h"

#include "tcs3471_interface.h"
#include "t6713.h"
#include "mpl115a2.h"
#include <sht3x.h>

TCS3471Handle_t tcs3471;

#define I2C_BUS 0
#define SCL_PIN 4
#define SDA_PIN 5

void IRAM *zalloc(size_t nbytes);

int wifi_disconnected = 0;

#define BUF_SIZE (1024)
#define MAX_PACKET_SIZE (512)

SemaphoreHandle_t wifi_alive;

QueueHandle_t publish_queue;
#define PUB_MSG_LEN 16

/* You can use http://test.mosquitto.org/ to test mqtt_client instead
 * of setting up your own MQTT server */

#define MQTT_HOST ("chamber1.local")
#define MQTT_PORT 5555

#define MQTT_USER NULL
#define MQTT_PASS NULL

#define MQTT_CLIENT_THREAD_NAME         "mqtt_client_thread"
#define MQTT_CLIENT_THREAD_STACK_WORDS  8192
#define MQTT_CLIENT_THREAD_PRIO         8

/* wrapper for passing json message to publish queue */
typedef struct QueueMessage_t {
    size_t len;
    uint8_t *data;
} QueueMessage;

static void  wifi_task(void *pvParameters)
{
    uint8_t status  = 0;
    uint8_t retries = 30;
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    printf("WiFi: connecting to WiFi\n\r");
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    while(1)
    {
        while ((status != STATION_GOT_IP) && (retries)){
            status = sdk_wifi_station_get_connect_status();
            printf("%s: status = %d\n\r", __func__, status );
            if( status == STATION_WRONG_PASSWORD ){
                printf("WiFi: wrong password\n\r");
                break;
            } else if( status == STATION_NO_AP_FOUND ) {
                printf("WiFi: AP not found\n\r");
                break;
            } else if( status == STATION_CONNECT_FAIL ) {
                printf("WiFi: connection failed\r\n");
                break;
            }
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
            --retries;
        }
#if 0
        if (status == STATION_GOT_IP) {
            printf("WiFi: Connected\n\r");
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }
#endif

        while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP) {
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }
        printf("WiFi: disconnected\n\r");
        sdk_wifi_station_disconnect();
        //retries = 30;
        //vTaskDelay( 1000 / portTICK_PERIOD_MS );
        printf("Restarting...\n");
        uart_flush_txfifo(0);
        uart_flush_txfifo(1);
        sdk_system_restart();
    }
}

static void i2c_detect(void)
{
    uint8_t i = 0;

    printf("Detecting i2c devices \r\n");
    for (i = 1; i < 127; i++) {
        if (!i2c_slave_write(I2C_BUS, i, NULL, NULL, 0)) {
            printf("Detected i2c device at : %x \r\n", i);
        }
    }
    
}

static const char *  get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *)my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

#define MQTT_TX_BUFSIZE (512)
#define MQTT_RX_BUFSIZE (512)

static void  mqtt_task(void *pvParameters)
{
    int ret         = 0;
    struct mqtt_network network;
    mqtt_client_t client   = mqtt_client_default;
    char mqtt_client_id[20];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    uint8_t *mqtt_buf = (uint8_t *)zalloc(MQTT_TX_BUFSIZE);
    uint8_t *mqtt_readbuf = (uint8_t *)zalloc(MQTT_RX_BUFSIZE);

    mqtt_network_new( &network );
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    while(1) {
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: started\n\r", __func__);
        printf("%s: (Re)connecting to MQTT server %s ... ",__func__,
               MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if( ret ){
            printf("error: %d\n\r", ret);
            taskYIELD();
            continue;
        }
        printf("done\n\r");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, MQTT_TX_BUFSIZE,
                      mqtt_readbuf, MQTT_RX_BUFSIZE);

        data.willFlag       = 0;
        data.MQTTVersion    = 3;
        data.clientID.cstring   = mqtt_client_id;
        data.username.cstring   = MQTT_USER;
        data.password.cstring   = MQTT_PASS;
        data.keepAliveInterval  = 10;
        data.cleansession   = 0;
        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if(ret){
            printf("error: %d\n\r", ret);
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }
        printf("done\r\n");
        xQueueReset(publish_queue);

        while(1){

            char msg[PUB_MSG_LEN - 1] = "\0";
            QueueMessage *qmsg;

            while(xQueueReceive(publish_queue, &qmsg, 0) ==
                  pdTRUE){
                printf("got message to publish\r\n");
                mqtt_message_t message;
                message.payload = qmsg->data;
                message.payloadlen = qmsg->len;
                message.dup = 0;
                message.qos = MQTT_QOS2;
                message.retained = 0;
                ret = mqtt_publish(&client, "/chamber/device/sensor/reading", &message);
                if (ret != MQTT_SUCCESS ){
                    printf("error while publishing message: %d\n", ret );
                    /* TODO: handle what to do when publishing fails.
                     * Maybe retransmit message? */
                    free(qmsg->data);
                    free(qmsg);
                    break;
                } else {
                    printf("Published \r\n");
                    free(qmsg->data);
                    free(qmsg);
                }
            }

            ret = mqtt_yield(&client, 100);
            if (ret == MQTT_DISCONNECTED)
                break;
        }
        printf("Connection dropped, request restart\n\r");
        mqtt_network_disconnect(&network);
        taskYIELD();
    }

    free(mqtt_buf);
    free(mqtt_readbuf);
}

static void sensor_task(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool tcs3471_detected;
    static sht3x_sensor_t* sensor = NULL;

    if ((sensor = sht3x_init_sensor (I2C_BUS, SHT3x_ADDR_1))) {
        printf("SHT3x sensor initialized\r\n");
    } else {
        printf("Could not initialize SHT3x sensor\n");
    }

    tcs3471 = tcs3471_create();

    printf("Detecting TCS3471\r\n");
    tcs3471_detected = tcs3471_detect(tcs3471);

    printf("TCS3471 detected: %s\r\n", tcs3471_detected ? "true" : "false");

    if (tcs3471_detected) {
        printf("TCS3471 Chip ID: %02x\r\n", tcs3471_getChipID(tcs3471));
        tcs3471_setIntegrationTime(tcs3471, 2.4); //old value: 700
        tcs3471_setWaitTime(tcs3471, 2.4);
        tcs3471_setGain(tcs3471, GAIN_1X);
        tcs3471_enable(tcs3471);
    }

    while (1) {
        printf("Taking env sensor reading\r\n");

        uint16_t clear = 0;
        uint16_t red = 0;
        uint16_t green = 0;
        uint16_t blue = 0;
        uint16_t co2ppm = 0;
        float air_pressure = 0.0f;
        float temperature = 0.0f;
        float humidity = 0.0f;

        if (tcs3471_rgbcValid(tcs3471)) {
            clear = tcs3471_readCData(tcs3471);
            red = tcs3471_readRData(tcs3471);
            green = tcs3471_readGData(tcs3471);
            blue = tcs3471_readBData(tcs3471);
            printf("Light sensor reading, Clear: %d, Red: %d, Green: %d, Blue: %d\r\n",
                    clear, red, green, blue);
        } else {
            printf("WARNING: Light sensor reading invalid \r\n");
        }

        co2ppm = t6713_read_gas_ppm();
        printf("CO2 sensor reading: %u PPM\r\n", co2ppm);

        air_pressure = mpl115_read_pressure();
        printf("Air Pressure sensor reading: %f kPa \r\n", air_pressure);

        if (sht3x_measure (sensor, &temperature, &humidity)) {
            printf("%.3f SHT3x Sensor: %.2f °C, %.2f %%\n",
            (double)sdk_system_get_time()*1e-3, temperature, humidity);
        } else {
            printf("SHT3x Sensor: reading error\r\n");
        }

        cJSON *root = cJSON_CreateObject();
        if (root == NULL) {
            printf("Failed to create JSON root object\r\n");
            goto end;
        }
        cJSON *_temperature = cJSON_CreateNumber(temperature);
        if (_temperature == NULL) {
            printf("Failed to create JSON temperature object\r\n");
            goto end;
        }
        cJSON_AddItemToObject(root, "temperature", _temperature);

        cJSON *_humidity = cJSON_CreateNumber(humidity);
        if (_humidity == NULL) {
            printf("Failed to create JSON humidity object\r\n");
            goto end;
        }
        cJSON_AddItemToObject(root, "humidity", _humidity);

        cJSON *_clear = cJSON_CreateNumber(clear);
        if (_clear == NULL) {
            printf("Failed to create JSON clear object\r\n");
            goto end;
        }
        cJSON_AddItemToObject(root, "clear", _clear);

        cJSON *_red = cJSON_CreateNumber(red);
        if (_red == NULL) {
            printf("Failed to create JSON red object\r\n");
            goto end;
        }
        cJSON_AddItemToObject(root, "red", _red);

        cJSON *_green = cJSON_CreateNumber(green);
        if (_green == NULL) { 
            printf("Failed to create JSON green object\r\n");
            goto end;
        }
        cJSON_AddItemToObject(root, "green", _green);

        cJSON *_blue = cJSON_CreateNumber(blue);
        if (_blue == NULL) {
            printf("Failed to create JSON blue object\r\n");
            goto end;
        }
        cJSON_AddItemToObject(root, "blue", _blue);

        cJSON *_co2ppm = cJSON_CreateNumber(co2ppm);
        if (_co2ppm == NULL) {
            printf("Failed to create JSON co2 object\r\n");
            goto end;
        }
        cJSON_AddItemToObject(root, "co2", _co2ppm);

        cJSON *_pressure = cJSON_CreateNumber(air_pressure);
        if (_pressure == NULL) {
            printf("Failed to create JSON air pressure object\r\n");
            goto end;
        }
        cJSON_AddItemToObject(root, "air_pressure", _pressure);

        char *json_msg = cJSON_Print(root);
        if (json_msg == NULL) {
            printf("Failed to print json msg string\r\n");
            goto end;
        }

        printf("Json string len: %d \r\n", strlen(json_msg));

        QueueMessage *qmsg = (QueueMessage*) zalloc(sizeof(QueueMessage));
        qmsg->len = strlen(json_msg) + 1; // for string NULL delimiter
        qmsg->data = json_msg;

        if (xQueueSend(publish_queue, &qmsg, portMAX_DELAY) == pdFALSE) {
            printf("Publish queue overflow.\r\n");
            free(qmsg);
            free(json_msg);
        }

end:
        cJSON_Delete(root);
        
        vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
    }
    
    tcs3471_delete(tcs3471);

    printf("sensor_client_thread going to be deleted\n");
    vTaskDelete(NULL);
    return;
}

void user_uart_init(void)
{
    uart_set_baud(0, 115200);

    gpio_set_iomux_function(2, IOMUX_GPIO2_FUNC_UART1_TXD);
    
    /* Set baud rate of UART1 used for debugging  */
    uart_set_baud(1, 115200);
}

void user_init(void)
{
    user_uart_init();

    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_80K);

    i2c_detect();

    vSemaphoreCreateBinary(wifi_alive);

    publish_queue = xQueueCreate(1, sizeof(uint8_t *));
    
    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 2, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 512, NULL, 4, NULL);
    xTaskCreate(&sensor_task, "sensor_task", 512, NULL, 5, NULL);
}
