#ifndef MQTT_H
#define MQTT_H

#include "lwip/altcp_tls.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h" // needed to set hostname
#include "pico/unique_id.h"

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

#define UNIQUE_ID_HEX_LEN (2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1)

// This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#define DNS_RETRY_MS 5000
#define MQTT_RETRY_MS 5000

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN (sizeof(MQTT_USERNAME) + 50)
#endif

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

typedef struct {
    mqtt_client_t *mqttClientInst;
    struct mqtt_connect_client_info_t mqttClientInfo;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    char clientId[((sizeof(MQTT_DEVICE_NAME) - 1) + 1 + (UNIQUE_ID_HEX_LEN - 1) + 1)];
    char willTopic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqttServerAddress;
    bool connectDone;
    int subscribeCount;
    bool stopClient;
    async_at_time_worker_t requestWorker;
    async_at_time_worker_t publishWorker;
} mqtt_t;

typedef struct {
    mqtt_t *state;
    char topic[MQTT_TOPIC_LEN];
    char *msg;
    size_t msgLen;
} mqtt_publish_job_t;

void mqtt_start(void);

void mqtt_publish_dynamic(const char *subtopic, char *msg, size_t len);

bool mqtt_publish_blocking(const char *subtopic, const uint8_t *msg, size_t len);

// keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

// qos passed to mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// topic used for last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#define MQTT_SERVER_PORT 8883

#endif
