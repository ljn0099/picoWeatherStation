#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "lwip/altcp_tls.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h" // needed to set hostname
#include "lwip/dns.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN (sizeof(MQTT_USERNAME) + 50)
#endif

typedef struct {
    mqtt_client_t *mqttClientInst;
    struct mqtt_connect_client_info_t mqttClientInfo;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqttServerAddress;
    bool connectDone;
    int subscribeCount;
    bool stopClient;
    async_at_time_worker_t requestWorker;
} mqtt_t;

// DNS
#define DNS_SERVER "1.1.1.1"    
#define DNS_SERVER_ALT "1.0.0.1"

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

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

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

#define MQTT_SERVER_PORT 8883

static const char *full_topic(const char *name) {
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "%s%s", MQTT_USERNAME, name);
    return full_topic;
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    mqtt_t *state = (mqtt_t *)arg;

    const char *basic_topic = state->topic + (sizeof(MQTT_USERNAME) - 1);

    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);

    // if (strcmp(basic_topic, "/ping") == 0) {
    //     char buf[11];
    //     snprintf(buf, sizeof(buf), "%lu", to_ms_since_boot(get_absolute_time()) / 1000);
    //     mqtt_publish(state->mqttClientInst, full_topic("/uptime"), buf, strlen(buf),
    //                  MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    // }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, uint32_t totLen) {
    mqtt_t *state = (mqtt_t *)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    mqtt_t *state = (mqtt_t *)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connectDone = true;
        // sub_unsub_topics(state, true); // subscribe;

        // indicate online
        // if (state->mqtt_client_info.will_topic) {
        //     mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1,
        //                  MQTT_WILL_QOS, true, pub_request_cb, state);
        // }

        // Publish temperature every 10 sec if it's changed
        // temperature_worker.user_data = state;
        // async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker,
        // 0);
    }
    else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connectDone) {
            panic("Failed to connect to mqtt server");
        }
    }
    else {
        panic("Unexpected status");
    }
}

static void mqtt_start_client(mqtt_t *state) {
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqttServerAddress));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqttClientInst, &state->mqttServerAddress, MQTT_SERVER_PORT,
                            mqtt_connection_cb, state, &state->mqttClientInfo) != ERR_OK) {
        panic("MQTT broker connection error");
    }
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqttClientInst->conn), MQTT_SERVER);

    mqtt_set_inpub_callback(state->mqttClientInst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb,
                            state);
    cyw43_arch_lwip_end();
}

static void mqtt_dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    mqtt_t *state = (mqtt_t *)arg;
    if (ipaddr) {
        state->mqttServerAddress = *ipaddr;
        DEBUG_printf("mqtt address %s\n", ipaddr_ntoa(ipaddr));
        mqtt_start_client(state);
    }
    else {
        panic("mqtt dns request failed\n");
        // DEBUG_printf("mqtt dns request failed\n");
        // ntp_result(state, -1, NULL);
    }
}

static void request_worker_fn(__unused async_context_t *context, async_at_time_worker_t *worker) {
    mqtt_t *state = (mqtt_t *)worker->user_data;

    int err = dns_gethostbyname(MQTT_SERVER, &state->mqttServerAddress, mqtt_dns_found, state);
    if (err == ERR_OK) {
        mqtt_start_client(state);
    }
    else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        panic("dns request failed\n");
        // DEBUG_printf("dns request failed\n");
        // ntp_result(state, -1, NULL);
    }
}

static mqtt_t *mqtt_init(const char *uniqueId) {
    mqtt_t *state = (mqtt_t *)calloc(1, sizeof(mqtt_t));
    if (!state) {
        DEBUG_printf("failed to allocate state for mqtt\n");
        return NULL;
    }

    state->mqttClientInst = mqtt_client_new();
    if (!state->mqttClientInst) {
        DEBUG_printf("MQTT client instance creation error");
        return NULL;
    }

    state->requestWorker.do_work = request_worker_fn;
    state->requestWorker.user_data = state;

    state->mqttClientInfo.client_id = uniqueId;
    state->mqttClientInfo.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec

    state->mqttClientInfo.client_user = MQTT_USERNAME;
    state->mqttClientInfo.client_pass = MQTT_PASSWORD;

    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(MQTT_WILL_TOPIC), sizeof(will_topic));
    state->mqttClientInfo.will_topic = will_topic;
    state->mqttClientInfo.will_msg = MQTT_WILL_MSG;
    state->mqttClientInfo.will_qos = MQTT_WILL_QOS;
    state->mqttClientInfo.will_retain = true;

    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    state->mqttClientInfo.tls_config =
        altcp_tls_create_config_client((const uint8_t *)ca_cert, sizeof(ca_cert));

    return state;
}

void mqtt_start(void) {
    // Use board unique id
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for (int i = 0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    // Generate a unique name, e.g. pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    INFO_printf("Device name %s\n", client_id_buf);

    mqtt_t *state = mqtt_init(client_id_buf);

    hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                       &state->requestWorker, 0));
}

int main(void) {
    stdio_init_all();
    INFO_printf("mqtt client starting\n");

    if (cyw43_arch_init()) {
        panic("Failed to inizialize CYW43");
    }

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK,
                                           30000)) {
        panic("Failed to connect");
    }
    INFO_printf("\nConnected to Wifi\n");

    // Set dns servers
    ip_addr_t dns1, dns2;               

    ip4addr_aton(DNS_SERVER, &dns1);
    ip4addr_aton(DNS_SERVER_ALT, &dns2);
                                        
    dns_setserver(0, &dns1);            
    dns_setserver(1, &dns2);

    mqtt_start();

    while (1) {
        tight_loop_contents();
    }

    return 0;
}
