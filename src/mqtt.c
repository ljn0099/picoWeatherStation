#include "lwip/apps/mqtt.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "lwip/altcp_tls.h"
#include "lwip/apps/mqtt_priv.h" // needed to set hostname
#include "lwip/dns.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "include/mqtt.h"
#include "include/queues.h"
#include "include/utils.h"
#include "include/weather_types.h"

#include "hardware/watchdog.h"

static const char *full_topic(const char *name) {
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "stations/%s%s", MQTT_USERNAME, name);
    return full_topic;
}

static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        ERROR_printf("pub_request_cb failed %d", err);
    }
}

static void sub_request_cb(void *arg, err_t err) {
    mqtt_t *state = (mqtt_t *)arg;
    if (err != 0) {
        ERROR_printf("subscribe request failed %d", err);
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, MQTT_RETRY_MS);
        return;
    }
    state->subscribeCount++;
}

static void unsub_request_cb(void *arg, err_t err) {
    mqtt_t *state = (mqtt_t *)arg;
    if (err != 0) {
        ERROR_printf("unsubscribe request failed %d", err);
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, MQTT_RETRY_MS);
        return;
    }
    state->subscribeCount--;
    assert(state->subscribeCount >= 0);
}

static void sub_unsub_topics(mqtt_t *state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqttClientInst, full_topic("/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqttClientInst, full_topic("/reboot"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    (void)flags;
    mqtt_t *state = (mqtt_t *)arg;

    const char *basicTopic = state->topic + strlen("stations/") + sizeof(MQTT_USERNAME) - 1;

    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);

    if (strcmp(basicTopic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%lu", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqttClientInst, full_topic("/uptime"), buf, strlen(buf),
                     MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
    else if (strcmp(basicTopic, "/reboot") == 0) {
        watchdog_enable(1, false);
        while (1);
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, uint32_t totLen) {
    (void)totLen;
    mqtt_t *state = (mqtt_t *)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void)client;
    mqtt_t *state = (mqtt_t *)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connectDone = true;
        sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqttClientInfo.will_topic) {
            mqtt_publish(state->mqttClientInst, state->mqttClientInfo.will_topic, "1", 1,
                         MQTT_WILL_QOS, true, pub_request_cb, state);
        }
        async_context_add_at_time_worker_in_ms(
            cyw43_arch_async_context(), &state->publishWorker, MQTT_PUBLISH_TIME_MS);
    }
    else {
        DEBUG_printf("MQTT disconnected (status %d), reconnecting...\n", status);
        state->connectDone = false;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, MQTT_RETRY_MS);
    }
}

static void mqtt_start_client(mqtt_t *state) {
    DEBUG_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    DEBUG_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqttServerAddress));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqttClientInst, &state->mqttServerAddress, MQTT_SERVER_PORT,
                            mqtt_connection_cb, state, &state->mqttClientInfo) != ERR_OK) {
        DEBUG_printf("MQTT broker connection error\n");
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, MQTT_RETRY_MS);
        cyw43_arch_lwip_end();
        return;
    }
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqttClientInst->conn), MQTT_SERVER);

    mqtt_set_inpub_callback(state->mqttClientInst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb,
                            state);
    cyw43_arch_lwip_end();
}

static void mqtt_dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    (void)hostname;
    mqtt_t *state = (mqtt_t *)arg;
    if (ipaddr) {
        state->mqttServerAddress = *ipaddr;
        DEBUG_printf("mqtt address %s\n", ipaddr_ntoa(ipaddr));
        mqtt_start_client(state);
    }
    else {
        DEBUG_printf("mqtt dns request failed\n");
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, DNS_RETRY_MS);
    }
}

static void publish_worker_request_fallback_cb(__unused async_context_t *context, async_at_time_worker_t *worker) {
    mqtt_t *state = (mqtt_t *)worker->user_data;
    state->weatherDataOnFly = false;
}

static void publish_worker_request_cb(void *arg, err_t err) {
    mqtt_t *state = (mqtt_t *)arg;
    async_context_add_at_time_worker_in_ms(
            cyw43_arch_async_context(), &state->publishWorkerReqFallback, 30000);

    if (err != 0) {
        DEBUG_printf("publish_worker_request_cb failed %d\n", err);
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, MQTT_RETRY_MS);
    }
    else {
        DEBUG_printf("publish_worker_request_cb correct\n");
        queue_try_remove(&weatherSerializedQueue, NULL);
    }

    state->weatherDataOnFly = false;
}

static void publish_worker_fn(__unused async_context_t *context, async_at_time_worker_t *worker) {
    mqtt_t *state = (mqtt_t *)worker->user_data;

    payload_t payload;

    if (state->weatherDataOnFly) {
        hard_assert(async_context_add_at_time_worker_in_ms(
            cyw43_arch_async_context(), &state->publishWorker, MQTT_PUBLISH_TIME_MS));
        return;
    }

    if (!queue_try_peek(&weatherSerializedQueue, &payload)) {
        hard_assert(async_context_add_at_time_worker_in_ms(
            cyw43_arch_async_context(), &state->publishWorker, MQTT_PUBLISH_TIME_MS));
        return;
    }

    const char *topic = full_topic("/data");

    err_t err = mqtt_publish(state->mqttClientInst, topic, payload.msg, payload.len,
                             MQTT_PUBLISH_QOS, 0, &publish_worker_request_cb, state);
    async_context_add_at_time_worker_in_ms(
            cyw43_arch_async_context(), &state->publishWorkerReqFallback, 30000);

    if (err == ERR_OK) {
        state->weatherDataOnFly = true;
    }
    else {
        DEBUG_printf("Error trying to publish mqtt msg\n");
    }

    hard_assert(async_context_add_at_time_worker_in_ms(
        cyw43_arch_async_context(), &state->publishWorker, MQTT_PUBLISH_TIME_MS));
}

static void request_worker_fn(__unused async_context_t *context, async_at_time_worker_t *worker) {
    mqtt_t *state = (mqtt_t *)worker->user_data;

    int err = dns_gethostbyname(MQTT_SERVER, &state->mqttServerAddress, mqtt_dns_found, state);
    if (err == ERR_OK) {
        mqtt_start_client(state);
    }
    else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        DEBUG_printf("dns request failed\n");
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, DNS_RETRY_MS);
    }
}

static mqtt_t *mqtt_init() {
    mqtt_t *state = (mqtt_t *)calloc(1, sizeof(mqtt_t));
    if (!state)
        panic("failed to allocate state for mqtt\n");

    state->mqttClientInst = mqtt_client_new();
    if (!state->mqttClientInst) {
        panic("MQTT client instance creation error\n");
    }

    state->requestWorker.do_work = request_worker_fn;
    state->requestWorker.user_data = state;

    state->publishWorker.do_work = publish_worker_fn;
    state->publishWorker.user_data = state;

    state->publishWorkerReqFallback.do_work = publish_worker_request_fallback_cb;
    state->publishWorkerReqFallback.user_data = state;

    state->mqttClientInfo.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec

    state->mqttClientInfo.client_user = MQTT_USERNAME;
    state->mqttClientInfo.client_pass = MQTT_PASSWORD;

    strncpy(state->willTopic, full_topic(MQTT_WILL_TOPIC), sizeof(state->willTopic));
    state->mqttClientInfo.will_topic = state->willTopic;
    state->mqttClientInfo.will_msg = MQTT_WILL_MSG;
    state->mqttClientInfo.will_qos = MQTT_WILL_QOS;
    state->mqttClientInfo.will_retain = true;
    state->weatherDataOnFly = false;

    static const uint8_t ca_cert[] = MQTT_TLS_ROOT_CERT;
    state->mqttClientInfo.tls_config =
        altcp_tls_create_config_client((const uint8_t *)ca_cert, sizeof(ca_cert));

    return state;
}

void mqtt_start(void) {
    // Use board unique id
    char uniqueIdBuf[UNIQUE_ID_HEX_LEN];

    pico_get_unique_board_id_string(uniqueIdBuf, sizeof(uniqueIdBuf));
    for (size_t i = 0; i < sizeof(uniqueIdBuf) - 1; i++) {
        uniqueIdBuf[i] = tolower(uniqueIdBuf[i]);
    }

    mqtt_t *state = mqtt_init();

    snprintf(state->clientId, sizeof(state->clientId), "%s-%s", MQTT_DEVICE_NAME, uniqueIdBuf);
    state->mqttClientInfo.client_id = state->clientId;

    hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                       &state->requestWorker, 0));
}
