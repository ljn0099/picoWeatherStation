#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "lwip/altcp_tls.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h" // needed to set hostname
#include "lwip/dns.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "include/mqtt.h"
#include "include/utils.h"

static mqtt_t *mqttState = NULL;

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
        hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, MQTT_RETRY_MS));
        return;
    }
    state->subscribeCount++;
}

static void unsub_request_cb(void *arg, err_t err) {
    mqtt_t *state = (mqtt_t *)arg;
    if (err != 0) {
        ERROR_printf("unsubscribe request failed %d", err);
        hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, MQTT_RETRY_MS));
        return;
    }
    state->subscribeCount--;
    assert(state->subscribeCount >= 0);
}

static void sub_unsub_topics(mqtt_t *state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqttClientInst, full_topic("/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    mqtt_t *state = (mqtt_t *)arg;

    const char *basic_topic = state->topic + (sizeof(MQTT_USERNAME) - 1);

    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);

    if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%lu", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqttClientInst, full_topic("/uptime"), buf, strlen(buf),
                     MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, uint32_t totLen) {
    mqtt_t *state = (mqtt_t *)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    mqtt_t *state = (mqtt_t *)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connectDone = true;
        sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqttClientInfo.will_topic) {
            mqtt_publish(state->mqttClientInst, state->mqttClientInfo.will_topic, "1", 1,
                         MQTT_WILL_QOS, true, pub_request_cb, state);
        }
    }
    else {
        DEBUG_printf("MQTT disconnected (status %d), reconnecting...\n", status);
        state->connectDone = false;
        hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, MQTT_RETRY_MS));
    }
}

static void mqtt_start_client(mqtt_t *state) {
    state->mqttClientInst = mqtt_client_new();
    if (!state->mqttClientInst) {
        panic("MQTT client instance creation error\n");
    }

    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqttServerAddress));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqttClientInst, &state->mqttServerAddress, MQTT_SERVER_PORT,
                            mqtt_connection_cb, state, &state->mqttClientInfo) != ERR_OK) {
        DEBUG_printf("MQTT broker connection error\n");
        hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, MQTT_RETRY_MS));
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
    mqtt_t *state = (mqtt_t *)arg;
    if (ipaddr) {
        state->mqttServerAddress = *ipaddr;
        DEBUG_printf("mqtt address %s\n", ipaddr_ntoa(ipaddr));
        mqtt_start_client(state);
    }
    else {
        DEBUG_printf("mqtt dns request failed\n");
        hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, DNS_RETRY_MS));
    }
}

static void request_worker_fn(__unused async_context_t *context, async_at_time_worker_t *worker) {
    mqtt_t *state = (mqtt_t *)worker->user_data;

    int err = dns_gethostbyname(MQTT_SERVER, &state->mqttServerAddress, mqtt_dns_found, state);
    if (err == ERR_OK) {
        mqtt_start_client(state);
    }
    else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        DEBUG_printf("dns request failed\n");
        hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                           &state->requestWorker, DNS_RETRY_MS));
    }
}

static mqtt_t *mqtt_init() {
    mqtt_t *state = (mqtt_t *)calloc(1, sizeof(mqtt_t));
    if (!state)
        panic("failed to allocate state for mqtt\n");

    state->requestWorker.do_work = request_worker_fn;
    state->requestWorker.user_data = state;

    state->mqttClientInfo.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec

    state->mqttClientInfo.client_user = MQTT_USERNAME;
    state->mqttClientInfo.client_pass = MQTT_PASSWORD;

    snprintf(state->willTopic, sizeof(state->willTopic), "%s%s", MQTT_USERNAME, MQTT_WILL_TOPIC);
    state->mqttClientInfo.will_topic = state->willTopic;
    state->mqttClientInfo.will_msg = MQTT_WILL_MSG;
    state->mqttClientInfo.will_qos = MQTT_WILL_QOS;
    state->mqttClientInfo.will_retain = true;

    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    state->mqttClientInfo.tls_config =
        altcp_tls_create_config_client((const uint8_t *)ca_cert, sizeof(ca_cert));

    return state;
}

void mqtt_publish_worker_fn(__unused async_context_t *ctx, async_at_time_worker_t *worker) {
    mqtt_publish_job_t *job = (mqtt_publish_job_t *)worker->user_data;

    err_t err = mqtt_publish(job->state->mqttClientInst, job->topic, job->msg,
                             job->msgLen, MQTT_PUBLISH_QOS, 0, NULL, job->state);
    if (err != ERR_OK) {
        DEBUG_printf("MQTT publish error %d\n", err);
    }

    free(job->msg);
    free(job);
}

void mqtt_publish_dynamic(const char *topic, char *msg, size_t len) {
    mqtt_publish_job_t *job = malloc(sizeof(*job));
    if (!job) {
        free(msg);
        return;
    }

    job->state = mqttState;
    strncpy(job->topic, full_topic(topic), sizeof(job->topic)-1);
    job->topic[sizeof(job->topic)-1] = 0;

    job->msg = msg;
    job->msgLen = len;

    async_at_time_worker_t worker = {
        .do_work = mqtt_publish_worker_fn,
        .user_data = job
    };
    hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &worker, 0));
}


void mqtt_start(void) {
    // Use board unique id
    char uniqueIdBuf[UNIQUE_ID_HEX_LEN];

    pico_get_unique_board_id_string(uniqueIdBuf, sizeof(uniqueIdBuf));
    for (int i = 0; i < sizeof(uniqueIdBuf) - 1; i++) {
        uniqueIdBuf[i] = tolower(uniqueIdBuf[i]);
    }

    mqttState = mqtt_init();

    snprintf(mqttState->clientId, sizeof(mqttState->clientId), "%s-%s", MQTT_DEVICE_NAME, uniqueIdBuf);
    mqttState->mqttClientInfo.client_id = mqttState->clientId;

    hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                       &mqttState->requestWorker, 0));
}
