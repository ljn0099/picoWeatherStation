#include <string.h>
#include <time.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "pico/aon_timer.h"
#include "pico/util/queue.h"

#include "include/ntp.h"
#include "include/queues.h"
#include "include/utils.h"
#include "include/weather_types.h"

#define NTP_MSG_LEN 48

#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_ERA_SECONDS (1ULL << 32)

#define NTP_CURRENT_ERA_BASE ((NTP_CURRENT_ERA * NTP_ERA_SECONDS) + NTP_DELTA)

#define NS_PER_SEC 1000000000
#define NS_PER_MS 1000000

#define NTP_FRAC_TO_NS(frac) ((uint32_t)((((uint64_t)(frac)) * NS_PER_SEC) >> 32))

// t0 = time when client send the request
// t1 = time when server recieves the request
// t2 = time when server sends the response
// t3 = time when client recieves the response

#define T1_SEC_POS 32
#define T1_FRAC_POS 36

#define T2_SEC_POS 40
#define T2_FRAC_POS 44

void ntp_apply_offset_timespec(struct timespec *localTime, const struct timespec *offset) {
    int64_t totalSec = localTime->tv_sec + offset->tv_sec;
    int64_t totalNsec = localTime->tv_nsec + offset->tv_nsec;

    if (totalNsec >= NS_PER_SEC) {
        totalSec++;
        totalNsec -= NS_PER_SEC;
    }
    else if (totalNsec < 0) {
        totalSec--;
        totalNsec += NS_PER_SEC;
    }

    localTime->tv_sec = totalSec;
    localTime->tv_nsec = totalNsec;
}

static void ntp_result(ntp_t *state, int status, struct timespec *offset) {
    time_msg_t msg;

    async_context_remove_at_time_worker(cyw43_arch_async_context(), &state->resendWorker);
    state->requestPending = false;

    if (status == 0 && offset) {
        state->attempts = 0;
        DEBUG_printf("Got offset\n");
        msg.alert = NTP_SUCCESS;
        msg.offset = *offset;
        queue_try_add(&timeResultQueue, &msg);
        return;
    }

    // Failure
    state->attempts++;

    if (state->attempts >= NTP_MAX_ATTEMPTS) {
        state->attempts = 0;
        msg.alert = NTP_FAILURE;
        msg.offset.tv_sec = 0;
        msg.offset.tv_nsec = 0;
        queue_try_add(&timeResultQueue, &msg);
    }
    else {
        time_request_t req = NTP_REQUEST;
        queue_try_add(&timeRequestQueue, &req);
    }
}

static void ntp_request(ntp_t *state) {
    cyw43_arch_lwip_begin();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
    uint8_t *req = (uint8_t *)p->payload;
    memset(req, 0, NTP_MSG_LEN);
    req[0] = 0x1b;

    aon_timer_get_time(&state->t0);

    udp_sendto(state->ntpPcb, p, &state->ntpServerAddress, NTP_PORT);
    pbuf_free(p);
    cyw43_arch_lwip_end();
}

static void ntp_dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    ntp_t *state = (ntp_t *)arg;
    if (ipaddr) {
        state->ntpServerAddress = *ipaddr;
        DEBUG_printf("ntp address %s\n", ipaddr_ntoa(ipaddr));
        ntp_request(state);
    }
    else {
        DEBUG_printf("ntp dns request failed\n");
        ntp_result(state, -1, NULL);
    }
}

static inline uint32_t ntp_read_u32(struct pbuf *p, uint16_t offset) {
    uint8_t buf[4] = {0};
    pbuf_copy_partial(p, buf, sizeof(buf), offset);
    return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

static inline uint8_t ntp_read_mode(struct pbuf *p) {
    return pbuf_get_at(p, 0) & 0x7;
}

static inline uint8_t ntp_read_leap_indicator(struct pbuf *p) {
    return (pbuf_get_at(p, 0) & 0xC0) >> 6;
}

static inline uint8_t ntp_read_version(struct pbuf *p) {
    return (pbuf_get_at(p, 0) & 0x38) >> 3;
}

static inline uint8_t ntp_read_stratum(struct pbuf *p) {
    return pbuf_get_at(p, 1);
}

static inline void ntp_read_reference_id_str(struct pbuf *p, char *out) {
    uint8_t buf[4];
    pbuf_copy_partial(p, buf, sizeof(buf), 12);
    for (int i = 0; i < 4; i++) {
        out[i] = buf[i];
    }
    out[4] = '\0'; // null-terminate
}

static inline void ntp_to_timespec_epoch_era(uint32_t sec, uint32_t frac, struct timespec *ts) {
    // Ajust with ntp era
    uint64_t totalSec = ((uint64_t)NTP_CURRENT_ERA * NTP_ERA_SECONDS) + sec;

    // Convert to epoch
    ts->tv_sec = totalSec - NTP_DELTA;

    // Convert ntp frac to nanoseconds
    ts->tv_nsec = NTP_FRAC_TO_NS(frac);
}

static inline void ntp_to_timespec_epoch_rollover(uint32_t sec, uint32_t frac,
                                                  struct timespec *ts) {
    uint64_t totalSec = sec;

    // Rollover adjustment
    if (totalSec < NTP_CURRENT_ERA_BASE)
        totalSec += NTP_ERA_SECONDS;

    // Convert to epoch
    ts->tv_sec = totalSec - NTP_DELTA;

    // Convert ntp frac to nanoseconds
    ts->tv_nsec = NTP_FRAC_TO_NS(frac);
}

static inline uint64_t ntp_to_epoch_sec_rollover(uint32_t sec) {
    uint64_t totalSec = sec;

    // Rollover adjustment
    if (totalSec < NTP_CURRENT_ERA_BASE)
        totalSec += NTP_ERA_SECONDS;

    // Convert to epoch
    return totalSec - NTP_DELTA;
}

static void ntp_get_offset_timespec(struct timespec *offset, struct timespec *t0,
                                    struct timespec *t1, struct timespec *t2, struct timespec *t3) {
    int64_t delta1 = (t1->tv_sec - t0->tv_sec) * NS_PER_SEC + (t1->tv_nsec - t0->tv_nsec);
    int64_t delta2 = (t2->tv_sec - t3->tv_sec) * NS_PER_SEC + (t2->tv_nsec - t3->tv_nsec);

    int64_t totalOffset = (delta1 + delta2) / 2;

    offset->tv_sec = totalOffset / NS_PER_SEC;
    offset->tv_nsec = totalOffset % NS_PER_SEC;

    if (offset->tv_nsec < 0) {
        offset->tv_sec--;
        offset->tv_nsec += NS_PER_SEC;
    }
}

int64_t ntp_calculate_rtt(const struct timespec *t0, const struct timespec *t1,
                          const struct timespec *t2, const struct timespec *t3) {
    int64_t delta1 = (t1->tv_sec - t0->tv_sec) * NS_PER_SEC + (t1->tv_nsec - t0->tv_nsec);
    int64_t delta2 = (t2->tv_sec - t3->tv_sec) * NS_PER_SEC + (t2->tv_nsec - t3->tv_nsec);
    return delta1 - delta2;
}

static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr,
                     uint16_t port) {
    ntp_t *state = (ntp_t *)arg;

    struct timespec t3;
    aon_timer_get_time(&t3);

    uint8_t stratum = ntp_read_stratum(p);
    uint8_t mode = ntp_read_mode(p);
    uint8_t leapIndicator = ntp_read_leap_indicator(p);

    if (!ip_addr_cmp(addr, &state->ntpServerAddress)) {
        DEBUG_printf("invalid ntp response: wrong address\n");
        goto fail;
    }

    if (port != NTP_PORT) {
        DEBUG_printf("invalid ntp response: wrong port\n");
        goto fail;
    }

    if (p->tot_len != NTP_MSG_LEN) {
        DEBUG_printf("invalid ntp response: wrong length\n");
        goto fail;
    }

    if (mode != 4) {
        DEBUG_printf("invalid ntp response: wrong mode\n");
        goto fail;
    }

    if (stratum == 0 || stratum >= 16) {
        DEBUG_printf("invalid ntp response: wrong stratum\n");
        goto fail;
    }

    if (leapIndicator == 3) {
        DEBUG_printf("invalid ntp response: server unsynchronized\n");
        goto fail;
    }

    uint32_t t1Sec = ntp_read_u32(p, T1_SEC_POS);
    uint32_t t1Frac = ntp_read_u32(p, T1_FRAC_POS);

    uint32_t t2Sec = ntp_read_u32(p, T2_SEC_POS);
    uint32_t t2Frac = ntp_read_u32(p, T2_FRAC_POS);

    struct timespec t1;
    ntp_to_timespec_epoch_rollover(t1Sec, t1Frac, &t1);

    struct timespec t2;
    ntp_to_timespec_epoch_rollover(t2Sec, t2Frac, &t2);

    struct timespec offset;
    ntp_get_offset_timespec(&offset, &state->t0, &t1, &t2, &t3);

    int64_t rttNs = ntp_calculate_rtt(&state->t0, &t1, &t2, &t3);
    if (rttNs > (NTP_RTT_MAX_MS * NS_PER_MS)) { // 200 ms
        DEBUG_printf("NTP response round-trip too high: %lld ms\n", rttNs / NS_PER_MS);
        goto fail;
    }

    ntp_result(state, 0, &offset);
    pbuf_free(p);
    return;

fail:
    ntp_result(state, -1, NULL);
    pbuf_free(p);
}

static void request_worker_fn(__unused async_context_t *context, async_at_time_worker_t *worker) {
    ntp_t *state = (ntp_t *)worker->user_data;

    if (state->requestPending)
        goto retry;

    time_request_t msg;
    if (!queue_try_remove(&timeRequestQueue, &msg))
        goto retry;
    if (msg != NTP_REQUEST)
        goto retry;

    state->requestPending = true;

    hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                       &state->resendWorker, NTP_RESEND_TIME_MS));

    int err = dns_gethostbyname(NTP_SERVER, &state->ntpServerAddress, ntp_dns_found, state);
    if (err == ERR_OK) {
        ntp_request(state);
    }
    else if (err != ERR_INPROGRESS) {
        DEBUG_printf("dns request failed\n");
        ntp_result(state, -1, NULL);
    }

    hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                       &state->requestWorker, NTP_CHECK_QUEUE_MS));
    return;

retry:
    hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                       &state->requestWorker, NTP_CHECK_QUEUE_MS));
    return;
}

static void resend_worker_fn(__unused async_context_t *context, async_at_time_worker_t *worker) {
    ntp_t *state = (ntp_t *)worker->user_data;
    DEBUG_printf("ntp request failed\n");
    ntp_result(state, -1, NULL);
}

static ntp_t *ntp_init(void) {
    ntp_t *state = (ntp_t *)calloc(1, sizeof(ntp_t));
    if (!state) {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    state->ntpPcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!state->ntpPcb) {
        DEBUG_printf("failed to create pcb\n");
        free(state);
        return NULL;
    }
    udp_recv(state->ntpPcb, ntp_recv, state);
    state->requestWorker.do_work = request_worker_fn;
    state->requestWorker.user_data = state;
    state->resendWorker.do_work = resend_worker_fn;
    state->resendWorker.user_data = state;
    state->requestPending = false;
    state->attempts = 0;
    return state;
}

ntp_t *ntp_start(void) {
    ntp_t *state = ntp_init();
    if (!state)
        panic("Failed to init ntp");

    hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                       &state->requestWorker, 0));

    return state;
}

void ntp_stop(ntp_t *state) {
    if (!state)
        return;

    async_context_remove_at_time_worker(cyw43_arch_async_context(), &state->requestWorker);
    async_context_remove_at_time_worker(cyw43_arch_async_context(), &state->resendWorker);

    cyw43_arch_lwip_begin();
    if (state->ntpPcb) {
        udp_recv(state->ntpPcb, NULL, NULL);
        udp_remove(state->ntpPcb);
        state->ntpPcb = NULL;
    }
    cyw43_arch_lwip_end();

    free(state);
}
