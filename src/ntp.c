#include <time.h>
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "include/ntp.h"
#include "include/queues.h"
#include "include/utils.h"

static void ntp_result(ntp_t *state, int status, uint64_t *result) {
    async_context_remove_at_time_worker(cyw43_arch_async_context(), &state->resendWorker);
    async_context_remove_at_time_worker(cyw43_arch_async_context(), &state->requestWorker);
    if (status == 0 && result) {
        queue_try_add(&epochTimeQueue, result);
        free(state);
    }
    else {
        async_context_add_at_time_worker_in_ms(
            cyw43_arch_async_context(), &state->requestWorker,
            NTP_TEST_TIME_MS); // repeat the request in future
        DEBUG_printf("Next request in %ds\n", NTP_TEST_TIME_MS / 1000);
    }
}

// Make an NTP request
static void ntp_request(ntp_t *state) {
    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
    uint8_t *req = (uint8_t *)p->payload;
    memset(req, 0, NTP_MSG_LEN);
    req[0] = 0x1b;
    udp_sendto(state->ntpPcb, p, &state->ntpServerAddress, NTP_PORT);
    pbuf_free(p);
    cyw43_arch_lwip_end();
}

// Call back with a DNS result
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

// NTP data received
static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr,
                     u16_t port) {
    ntp_t *state = (ntp_t *)arg;
    uint8_t mode = pbuf_get_at(p, 0) & 0x7;
    uint8_t stratum = pbuf_get_at(p, 1);

    // Check the result
    if (ip_addr_cmp(addr, &state->ntpServerAddress) && port == NTP_PORT &&
        p->tot_len == NTP_MSG_LEN && mode == 0x4 && stratum != 0) {
        uint8_t seconds_buf[4] = {0};
        pbuf_copy_partial(p, seconds_buf, sizeof(seconds_buf), 40);
        uint32_t seconds_buf32 =
            seconds_buf[0] << 24 | seconds_buf[1] << 16 | seconds_buf[2] << 8 | seconds_buf[3];

        uint64_t seconds_since_1900 = (uint64_t)seconds_buf32;

        // Rollover adjustment (~2036/2037)
        if (seconds_since_1900 < NTP_DELTA) {
            seconds_since_1900 += 0x100000000ULL; // 2^32
        }

        uint64_t seconds_since_1970 = seconds_since_1900 - NTP_DELTA;
        uint64_t epoch = (uint64_t)seconds_since_1970;

        ntp_result(state, 0, &epoch);
    }
    else {
        DEBUG_printf("invalid ntp response\n");
        ntp_result(state, -1, NULL);
    }
    pbuf_free(p);
}

// Called to make a NTP request
static void request_worker_fn(__unused async_context_t *context, async_at_time_worker_t *worker) {
    ntp_t *state = (ntp_t *)worker->user_data;
    hard_assert(
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &state->resendWorker,
                                               NTP_RESEND_TIME_MS)); // in case UDP request is lost
    int err = dns_gethostbyname(NTP_SERVER, &state->ntpServerAddress, ntp_dns_found, state);
    if (err == ERR_OK) {
        ntp_request(state); // Cached DNS result, make NTP request
    }
    else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        DEBUG_printf("dns request failed\n");
        ntp_result(state, -1, NULL);
    }
}

// Called to resend an NTP request if it appears to get lost
static void resend_worker_fn(__unused async_context_t *context, async_at_time_worker_t *worker) {
    ntp_t *state = (ntp_t *)worker->user_data;
    DEBUG_printf("ntp request failed\n");
    ntp_result(state, -1, NULL);
}

static ntp_t *ntp_init(void) {
    ntp_t *state = (ntp_t *)calloc(1, sizeof(ntp_t));
    if (!state) {
        DEBUG_printf("failed to allocate state for ntp\n");
        return NULL;
    }
    state->ntpPcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!state->ntpPcb) {
        DEBUG_printf("failed to create pcb for ntp\n");
        free(state);
        return NULL;
    }
    udp_recv(state->ntpPcb, ntp_recv, state);
    state->requestWorker.do_work = request_worker_fn;
    state->requestWorker.user_data = state;
    state->resendWorker.do_work = resend_worker_fn;
    state->resendWorker.user_data = state;
    return state;
}

// Runs ntp test forever
void ntp_start_request(void) {
    ntp_t *state = ntp_init();
    hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),
                                                       &state->requestWorker, 0));
}
