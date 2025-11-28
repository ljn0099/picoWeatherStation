#ifndef NTP_H
#define NTP_H

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "pico/cyw43_arch.h"
#include <time.h>

#define NTP_SERVER "ntp.roa.es"
#define NTP_PORT 123

#define NTP_CURRENT_ERA 0U

#define NTP_RTT_MAX_MS 200
#define NTP_MAX_ATTEMPTS 5

#define NTP_RETRY_TIME_MS (15 * 1000)
#define NTP_RESEND_TIME_MS (10 * 1000)
#define NTP_CHECK_QUEUE_MS (7 * 1000)

#define NTP_TEST_TIME_MS (30 * 1000)

typedef struct {
    ip_addr_t ntpServerAddress;
    struct udp_pcb *ntpPcb;
    async_at_time_worker_t requestWorker;
    async_at_time_worker_t resendWorker;
    struct timespec t0;
    bool requestPending;
    uint32_t attempts;
} ntp_t;

ntp_t *ntp_start(void);

void ntp_stop(ntp_t *state);

void ntp_apply_offset_timespec(struct timespec *localTime, const struct timespec *offset);

#endif
