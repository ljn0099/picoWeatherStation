#ifndef NTP_H
#define NTP_H

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "pico/cyw43_arch.h"

#define NTP_SERVER "ntp.roa.es"
#define NTP_PORT 123
#define NTP_MSG_LEN 48
#define NTP_DELTA 2208988800ULL // seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_TEST_TIME_MS (30 * 1000)
#define NTP_RESEND_TIME_MS (10 * 1000)

typedef struct {
    ip_addr_t ntpServerAddress;
    struct udp_pcb *ntpPcb;
    async_at_time_worker_t requestWorker;
    async_at_time_worker_t resendWorker;
} ntp_t;

void ntp_start_request(void);

#endif
