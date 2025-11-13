#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "pico/cyw43_arch.h"

#include "sensor/dps310.h"
#include "sensor/hdc3022.h"
#include "sensor/ltr390.h"
#include "sensor/pcf8523.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

// DNS
#define DNS_SERVER "1.1.1.1"
#define DNS_SERVER_ALT "1.0.0.1"

// NTP
#define NTP_SERVER "ntp.roa.es"
#define NTP_PORT 123
#define NTP_MSG_LEN 48
#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_TEST_TIME_MS (30 * 1000)
#define NTP_RESEND_TIME_MS (10 * 1000)

// I2C Config
#define I2C_BUS i2c0
#define I2C_SDA 16
#define I2C_SCL 17
#define I2C_SPEED 100000 // 100kHz

// I2C Addresses
#define HDC3022_ADDRESS 0x44
#define LTR390_ADDRESS 0x53
#define DPS310_ADDRESS 0x77
#define PCF8523_ADDRESS 0x68

// Sensors config
#define LTR390_MEAS_RATE LTR390_MEAS_RATE_100MS
#define LTR390_GAIN LTR390_GAIN_3
#define LTR390_RESOLUTION LTR390_RES_18BIT
#define LTR390_INT_TIME_MS ((LTR390_RES_18BIT_INT_TIME_US / 1000.0f) + 20)

#define HDC3022_MODE HDC3022_AUTO_MEAS_1X_1S_LPM_0

#define DPS310_PRES_OVERSAMPLING DPS310_OVERSAMPLING_16_TIMES
#define DPS310_TEMP_OVERSAMPLING DPS310_OVERSAMPLING_1_TIME
#define DPS310_MODE DPS310_MODE_CONT_PRES_TEMP_MEAS

// Time config
#define SAMPLE_INTERVAL_MS 2000   // 2 sec
#define COMPUTE_INTERVAL_MS 10000 // 10 sec
#define SAMPLE_INTERVAL_S (SAMPLE_INTERVAL_MS / 1000)
#define COMPUTE_INTERVAL_S (COMPUTE_INTERVAL_MS / 1000)

#define WATCHDOG_INTERVAL_MS 1000

// Sensor GPIOs
#define RAIN_GAUGE_PIN 21
#define ANEMOMETER_PIN 22
#define WIND_VANE_PIN 26
#define WIND_VANE_ADC_CHANNEL 0

// Constants
#define BASE_CENTURY 2000

#define ANEMOMETER_FACTOR 2.4f
#define PLUVIOMETER_FACTOR 0.2794f

#define ADC_VREF 3.3f // RPI Pico VREF
#define ADC_BITS 12   // RPI Pico ADC Resolution

#define WIND_VANE_TOLERANCE_V 0.05f
#define WIND_VANE_TOLERANCE_ADC (VOLT_TO_ADC(WIND_VANE_TOLERANCE_V))

#define WEATHER_FINAL_QUEUE_SIZE 5
#define WEATHER_COMPUTE_QUEUE_SIZE 16
#define WEATHER_SAMPLE_QUEUE_SIZE 16

// Utils
#define ARRAY_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))

#define VOLT_TO_ADC(v) ((uint16_t)((v) * ((1 << ADC_BITS) - 1) / ADC_VREF))
#define ADC_TO_VOLT(adc) ((float)(adc) * ADC_VREF / ((1 << ADC_BITS) - 1))

#define RAD_TO_DEG(x) ((x) * 180.0f / M_PI)
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0f)
#define PA_TO_HPA(x) ((x) / 100.0f)

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

#define COMPUTE_QUEUE_ADD_CMD(cmd) queue_try_add(&weatherComputeQueue, &(computeCmd_t){cmd})

#define SAMPLE_QUEUE_ADD_CMD(cmd) queue_try_add(&weatherSampleQueue, &(sampleCmd_t){cmd})

typedef struct {
    float value;
    bool valid;
} finalData_t;

typedef struct {
    float sum;
    uint32_t count;
} avgData_t;

typedef struct {
    float sumXRad;
    float sumYRad;
    float lastDirRad;
    uint32_t count;
} windDirection_t;

typedef struct {
    volatile uint32_t interrupts;
} rainfall_t;

typedef struct {
    volatile uint32_t interrupts;
} windSpeed_t;

typedef struct {
    volatile uint32_t interrupts;
    uint32_t peakInterrupts;
    float peakDirectionRad;
} peakWind_t;

// Global struct because its for IRQs
typedef struct {
    rainfall_t rainfall;
    windSpeed_t windSpeed;
    peakWind_t peakWind;
} weatherIrq_t;

// Struct for sensor internals
typedef struct {
    hdc3022_t hdc3022;
    dps310_t dps310;
    ltr390_t ltr390;
    pcf8523_t pcf8523;
} weatherSensor_t;

// Struct for averages accumulation
typedef struct {
    windDirection_t windDirection;
    avgData_t temperature;
    avgData_t humidity;
    avgData_t pressure;
    avgData_t lux;
    avgData_t uvi;
} weatherAverage_t;

// Struct with the final data
typedef struct {
    uint64_t epochTime;
    finalData_t windSpeedKmH;
    finalData_t windDirectionDeg;

    finalData_t peakWindSpeedKmH;
    finalData_t peakWindDirectionDeg;

    finalData_t rainfallMM;

    finalData_t temperatureCelsius;
    finalData_t humidityRh;
    finalData_t uvi;
    finalData_t lux;
    finalData_t pressureHPA;
} weatherFinal_t;

typedef struct {
    uint16_t adcValue;
    float directionRad;
} windVaneCalibration_t;

const windVaneCalibration_t windVaneCalibrations[] = {
    {VOLT_TO_ADC(0.411f), DEG_TO_RAD(0.00f)},  // N
    {VOLT_TO_ADC(1.376f), DEG_TO_RAD(22.5f)},  // NNE
    {VOLT_TO_ADC(1.202f), DEG_TO_RAD(45.0f)},  // NE
    {VOLT_TO_ADC(2.794f), DEG_TO_RAD(67.5f)},  // ENE
    {VOLT_TO_ADC(2.721f), DEG_TO_RAD(90.0f)},  // E
    {VOLT_TO_ADC(2.879f), DEG_TO_RAD(112.5f)}, // ESE
    {VOLT_TO_ADC(2.248f), DEG_TO_RAD(135.0f)}, // SE
    {VOLT_TO_ADC(2.538f), DEG_TO_RAD(157.5f)}, // SSE
    {VOLT_TO_ADC(1.803f), DEG_TO_RAD(180.0f)}, // S
    {VOLT_TO_ADC(1.978f), DEG_TO_RAD(202.5f)}, // SSW
    {VOLT_TO_ADC(0.749f), DEG_TO_RAD(225.0f)}, // SW
    {VOLT_TO_ADC(0.824f), DEG_TO_RAD(247.5f)}, // WSW
    {VOLT_TO_ADC(0.124f), DEG_TO_RAD(270.0f)}, // W
    {VOLT_TO_ADC(0.331f), DEG_TO_RAD(292.5f)}, // WNW
    {VOLT_TO_ADC(0.223f), DEG_TO_RAD(315.0f)}, // NW
    {VOLT_TO_ADC(0.584f), DEG_TO_RAD(337.5f)}  // NNW
};

static inline float get_wind_speed_kmh(uint32_t interruptions, float timeSec) {
    return ANEMOMETER_FACTOR * ((float)interruptions / timeSec);
}

static inline float get_rain_mm(uint32_t interrupts) {
    return PLUVIOMETER_FACTOR * interrupts;
}

static inline float get_wind_direction_rad(uint16_t adcValue) {
    for (size_t i = 0; i < ARRAY_LEN(windVaneCalibrations); i++) {
        int diff = (int)adcValue - (int)windVaneCalibrations[i].adcValue;

        if (diff < 0)
            diff = -diff;

        if (diff <= WIND_VANE_TOLERANCE_ADC)
            return windVaneCalibrations[i].directionRad;
    }
    return -1.0f;
}

void wind_direction_compute(windDirection_t *directionData, finalData_t *finalData) {
    if (directionData->count == 0) {
        finalData->valid = false;
        return;
    }

    float meanXRad = directionData->sumXRad / directionData->count;
    float meanYRad = directionData->sumYRad / directionData->count;

    float avgDirDeg = RAD_TO_DEG(atan2f(meanYRad, meanXRad));
    avgDirDeg = fmodf(avgDirDeg + 360.0f, 360.0f);

    finalData->value = avgDirDeg;
    finalData->valid = true;

    directionData->sumYRad = 0.0f;
    directionData->sumXRad = 0.0f;
    directionData->count = 0;
}

void wind_direction_update(windDirection_t *directionData) {
    uint16_t adcRaw = adc_read();
    float dirRad = get_wind_direction_rad(adcRaw);
    if (dirRad < 0.0f) {
        DEBUG_printf("ADC %d out of range\n", adcRaw);
        return;
    }
    // DEBUG_printf("Wind direction: %.2f\n", RAD_TO_DEG(dirRad));
    directionData->lastDirRad = dirRad;
    directionData->sumXRad += cosf(dirRad);
    directionData->sumYRad += sinf(dirRad);
    directionData->count++;
}

weatherIrq_t weatherIrq;

static inline void rain_gauge_compute(finalData_t *finalData) {
    finalData->value = get_rain_mm(weatherIrq.rainfall.interrupts);
    finalData->valid = true;
    weatherIrq.rainfall.interrupts = 0;
}

static inline void wind_speed_average_compute(finalData_t *finalData) {
    finalData->value = get_wind_speed_kmh(weatherIrq.windSpeed.interrupts, COMPUTE_INTERVAL_S);
    finalData->valid = true;
    weatherIrq.windSpeed.interrupts = 0;
}

static inline void peak_wind_update(windDirection_t *windDirection) {
    if (weatherIrq.peakWind.interrupts >= weatherIrq.peakWind.peakInterrupts) {
        weatherIrq.peakWind.peakInterrupts = weatherIrq.peakWind.interrupts;
        if (windDirection->count != 0) {
            weatherIrq.peakWind.peakDirectionRad = windDirection->lastDirRad;
        }
        else {
            weatherIrq.peakWind.peakDirectionRad = -1.0f;
        }
    }
    weatherIrq.peakWind.interrupts = 0;
}

static inline void peak_wind_compute(finalData_t *peakSpeed, finalData_t *peakDirection) {
    peakSpeed->value = get_wind_speed_kmh(weatherIrq.peakWind.peakInterrupts, SAMPLE_INTERVAL_S);
    if (weatherIrq.peakWind.peakDirectionRad >= 0.0f) {
        peakDirection->value = RAD_TO_DEG(weatherIrq.peakWind.peakDirectionRad);
        peakDirection->valid = true;
    }
    else {
        peakDirection->valid = false;
    }
    weatherIrq.peakWind.peakInterrupts = 0;
    weatherIrq.peakWind.peakDirectionRad = 0.0f;
}

static inline void avg_data_update(avgData_t *avgData, float value) {
    avgData->sum += value;
    avgData->count++;
}

static inline void avg_data_compute(avgData_t *avgData, finalData_t *finalData) {
    if (avgData->count == 0) {
        finalData->valid = false;
        return;
    }

    finalData->value = avgData->sum / avgData->count;
    finalData->valid = true;

    avgData->sum = 0.0f;
    avgData->count = 0;
}

// Wind speed and rainfall don't need to be sampled in the callback because they
// are sampled in the IRQ
typedef enum {
    SAMPLE_HDC3022,
    SAMPLE_DPS310,
    SAMPLE_LTR390,
    SAMPLE_PEAK_WIND,
    SAMPLE_WIND_DIRECTION,
    SAMPLE_PEAK_WIND_DIRECTION
} sampleCmd_t;

typedef enum {
    COMPUTE_TEMP,
    COMPUTE_HUMIDITY,
    COMPUTE_PRES,
    COMPUTE_LUX,
    COMPUTE_UVI,
    COMPUTE_WIND_SPEED,
    COMPUTE_PEAK_WIND,
    COMPUTE_WIND_DIRECTION,
    COMPUTE_RAINFALL,
    COMPUTE_TIMESTAMP,
    COMPUTE_SEND_DATA
} computeCmd_t;

queue_t weatherFinalQueue;
queue_t weatherComputeQueue;
queue_t weatherSampleQueue;
queue_t epochTimeQueue;

// NOTE: SAMPLE_WIND_DIRECTION should always be procesed before SAMPLE_PEAK_WIND
bool sample_callback(__unused struct repeating_timer *t) {
    SAMPLE_QUEUE_ADD_CMD(SAMPLE_WIND_DIRECTION);
    SAMPLE_QUEUE_ADD_CMD(SAMPLE_PEAK_WIND);
    SAMPLE_QUEUE_ADD_CMD(SAMPLE_HDC3022);
    SAMPLE_QUEUE_ADD_CMD(SAMPLE_DPS310);
    SAMPLE_QUEUE_ADD_CMD(SAMPLE_LTR390);
    return true;
}

bool compute_callback(__unused struct repeating_timer *t) {
    COMPUTE_QUEUE_ADD_CMD(COMPUTE_RAINFALL);

    COMPUTE_QUEUE_ADD_CMD(COMPUTE_WIND_DIRECTION);
    COMPUTE_QUEUE_ADD_CMD(COMPUTE_WIND_SPEED);

    COMPUTE_QUEUE_ADD_CMD(COMPUTE_PEAK_WIND);

    COMPUTE_QUEUE_ADD_CMD(COMPUTE_TEMP);
    COMPUTE_QUEUE_ADD_CMD(COMPUTE_HUMIDITY);
    COMPUTE_QUEUE_ADD_CMD(COMPUTE_PRES);
    COMPUTE_QUEUE_ADD_CMD(COMPUTE_LUX);
    COMPUTE_QUEUE_ADD_CMD(COMPUTE_UVI);

    COMPUTE_QUEUE_ADD_CMD(COMPUTE_TIMESTAMP);
    COMPUTE_QUEUE_ADD_CMD(COMPUTE_SEND_DATA);
    return true;
}

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == RAIN_GAUGE_PIN) {
        weatherIrq.rainfall.interrupts++;
        DEBUG_printf("Interrupt %ld on rain gauge\n", weatherIrq.rainfall.interrupts);
    }
    else if (gpio == ANEMOMETER_PIN) {
        weatherIrq.windSpeed.interrupts++;
        weatherIrq.peakWind.interrupts++;
        // DEBUG_printf("Interrupt %ld on anemometer\n", windSpeedData.interrupts);
    }
}

void process_sample_queue();
void process_compute_queue();

void core1_entry(void);

bool hdc3022_init(hdc3022_t *hdc3022) {
    if (!hdc3022_init_struct(hdc3022, I2C_BUS, HDC3022_DEFAULT_ADDR, HDC3022_AUTO_MEAS_NONE,
                             true)) { // Auto detect default mode
        DEBUG_printf("Error to init the hdc3022 struct\n");
        return false;
    }
    if (!hdc3022_soft_reset(hdc3022)) {
        ERROR_printf("Error resetting hdc3022\n");
        return false;
    }

    if (!hdc3022_set_auto_meas_mode(hdc3022, HDC3022_MODE)) {
        ERROR_printf("Error setting hdc3022 mode\n");
        return false;
    }

    return true;
}

bool dps310_init(dps310_t *dps310) {
    if (!dps310_init_struct(dps310, DPS310_ADDRESS, I2C_BUS)) {
        ERROR_printf("Error to init the dps310 struct\n");
        return false;
    }
    if (!dps310_wait_until_ready(dps310, false, true)) {
        ERROR_printf("Error setting hdc3022 mode\n");
        return false;
    }
    if (!dps310_soft_reset(dps310)) {
        ERROR_printf("Error resetting dps310\n");
        return false;
    }
    if (!dps310_wait_until_ready(dps310, true, true)) {
        ERROR_printf("Error dps310\n");
        return false;
    }

    if (!dps310_read_temp_calibration(dps310)) {
        ERROR_printf("Error reading dps310 temp calibration\n");
        return false;
    }
    if (!dps310_read_pres_calibration(dps310)) {
        ERROR_printf("Error reading dps310 pressure calibration\n");
        return false;
    }
    if (!dps310_auto_set_temp_source(dps310)) {
        ERROR_printf("Error setting dps310 temp source\n");
        return false;
    }
    if (!dps310_correct_temp(dps310)) {
        ERROR_printf("Error correcting dps310 temperature\n");
        return false;
    }

    if (!dps310_set_pres_oversampling(dps310, DPS310_PRES_OVERSAMPLING))
        return false;
    if (!dps310_set_temp_oversampling(dps310, DPS310_TEMP_OVERSAMPLING))
        return false;

    if (!dps310_set_meas_mode(dps310, DPS310_MODE))
        return false;

    return true;
}

bool ltr390_init(ltr390_t *ltr390) {
    ltr390_init_struct(ltr390, I2C_BUS, LTR390_ADDRESS, I2C_SPEED, NULL, false);
    if (!ltr390_soft_reset(ltr390)) {
        ERROR_printf("Error reseting ltr390\n");
        return false;
    }
    if (!ltr390_set_resolution(ltr390, LTR390_RESOLUTION)) {
        ERROR_printf("Error setting ltr390 resolution\n");
        return false;
    }
    if (!ltr390_set_gain(ltr390, LTR390_GAIN)) {
        ERROR_printf("Error setting ltr390 gain\n");
        return false;
    }
    if (!ltr390_set_meas_rate(ltr390, LTR390_MEAS_RATE)) {
        ERROR_printf("Error setting ltr390 meas rate\n");
        return false;
    }
    if (!ltr390_enable(ltr390, true)) {
        ERROR_printf("Error enabling ltr390\n");
        return false;
    }
}

bool pcf8523_init(pcf8523_t *pcf8523) {
    if (!pcf8523_init_struct(pcf8523, I2C_BUS, PCF8523_ADDRESS, true, false)) {
        ERROR_printf("Error initializating the pcf8523 struct\n");
        return false;
    }
    if (!pcf8523_soft_reset(pcf8523)) {
        ERROR_printf("Error resetting pcf8523\n");
        return false;
    }
    // Wait to oscilator to stabilize
    sleep_ms(2000);
    if (!pcf8523_clear_os_integrity_flag(pcf8523)) {
        ERROR_printf("Error clearing pcf8523 oscilator intergrity flag\n");
        return false;
    }

    return true;
}

bool pcf8523_set(pcf8523_t *pcf8523, uint64_t epochTime) {
    pcf8523_Datetime_t dt = epoch_to_pcf8523_datetime(epochTime);

    if (!pcf8523_set_datetime(pcf8523, &dt)) {
        ERROR_printf("Error setting the datetime\n");
        return false;
    }

    return true;
}

void hdc3022_sample(hdc3022_t *hdc3022, avgData_t *tempAvg, avgData_t *humidityAvg) {
    float temp, humidity;
    if (!hdc3022_readout_auto_meas(hdc3022, HDC3022_AUTO_MEAS_READOUT, HDC3022_TEMP_CELSIUS, &temp,
                                   &humidity)) {
        ERROR_printf("Error doing the HDC3022 readout\n");
        return;
    }
    if (tempAvg)
        avg_data_update(tempAvg, temp);
    if (humidityAvg)
        avg_data_update(humidityAvg, humidity);
}

void dps310_sample(dps310_t *dps310, avgData_t *presAvg) {
    float pres;
    if (!dps310_read_pres(dps310, &pres, true)) {
        ERROR_printf("Error doing the DPS310 readout\n");
        return;
    }
    pres = PA_TO_HPA(pres);
    if (presAvg)
        avg_data_update(presAvg, pres);
}

void ltr390_sample(ltr390_t *ltr390, avgData_t *avgLux, avgData_t *avgUvi) {
    float lux, uvi;
    if (avgLux) {
        if (!ltr390_set_mode(ltr390, LTR390_ALS_MODE))
            return;
        sleep_ms(LTR390_INT_TIME_MS);
        if (!ltr390_read_lux(ltr390, &lux))
            return;
        avg_data_update(avgLux, lux);
    }
    if (avgUvi) {
        if (!ltr390_set_mode(ltr390, LTR390_UVS_MODE))
            return;
        sleep_ms(LTR390_INT_TIME_MS);
        if (!ltr390_read_uvi(ltr390, &uvi))
            return;

        avg_data_update(avgUvi, uvi);
    }
}

void pcf8523_get_epoch(pcf8523_t *pcf8523, uint64_t *epochTime) {
    pcf8523_Datetime_t datetime;
    if (!pcf8523_read_datetime(pcf8523, &datetime)) {
        panic("Error reading timestamp from pcf8523");
    }

    *epochTime = pcf8523_datetime_to_epoch(&datetime, BASE_CENTURY);
}

typedef struct {
    ip_addr_t ntpServerAddress;
    struct udp_pcb *ntpPcb;
    async_at_time_worker_t requestWorker;
    async_at_time_worker_t resendWorker;
    bool epochObtained;
    uint64_t epoch;
} ntp_t;

static void ntp_result(ntp_t* state, int status, time_t *result) {
    async_context_remove_at_time_worker(cyw43_arch_async_context(), &state->resendWorker);
    if (status == 0 && result) {
        state->epoch = *result;
        state->epochObtained = true;
    }
    else {
        hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(),  &state->requestWorker, NTP_TEST_TIME_MS)); // repeat the request in future
        printf("Next request in %ds\n", NTP_TEST_TIME_MS / 1000);
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
    uint8_t *req = (uint8_t *) p->payload;
    memset(req, 0, NTP_MSG_LEN);
    req[0] = 0x1b;
    udp_sendto(state->ntpPcb, p, &state->ntpServerAddress, NTP_PORT);
    pbuf_free(p);
    cyw43_arch_lwip_end();
}

// Call back with a DNS result
static void ntp_dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    ntp_t *state = (ntp_t*)arg;
    if (ipaddr) {
        state->ntpServerAddress = *ipaddr;
        printf("ntp address %s\n", ipaddr_ntoa(ipaddr));
        ntp_request(state);
    } else {
        printf("ntp dns request failed\n");
        ntp_result(state, -1, NULL);
    }
}

// NTP data received
static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    ntp_t *state = (ntp_t*)arg;
    uint8_t mode = pbuf_get_at(p, 0) & 0x7;
    uint8_t stratum = pbuf_get_at(p, 1);

    // Check the result
    if (ip_addr_cmp(addr, &state->ntpServerAddress) && port == NTP_PORT && p->tot_len == NTP_MSG_LEN &&
        mode == 0x4 && stratum != 0) {
        uint8_t seconds_buf[4] = {0};
        pbuf_copy_partial(p, seconds_buf, sizeof(seconds_buf), 40);
        uint32_t seconds_since_1900 = seconds_buf[0] << 24 | seconds_buf[1] << 16 |
                                      seconds_buf[2] << 8  | seconds_buf[3];

        // Rollover adjustment (~2036/2037)
        if (seconds_since_1900 < NTP_DELTA) {
            seconds_since_1900 += 0x100000000; // 2^32
        }

        uint32_t seconds_since_1970 = seconds_since_1900 - NTP_DELTA;
        time_t epoch = (time_t)seconds_since_1970;

        ntp_result(state, 0, &epoch);
    } else {
        printf("invalid ntp response\n");
        ntp_result(state, -1, NULL);
    }
    pbuf_free(p);
}

// Called to make a NTP request
static void request_worker_fn(__unused async_context_t *context, async_at_time_worker_t *worker) {
    ntp_t* state = (ntp_t*)worker->user_data;
    hard_assert(async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &state->resendWorker, NTP_RESEND_TIME_MS)); // in case UDP request is lost
    int err = dns_gethostbyname(NTP_SERVER, &state->ntpServerAddress, ntp_dns_found, state);
    if (err == ERR_OK) {
        ntp_request(state); // Cached DNS result, make NTP request
    } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        printf("dns request failed\n");
        ntp_result(state, -1, NULL);
    }
}

// Called to resend an NTP request if it appears to get lost
static void resend_worker_fn(__unused async_context_t *context, async_at_time_worker_t *worker) {
    ntp_t* state = (ntp_t*)worker->user_data;
    printf("ntp request failed\n");
    ntp_result(state, -1, NULL);
}

static ntp_t* ntp_init(void) {
    ntp_t *state = (ntp_t*)calloc(1, sizeof(ntp_t));
    if (!state) {
        printf("failed to allocate state for ntp\n");
        return NULL;
    }
    state->ntpPcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!state->ntpPcb) {
        printf("failed to create pcb for ntp\n");
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
uint64_t get_ntp_epoch(void) {
    ntp_t *state = ntp_init();
    if (!state)
        return 0;

    hard_assert(async_context_add_at_time_worker_in_ms(
        cyw43_arch_async_context(), &state->requestWorker, 0));

    while (!state->epochObtained) {
        sleep_ms(100);
    }

    uint64_t result = state->epoch;
    free(state);
    return result;
}

int main(void) {
    stdio_init_all();

    DEBUG_printf("Hello :)\n");

    weatherAverage_t weatherAverage = {0};
    weatherFinal_t weatherFinal = {0};
    weatherSensor_t weatherSensor = {0};

    // Queue
    queue_init(&weatherFinalQueue, sizeof(weatherFinal_t), WEATHER_FINAL_QUEUE_SIZE);
    queue_init(&weatherComputeQueue, sizeof(computeCmd_t), WEATHER_COMPUTE_QUEUE_SIZE);
    queue_init(&weatherSampleQueue, sizeof(sampleCmd_t), WEATHER_SAMPLE_QUEUE_SIZE);
    queue_init(&epochTimeQueue, sizeof(uint64_t), 1);

    // Init core 1
    multicore_launch_core1(core1_entry);

    // Rain gauge
    gpio_init(RAIN_GAUGE_PIN);
    gpio_set_dir(RAIN_GAUGE_PIN, GPIO_IN);

    // Anemometer
    gpio_init(ANEMOMETER_PIN);
    gpio_set_dir(ANEMOMETER_PIN, GPIO_IN);

    // I2C
    i2c_init(I2C_BUS, I2C_SPEED);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // RTC
    if (!pcf8523_init(&weatherSensor.pcf8523)) {
        panic("Error initializing pcf8523");
    }

    uint64_t epochTime;
    queue_remove_blocking(&epochTimeQueue, &epochTime);

    if (!pcf8523_set(&weatherSensor.pcf8523, epochTime)) {
        panic("Error setting pcf8523 time");
    }

    // HDC3022
    hdc3022_init(&weatherSensor.hdc3022);

    // DPS310
    dps310_init(&weatherSensor.dps310);

    // LTR390
    ltr390_init(&weatherSensor.ltr390);

    // Wind vane
    adc_init();
    adc_gpio_init(WIND_VANE_PIN);
    adc_select_input(WIND_VANE_ADC_CHANNEL);

    // IRQ
    gpio_set_irq_enabled_with_callback(RAIN_GAUGE_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(ANEMOMETER_PIN, GPIO_IRQ_EDGE_RISE, true);

    // Timers
    struct repeating_timer computeTimer;
    if (!add_repeating_timer_ms(-COMPUTE_INTERVAL_MS, compute_callback, NULL, &computeTimer)) {
        panic("Error creating timer");
    }
    struct repeating_timer sampleTimer;
    if (!add_repeating_timer_ms(-SAMPLE_INTERVAL_MS, sample_callback, NULL, &sampleTimer)) {
        panic("Error creating timer");
    }

    // Main loop
    while (1) {
        process_sample_queue(&weatherAverage, &weatherSensor);
        process_compute_queue(&weatherAverage, &weatherFinal, &weatherSensor);
        // watchdog_update();
    }

    gpio_set_irq_enabled(RAIN_GAUGE_PIN, GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(ANEMOMETER_PIN, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled_with_callback(0, 0, false, NULL);

    cancel_repeating_timer(&computeTimer);
    cancel_repeating_timer(&sampleTimer);

    cyw43_arch_deinit();

    return 0;
}

void process_sample_queue(weatherAverage_t *weatherAverage, weatherSensor_t *weatherSensor) {
    sampleCmd_t cmd;
    while (queue_try_remove(&weatherSampleQueue, &cmd)) {
        switch (cmd) {
        case SAMPLE_WIND_DIRECTION:
            wind_direction_update(&weatherAverage->windDirection);
            break;

        case SAMPLE_PEAK_WIND:
            peak_wind_update(&weatherAverage->windDirection);
            break;

        case SAMPLE_HDC3022:
            hdc3022_sample(&weatherSensor->hdc3022, &weatherAverage->temperature,
                           &weatherAverage->humidity);
            break;

        case SAMPLE_DPS310:
            dps310_sample(&weatherSensor->dps310, &weatherAverage->pressure);
            break;

        case SAMPLE_LTR390:
            ltr390_sample(&weatherSensor->ltr390, &weatherAverage->lux, &weatherAverage->uvi);
            break;
        }
    }
}

void process_compute_queue(weatherAverage_t *weatherAverage, weatherFinal_t *weatherFinal,
                           weatherSensor_t *weatherSensor) {
    computeCmd_t cmd;
    while (queue_try_remove(&weatherComputeQueue, &cmd)) {
        switch (cmd) {
        case COMPUTE_RAINFALL:
            rain_gauge_compute(&weatherFinal->rainfallMM);
            // DEBUG_printf("Rainfall: %.2f mm\n", weatherFinal->rainfallMM.value);
            break;

        case COMPUTE_WIND_SPEED:
            wind_speed_average_compute(&weatherFinal->windSpeedKmH);
            // DEBUG_printf("Average wind speed: %.2f km/h\n", weatherFinal->windSpeedKmH.value);
            break;

        case COMPUTE_WIND_DIRECTION:
            wind_direction_compute(&weatherAverage->windDirection, &weatherFinal->windDirectionDeg);
            // DEBUG_printf("Average wind direction: %.2fº\n",
            // weatherFinal->windDirectionDeg.value);
            break;

        case COMPUTE_PEAK_WIND:
            peak_wind_compute(&weatherFinal->peakWindSpeedKmH, &weatherFinal->peakWindDirectionDeg);
            // DEBUG_printf("Peak wind speed: %.2f km/h\n", weatherFinal->peakWindSpeedKmH.value);
            // DEBUG_printf("Peak wind direction: %.2fº\n",
            // weatherFinal->peakWindDirectionDeg.value);
            break;

        case COMPUTE_TEMP:
            avg_data_compute(&weatherAverage->temperature, &weatherFinal->temperatureCelsius);
            // DEBUG_printf("Temperature: %.2f ºC\n", weatherFinal->temperatureCelsius.value);
            break;

        case COMPUTE_HUMIDITY:
            avg_data_compute(&weatherAverage->humidity, &weatherFinal->humidityRh);
            // DEBUG_printf("Humidity: %.2f Rh\n", weatherFinal->humidityRh.value);
            break;

        case COMPUTE_PRES:
            avg_data_compute(&weatherAverage->pressure, &weatherFinal->pressureHPA);
            // DEBUG_printf("Pressure: %.2f hPa\n", weatherFinal->pressureHPA.value);
            break;

        case COMPUTE_LUX:
            avg_data_compute(&weatherAverage->lux, &weatherFinal->lux);
            // DEBUG_printf("Lux: %.2f\n", weatherFinal->lux.value);
            break;

        case COMPUTE_UVI:
            avg_data_compute(&weatherAverage->uvi, &weatherFinal->uvi);
            // DEBUG_printf("UVI: %.2f\n", weatherFinal->uvi.value);
            break;

        case COMPUTE_TIMESTAMP:
            pcf8523_get_epoch(&weatherSensor->pcf8523, &weatherFinal->epochTime);
            break;

        case COMPUTE_SEND_DATA:
            if (!queue_try_add(&weatherFinalQueue, weatherFinal))
                DEBUG_printf("Final weather queue full\n");
            *weatherFinal = (weatherFinal_t){0};
            break;
        }
    }
}

void core1_entry(void) {
    DEBUG_printf("Hello from core 1 :)\n");

    // CYW43
    if (cyw43_arch_init()) {
        panic("Failed to init cyw43");
    }

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        panic("Failed to connect to the wifi");
    }

    // Set dns servers
    ip_addr_t dns1, dns2;

    ip4addr_aton(DNS_SERVER, &dns1);
    ip4addr_aton(DNS_SERVER_ALT, &dns2);

    dns_setserver(0, &dns1);
    dns_setserver(1, &dns2);

    uint64_t epochTime = get_ntp_epoch();
    queue_add_blocking(&epochTimeQueue, &epochTime);

    weatherFinal_t weatherFinal = {0};
    while (1) {
        queue_remove_blocking(&weatherFinalQueue, &weatherFinal);
        DEBUG_printf("Timestamp epoch start: %lld\n", weatherFinal.epochTime - COMPUTE_INTERVAL_S);
        DEBUG_printf("Timestamp epoch end: %lld\n", weatherFinal.epochTime);
        DEBUG_printf("Rainfall: %.2f mm\n", weatherFinal.rainfallMM.value);
        DEBUG_printf("Average wind speed: %.2f km/h\n", weatherFinal.windSpeedKmH.value);
        DEBUG_printf("Average wind direction: %.2fº\n", weatherFinal.windDirectionDeg.value);
        DEBUG_printf("Peak wind speed: %.2f km/h\n", weatherFinal.peakWindSpeedKmH.value);
        DEBUG_printf("Peak wind direction: %.2fº\n", weatherFinal.peakWindDirectionDeg.value);
        DEBUG_printf("Temperature: %.2f ºC\n", weatherFinal.temperatureCelsius.value);
        DEBUG_printf("Humidity: %.2f Rh\n", weatherFinal.humidityRh.value);
        DEBUG_printf("Pressure: %.2f hPa\n", weatherFinal.pressureHPA.value);
        DEBUG_printf("Lux: %.2f\n", weatherFinal.lux.value);
        DEBUG_printf("UVI: %.2f\n", weatherFinal.uvi.value);
        DEBUG_printf("\n");
    }
}
