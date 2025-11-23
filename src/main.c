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

#include "include/queues.h"
#include "include/weather_types.h"
#include "include/ntp.h"
#include "include/mqtt.h"
#include "include/utils.h"
#include "include/weather_protocol.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

// DNS
#define DNS_SERVER "1.1.1.1"
#define DNS_SERVER_ALT "1.0.0.1"

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
    COMPUTE_QUEUE_ADD_CMD(COMPUTE_SOLAR_IRRADIANCE);

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

void ltr390_sample(ltr390_t *ltr390, avgData_t *avgLux, avgData_t *avgUvi, avgData_t *avgSolar) {
    if (avgLux || avgSolar) {
        float lux;
        if (!ltr390_set_mode(ltr390, LTR390_ALS_MODE))
            return;
        sleep_ms(LTR390_INT_TIME_MS);
        if (!ltr390_read_lux(ltr390, &lux))
            return;

        if (avgLux)
            avg_data_update(avgLux, lux);
        if (avgSolar) {
            float solarIrradiance = LUX_TO_WM2(lux);

            avg_data_update(avgSolar, solarIrradiance);
        }
    }
    if (avgUvi) {
        float uvi;
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

int main(void) {
    stdio_init_all();

    DEBUG_printf("Hello :)\n");

    weatherAverage_t weatherAverage = {0};
    weatherFinal_t weatherFinal = {0};
    weatherSensor_t weatherSensor = {0};

    // Queue
    queues_init();

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
            ltr390_sample(&weatherSensor->ltr390, &weatherAverage->lux, &weatherAverage->uvi, &weatherAverage->solarIrradiance);
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

        case COMPUTE_SOLAR_IRRADIANCE:
            avg_data_compute(&weatherAverage->solarIrradiance, &weatherFinal->solarIrradianceWm2);
            // DEBUG_printf("Solar irradiance: %.2f Wm/2\n", weatherFinal->solarIrradianceWm2.value);
            break;

        case COMPUTE_TIMESTAMP:
            pcf8523_get_epoch(&weatherSensor->pcf8523, &weatherFinal->epochTimeEnd);
            weatherFinal->epochTimeStart = weatherFinal->epochTimeEnd - COMPUTE_INTERVAL_S;
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

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK,
                                           30000)) {
        panic("Failed to connect to the wifi");
    }

    // Set dns servers
    ip_addr_t dns1, dns2;

    ip4addr_aton(DNS_SERVER, &dns1);
    ip4addr_aton(DNS_SERVER_ALT, &dns2);

    dns_setserver(0, &dns1);
    dns_setserver(1, &dns2);

    ntp_start_request();

    mqtt_start();

    weatherFinal_t weatherFinal = {0};
    while (1) {
        queue_remove_blocking(&weatherFinalQueue, &weatherFinal);
        DEBUG_printf("Timestamp epoch start: %lld\n", weatherFinal.epochTimeStart);
        DEBUG_printf("Timestamp epoch end: %lld\n", weatherFinal.epochTimeEnd);
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
        DEBUG_printf("Solar irradiance: %.2f Wm/2\n", weatherFinal.solarIrradianceWm2.value);
        DEBUG_printf("\n");

        size_t payloadLen;
        uint8_t *payload = create_weather_payload(&weatherFinal, &payloadLen);

        if (!payload) {
            ERROR_printf("Error generating payload");
            continue;
        }

        DEBUG_printf("Payload length: %zu bytes\n", payloadLen);
        DEBUG_printf("Payload (hex): ");
        for (size_t i = 0; i < payloadLen; i++)
            DEBUG_printf("%02X ", payload[i]);
        DEBUG_printf("\n");
        while (!mqtt_publish_blocking("/data", payload, sizeof(payload))) {
            DEBUG_printf("Error publishing payload\n");
            sleep_ms(1000);
        }
        free(payload);
    }

    cyw43_arch_deinit();
}
