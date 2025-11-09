#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include "sensor/dps310.h"
#include "sensor/hdc3022.h"
#include "sensor/ltr390.h"

#include <math.h>
#include <stdio.h>

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
#define ANEMOMETER_FACTOR 2.4f
#define PLUVIOMETER_FACTOR 0.2794f

#define ADC_VREF 3.3f // RPI Pico VREF
#define ADC_BITS 12   // RPI Pico ADC Resolution

#define WIND_VANE_TOLERANCE_V 0.05f
#define WIND_VANE_TOLERANCE_ADC (VOLT_TO_ADC(WIND_VANE_TOLERANCE_V))

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
    SAMPLE_HDC3022 = (1 << 0),
    SAMPLE_DPS310 = (1 << 1),
    SAMPLE_LTR390 = (1 << 2),
    SAMPLE_PEAK_WIND = (1 << 3),
    SAMPLE_WIND_DIRECTION = (1 << 4),
    SAMPLE_PEAK_WIND_DIRECTION = (1 << 5)
} sampleFlags_t;

typedef enum {
    COMPUTE_TEMP = (1 << 0),
    COMPUTE_HUMIDITY = (1 << 1),
    COMPUTE_PRES = (1 << 2),
    COMPUTE_LUX = (1 << 3),
    COMPUTE_UVI = (1 << 4),
    COMPUTE_WIND_SPEED = (1 << 5),
    COMPUTE_PEAK_WIND = (1 << 6),
    COMPUTE_WIND_DIRECTION = (1 << 7),
    COMPUTE_RAINFALL = (1 << 8)
} computeFlags_t;

uint32_t sampleFlags = 0;
uint32_t computeFlags = 0;

// NOTE: SAMPLE_WIND_DIRECTION should always be procesed before SAMPLE_PEAK_WIND
bool sample_callback(__unused struct repeating_timer *t) {
    sampleFlags |= SAMPLE_WIND_DIRECTION;
    sampleFlags |= SAMPLE_PEAK_WIND;
    sampleFlags |= SAMPLE_HDC3022;
    sampleFlags |= SAMPLE_DPS310;
    sampleFlags |= SAMPLE_LTR390;
    return true;
}

bool compute_callback(__unused struct repeating_timer *t) {
    DEBUG_printf("\n"); // Separate meas intervals with \n
    computeFlags |= COMPUTE_RAINFALL;
    computeFlags |= COMPUTE_WIND_DIRECTION;
    computeFlags |= COMPUTE_WIND_SPEED;
    computeFlags |= COMPUTE_PEAK_WIND;
    computeFlags |= COMPUTE_TEMP;
    computeFlags |= COMPUTE_HUMIDITY;
    computeFlags |= COMPUTE_PRES;
    computeFlags |= COMPUTE_LUX;
    computeFlags |= COMPUTE_UVI;
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

void process_sample_flags();
void process_compute_flags();

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

int main(void) {
    stdio_init_all();

    DEBUG_printf("Hello :)\n");

    weatherAverage_t weatherAverage = {0};
    weatherFinal_t weatherFinal = {0};
    weatherSensor_t weatherSensor = {0};

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

    // HDC3022
    hdc3022_init(&weatherSensor.hdc3022);
    dps310_init(&weatherSensor.dps310);
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
        process_sample_flags(&weatherAverage, &weatherSensor);
        process_compute_flags(&weatherAverage, &weatherFinal);
        // watchdog_update();
    }

    return 0;
}

// NOTE: SAMPLE_WIND_DIRECTION should always be procesed before SAMPLE_PEAK_WIND
void process_sample_flags(weatherAverage_t *weatherAverage, weatherSensor_t *weatherSensor) {
    if (sampleFlags & SAMPLE_WIND_DIRECTION) {
        sampleFlags &= ~SAMPLE_WIND_DIRECTION;

        wind_direction_update(&weatherAverage->windDirection);
    }
    if (sampleFlags & SAMPLE_PEAK_WIND) {
        sampleFlags &= ~SAMPLE_PEAK_WIND;

        peak_wind_update(&weatherAverage->windDirection);
    }
    if (sampleFlags & SAMPLE_HDC3022) {
        sampleFlags &= ~SAMPLE_HDC3022;
        float temp, humidity;
        if (!hdc3022_readout_auto_meas(&weatherSensor->hdc3022, HDC3022_AUTO_MEAS_READOUT,
                                       HDC3022_TEMP_CELSIUS, &temp, &humidity)) {
            ERROR_printf("Error doing the HDC3022 readout\n");
            return;
        }
        avg_data_update(&weatherAverage->temperature, temp);
        avg_data_update(&weatherAverage->humidity, humidity);
    }

    if (sampleFlags & SAMPLE_DPS310) {
        sampleFlags &= ~SAMPLE_DPS310;

        float pres;
        if (!dps310_read_pres(&weatherSensor->dps310, &pres, true)) {
            ERROR_printf("Error doing the DPS310 readout\n");
            return;
        }
        pres = PA_TO_HPA(pres);
        avg_data_update(&weatherAverage->pressure, pres);
    }
    if (sampleFlags & SAMPLE_LTR390) {
        sampleFlags &= ~SAMPLE_LTR390;

        float lux, uvi;

        if (!ltr390_set_mode(&weatherSensor->ltr390, LTR390_ALS_MODE))
            return;
        sleep_ms(LTR390_INT_TIME_MS);
        if (!ltr390_read_lux(&weatherSensor->ltr390, &lux))
            return;

        if (!ltr390_set_mode(&weatherSensor->ltr390, LTR390_UVS_MODE))
            return;
        sleep_ms(LTR390_INT_TIME_MS);
        if (!ltr390_read_uvi(&weatherSensor->ltr390, &uvi))
            return;

        avg_data_update(&weatherAverage->lux, lux);
        avg_data_update(&weatherAverage->uvi, uvi);
    }
}

void process_compute_flags(weatherAverage_t *weatherAverage, weatherFinal_t *weatherFinal) {
    if (computeFlags & COMPUTE_RAINFALL) {
        computeFlags &= ~COMPUTE_RAINFALL;

        rain_gauge_compute(&weatherFinal->rainfallMM);
        DEBUG_printf("Rainfall: %.2f mm\n", weatherFinal->rainfallMM.value);
    }
    if (computeFlags & COMPUTE_WIND_SPEED) {
        computeFlags &= ~COMPUTE_WIND_SPEED;

        wind_speed_average_compute(&weatherFinal->windSpeedKmH);
        DEBUG_printf("Average wind speed: %.2fº\n", weatherFinal->windSpeedKmH.value);
    }
    if (computeFlags & COMPUTE_WIND_DIRECTION) {
        computeFlags &= ~COMPUTE_WIND_DIRECTION;

        wind_direction_compute(&weatherAverage->windDirection, &weatherFinal->windDirectionDeg);
        DEBUG_printf("Average wind direction: %.2fº\n", weatherFinal->windDirectionDeg.value);
    }
    if (computeFlags & COMPUTE_PEAK_WIND) {
        computeFlags &= ~COMPUTE_PEAK_WIND;

        peak_wind_compute(&weatherFinal->peakWindSpeedKmH, &weatherFinal->peakWindDirectionDeg);
        DEBUG_printf("Peak wind speed: %.2f km/h\n", weatherFinal->peakWindSpeedKmH.value);
        DEBUG_printf("Peak wind direction: %.2fº\n", weatherFinal->peakWindDirectionDeg.value);
    }
    if (computeFlags & COMPUTE_TEMP) {
        computeFlags &= ~COMPUTE_TEMP;

        avg_data_compute(&weatherAverage->temperature, &weatherFinal->temperatureCelsius);
        DEBUG_printf("Temperature: %.2f ºC\n", weatherFinal->temperatureCelsius.value);
    }
    if (computeFlags & COMPUTE_HUMIDITY) {
        computeFlags &= ~COMPUTE_HUMIDITY;

        avg_data_compute(&weatherAverage->humidity, &weatherFinal->humidityRh);
        DEBUG_printf("Humidity: %.2f Rh\n", weatherFinal->humidityRh.value);
    }
    if (computeFlags & COMPUTE_PRES) {
        computeFlags &= ~COMPUTE_PRES;

        avg_data_compute(&weatherAverage->pressure, &weatherFinal->pressureHPA);
        DEBUG_printf("Pressure: %.2f hPa\n", weatherFinal->pressureHPA.value);
    }
    if (computeFlags & COMPUTE_LUX) {
        computeFlags &= ~COMPUTE_LUX;

        avg_data_compute(&weatherAverage->lux, &weatherFinal->lux);
        DEBUG_printf("Lux: %.2f\n", weatherFinal->lux.value);
    }
    if (computeFlags & COMPUTE_UVI) {
        computeFlags &= ~COMPUTE_UVI;

        avg_data_compute(&weatherAverage->uvi, &weatherFinal->uvi);
        DEBUG_printf("UVI: %.2f\n", weatherFinal->uvi.value);
    }
}
