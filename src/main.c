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
} circAvgData_t;

typedef struct {
    volatile uint32_t interrupts;
} interruptSensor_t;

typedef struct {
    volatile uint32_t interrupts;
    volatile uint32_t peakInterrupts;
} peakInterruptSensor_t;

typedef struct {
    interruptSensor_t rainfall;
    interruptSensor_t windSpeed;
    peakInterruptSensor_t peakWindSpeed;
    circAvgData_t windDirection;
} weatherInternal_t;

typedef struct {
    finalData_t rainfallMM;
    finalData_t windSpeedKmH;
    finalData_t peakWindSpeedKmH;
    finalData_t windDirectionDeg;
    finalData_t peakWindDirectionDeg;
} weatherData_t;

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
}

static inline void avg_data_reset(avgData_t *avgData) {
    avgData->sum = 0.0f;
    avgData->count = 0;
}

static inline void circ_avg_data_update(circAvgData_t *circAvgData, float dirRad) {
    circAvgData->lastDirRad = dirRad;
    circAvgData->sumXRad += cosf(dirRad);
    circAvgData->sumYRad += sinf(dirRad);
    circAvgData->count++;
}

void circ_avg_data_compute(circAvgData_t *circAvgData, finalData_t *finalData) {
    if (circAvgData->count == 0) {
        finalData->valid = false;
        return;
    }

    float meanXRad = circAvgData->sumXRad / circAvgData->count;
    float meanYRad = circAvgData->sumYRad / circAvgData->count;

    float avgDirDeg = RAD_TO_DEG(atan2f(meanYRad, meanXRad));
    avgDirDeg = fmodf(avgDirDeg + 360.0f, 360.0f);

    finalData->value = avgDirDeg;
    finalData->valid = true;
}

static inline void circ_avg_data_reset(circAvgData_t *circAvgData) {
    circAvgData->sumYRad = 0.0f;
    circAvgData->sumXRad = 0.0f;
    circAvgData->count = 0;
}

static inline void rain_gauge_compute(interruptSensor_t *rainData, finalData_t *finalData) {
    finalData->value = get_rain_mm(rainData->interrupts);
    finalData->valid = true;
}

static inline void wind_speed_compute(interruptSensor_t *speedData, finalData_t *finalData) {
    finalData->value = get_wind_speed_kmh(speedData->interrupts, COMPUTE_INTERVAL_S);
    finalData->valid = true;
}

static inline void interrupt_data_reset(interruptSensor_t *intData) {
    intData->interrupts = 0;
}

static inline void peak_wind_speed_compute(peakInterruptSensor_t *peakSpeedData,
                                           finalData_t *finalData) {
    finalData->value = get_wind_speed_kmh(peakSpeedData->peakInterrupts, SAMPLE_INTERVAL_S);
    finalData->valid = true;
}

static inline void peak_interrupt_data_update(peakInterruptSensor_t *intData) {
    if (intData->interrupts > intData->peakInterrupts) {
        intData->peakInterrupts = intData->interrupts;
    }
    intData->interrupts = 0;
}

static inline void peak_wind_and_direction_update(peakInterruptSensor_t *intData,
                                                  circAvgData_t *windVane, finalData_t *finalData) {
    if (intData->interrupts >= intData->peakInterrupts) {
        intData->peakInterrupts = intData->interrupts;
        if (windVane->count != 0) {
            finalData->value = RAD_TO_DEG(windVane->lastDirRad);
            finalData->valid = true;
        }
    }
    intData->interrupts = 0;
}

static inline void peak_interrupt_data_reset(peakInterruptSensor_t *intData) {
    intData->interrupts = 0;
    intData->peakInterrupts = 0;
}

void wind_direction_update(circAvgData_t *directionData) {
    uint16_t adcRaw = adc_read();
    float dirRad = get_wind_direction_rad(adcRaw);
    if (dirRad < 0.0f) {
        DEBUG_printf("ADC %d out of range\n", adcRaw);
        return;
    }
    // DEBUG_printf("Wind direction: %.2f\n", RAD_TO_DEG(dirRad));

    circ_avg_data_update(directionData, dirRad);
}

static inline void final_data_reset(finalData_t *finalData) {
    finalData->valid = false;
}

// Wind speed and rainfall don't need to be sampled in the callback because they
// are sampled in the IRQ
typedef enum {
    SAMPLE_TEMP = (1 << 0),
    SAMPLE_HUMIDITY = (1 << 1),
    SAMPLE_PRES = (1 << 2),
    SAMPLE_LUX = (1 << 3),
    SAMPLE_UV = (1 << 3),
    SAMPLE_PEAK_WIND = (1 << 4),
    SAMPLE_WIND_DIRECTION = (1 << 5),
    SAMPLE_PEAK_WIND_DIRECTION = (1 << 6)
} sampleFlags_t;

typedef enum {
    COMPUTE_TEMP = (1 << 0),
    COMPUTE_HUMIDITY = (1 << 1),
    COMPUTE_PRES = (1 << 2),
    COMPUTE_LUX = (1 << 3),
    COMPUTE_UV = (1 << 4),
    COMPUTE_WIND_SPEED = (1 << 5),
    COMPUTE_PEAK_WIND_SPEED = (1 << 6),
    COMPUTE_WIND_DIRECTION = (1 << 7),
    COMPUTE_RAINFALL = (1 << 8)
} computeFlags_t;

uint32_t sampleFlags = 0;
uint32_t computeFlags = 0;

// NOTE: SAMPLE_WIND_DIRECTION should always be procesed before SAMPLE_PEAK_WIND
bool sample_callback(__unused struct repeating_timer *t) {
    sampleFlags |= SAMPLE_WIND_DIRECTION;
    sampleFlags |= SAMPLE_PEAK_WIND;
    return true;
}

bool compute_callback(__unused struct repeating_timer *t) {
    DEBUG_printf("\n");
    computeFlags |= COMPUTE_RAINFALL;
    computeFlags |= COMPUTE_WIND_DIRECTION;
    computeFlags |= COMPUTE_WIND_SPEED;
    computeFlags |= COMPUTE_PEAK_WIND_SPEED;
    return true;
}

weatherData_t weatherData = {0};
weatherInternal_t weatherInternal = {0};

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == RAIN_GAUGE_PIN) {
        weatherInternal.rainfall.interrupts++;
        DEBUG_printf("Interrupt %ld on rain gauge\n", weatherInternal.rainfall.interrupts);
    }
    else if (gpio == ANEMOMETER_PIN) {
        weatherInternal.windSpeed.interrupts++;
        weatherInternal.peakWindSpeed.interrupts++;
        // DEBUG_printf("Interrupt %ld on anemometer\n", windSpeedData.interrupts);
    }
}

void process_sample_flags();
void process_compute_flags();

int main(void) {
    stdio_init_all();

    // if (watchdog_enable_caused_reboot())
    //     DEBUG_printf("Rebooted by Watchdog\n");
    // else
    //     DEBUG_printf("Clean boot\n");

    // watchdog_enable(WATCHDOG_INTERVAL_MS,
    //                 true); // Stop watchdog while stepping through code

    printf("Hello\n");

    gpio_init(RAIN_GAUGE_PIN);
    gpio_set_dir(RAIN_GAUGE_PIN, GPIO_IN);

    gpio_init(ANEMOMETER_PIN);
    gpio_set_dir(ANEMOMETER_PIN, GPIO_IN);

    adc_init();
    adc_gpio_init(WIND_VANE_PIN);
    adc_select_input(WIND_VANE_ADC_CHANNEL);

    gpio_set_irq_enabled_with_callback(RAIN_GAUGE_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(ANEMOMETER_PIN, GPIO_IRQ_EDGE_RISE, true);

    struct repeating_timer computeTimer;
    if (!add_repeating_timer_ms(-COMPUTE_INTERVAL_MS, compute_callback, NULL, &computeTimer)) {
        panic("Error creating timer");
    }
    struct repeating_timer sampleTimer;
    if (!add_repeating_timer_ms(-SAMPLE_INTERVAL_MS, sample_callback, NULL, &sampleTimer)) {
        panic("Error creating timer");
    }

    while (1) {
        process_sample_flags();
        process_compute_flags();
        // watchdog_update();
    }

    return 0;
}

// NOTE: SAMPLE_PEAK_WIND should always be procesed before SAMPLE_PEAK_WIND_DIRECTION
void process_sample_flags() {
    if (sampleFlags & SAMPLE_WIND_DIRECTION) {
        sampleFlags &= ~SAMPLE_WIND_DIRECTION;

        wind_direction_update(&weatherInternal.windDirection);
    }
    if (sampleFlags & SAMPLE_PEAK_WIND) {
        sampleFlags &= ~SAMPLE_PEAK_WIND;

        peak_wind_and_direction_update(&weatherInternal.peakWindSpeed,
                                       &weatherInternal.windDirection,
                                       &weatherData.peakWindDirectionDeg);
    }
}

void process_compute_flags() {
    if (computeFlags & COMPUTE_RAINFALL) {
        computeFlags &= ~COMPUTE_RAINFALL;

        rain_gauge_compute(&weatherInternal.rainfall, &weatherData.rainfallMM);
        interrupt_data_reset(&weatherInternal.rainfall);

        DEBUG_printf("Rainfall: %f mm\n", weatherData.rainfallMM.value);
    }
    if (computeFlags & COMPUTE_WIND_SPEED) {
        computeFlags &= ~COMPUTE_WIND_SPEED;

        wind_speed_compute(&weatherInternal.windSpeed, &weatherData.windSpeedKmH);
        interrupt_data_reset(&weatherInternal.windSpeed);

        DEBUG_printf("Average wind speed: %f km/h\n", weatherData.windSpeedKmH.value);
    }
    if (computeFlags & COMPUTE_PEAK_WIND_SPEED) {
        computeFlags &= ~COMPUTE_PEAK_WIND_SPEED;

        peak_wind_speed_compute(&weatherInternal.peakWindSpeed, &weatherData.peakWindSpeedKmH);
        peak_interrupt_data_reset(&weatherInternal.peakWindSpeed);

        DEBUG_printf("Peak wind speed: %f km/h\n", weatherData.peakWindSpeedKmH.value);
        DEBUG_printf("Peak wind direction: %fº\n", weatherData.peakWindDirectionDeg.value);
    }
    if (computeFlags & COMPUTE_WIND_DIRECTION) {
        computeFlags &= ~COMPUTE_WIND_DIRECTION;

        circ_avg_data_compute(&weatherInternal.windDirection, &weatherData.windDirectionDeg);
        circ_avg_data_reset(&weatherInternal.windDirection);

        DEBUG_printf("Average wind direction: %.2fº\n", weatherData.windDirectionDeg.value);
    }
}
