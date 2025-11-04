#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#define ARRAY_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))

#define ADC_VREF 3.3f
#define ADC_BITS 12
#define VOLT_TO_ADC(v) ((uint16_t)((v) * ((1 << ADC_BITS) - 1) / ADC_VREF))
#define ADC_TO_VOLT(adc) ((float)(adc) * ADC_VREF / ((1 << ADC_BITS) - 1))

#define TOLERANCE_V 0.05f
#define TOLERANCE_ADC VOLT_TO_ADC(TOLERANCE_V)

#define RAD_TO_DEG(x) ((x) * 180.0f / M_PI)
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0f)

#define MEAS_TIME_MS 10000 // 10 sec
#define SAMPLE_TIME_MS 3000 // 3 sec
#define MEAS_TIME_S (MEAS_TIME_MS / 1000)
#define SAMPLE_TIME_S (SAMPLE_TIME_MS / 1000)

#define ANEMOMETER_PIN 22
#define WIND_VANE_PIN 26
#define WIND_VANE_ADC_CHANNEL 0

typedef struct {
    volatile uint32_t interrupts;
    volatile uint32_t peakInterrupts;
    uint32_t peakInterruptsBuf;

    float averageSpeed;
    float peakSpeed;
} anemometerData_t;

typedef struct {
    uint16_t adcValue;
    float direction;
} WindVaneCalibration_t;

typedef struct {
    float sumX;
    float sumY;
    uint32_t count;
    float meanDir;
    float peakDir;
} WindVaneData_t;

const WindVaneCalibration_t windVaneCalibrations[] = {
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

anemometerData_t anemometerData = {0};

WindVaneData_t windVaneData = {0};

float get_wind_speed_kmh(uint32_t interruptions, uint32_t timeSec) {
    return 2.4 * ((float)interruptions / timeSec);
}

float get_wind_direction(uint16_t adcValue) {
    for (size_t i = 0; i < ARRAY_LEN(windVaneCalibrations); i++) {
        int diff = (int)adcValue - (int)windVaneCalibrations[i].adcValue;

        if (diff < 0)
            diff = -diff;

        if (diff <= TOLERANCE_ADC)
            return windVaneCalibrations[i].direction;
    }
    return -1.0f;
}

void anemometer_meas() {
    anemometerData.averageSpeed = get_wind_speed_kmh(anemometerData.interrupts, MEAS_TIME_S);
    anemometerData.peakSpeed = get_wind_speed_kmh(anemometerData.peakInterrupts, SAMPLE_TIME_S);

    printf("\n");
    printf("Interrupts: %ld\n", anemometerData.interrupts);
    printf("Average wind speed: %f\n", anemometerData.averageSpeed);
    printf("Peak wind speed: %f\n", anemometerData.peakSpeed);
    printf("Peak wind direction: %f\n", windVaneData.peakDir);
    printf("\n");

    anemometerData.interrupts = 0;
    anemometerData.peakInterrupts = 0;
}

float wind_average_update() {
    uint16_t adcRaw = adc_read();
    float dir = get_wind_direction(adcRaw);
    if (dir < 0.0f)
        return -1.0f;
    printf("Updated to: %.2f\n", RAD_TO_DEG(dir));

    windVaneData.sumX += cosf(dir);
    windVaneData.sumY += sinf(dir);
    windVaneData.count++;
    return dir;
}

float wind_average_compute(void) {
    if (windVaneData.count == 0)
        return -1.0f;

    float meanX = windVaneData.sumX / windVaneData.count;
    float meanY = windVaneData.sumY / windVaneData.count;

    float meanDir = RAD_TO_DEG(atan2f(meanY, meanX));
    if (meanDir < 0)
        meanDir += 360.0f;
    windVaneData.meanDir = meanDir;

    printf("Mean 10sec: %.2f° with %lu samples\n", windVaneData.meanDir, windVaneData.count);

    windVaneData.sumX = windVaneData.sumY = 0.0f;
    windVaneData.count = 0;

    return meanDir;
}

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == ANEMOMETER_PIN) {
        anemometerData.interrupts++;
        anemometerData.peakInterruptsBuf++;
    }
}

bool meas_callback(__unused struct repeating_timer *t) {
    anemometer_meas();
    wind_average_compute();
    return true;
}

bool sample_callback(__unused struct repeating_timer *t) {
    float dir = wind_average_update();

    if (anemometerData.peakInterruptsBuf > anemometerData.peakInterrupts) {
        anemometerData.peakInterrupts = anemometerData.peakInterruptsBuf;
        if (dir >= 0.0f)
            windVaneData.peakDir = RAD_TO_DEG(dir);
    }
    anemometerData.peakInterruptsBuf = 0;

    return true;
}

int main() {
    stdio_init_all();
    printf("Hello wind\n");

    gpio_init(ANEMOMETER_PIN);
    gpio_set_dir(ANEMOMETER_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ANEMOMETER_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    adc_init();
    adc_gpio_init(WIND_VANE_PIN);
    adc_select_input(WIND_VANE_ADC_CHANNEL);

    struct repeating_timer measTimer;
    add_repeating_timer_ms(-MEAS_TIME_MS, meas_callback, NULL, &measTimer);
    struct repeating_timer sampleTimer;
    add_repeating_timer_ms(-SAMPLE_TIME_MS, sample_callback, NULL, &sampleTimer);

    // Wait forever
    while (1) {
        tight_loop_contents();
    }
}
