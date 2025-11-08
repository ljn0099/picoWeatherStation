#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

#define ARRAY_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))

#define ADC_VREF 3.3f
#define ADC_BITS 12
#define VOLT_TO_ADC(v) ((uint16_t)((v) * ((1 << ADC_BITS) - 1) / ADC_VREF))
#define ADC_TO_VOLT(adc) ((float)(adc) * ADC_VREF / ((1 << ADC_BITS) - 1))

#define MEAS_TIME_MS 10000  // 10 sec
#define SAMPLE_TIME_MS 3000 // 3 sec

#define WIND_VANE_PIN 26
#define WIND_VANE_ADC_CHANNEL 0

#define TOLERANCE_V 0.05f
#define TOLERANCE_ADC VOLT_TO_ADC(TOLERANCE_V)

#define RAD_TO_DEG(x) ((x) * 180.0f / M_PI)
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0f)

typedef struct {
    uint16_t adcValue;
    float direction;
} WindVaneCalibration_t;

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

/*
 *                 (0.0)
 *                   N
 *      (337.5)              (22.5)
 *            NNW          NNE
 *   (315.0)                    (45.0)
 *         NW                  NE
 * (292.5)                        (67.5)
 *      WNW                      ENE
 *
 *(270.0)W                        E(90.0)
 *
 *      WSW                      ESE
 *(247.5)                          (112.5)
 *         SW                  SE
 *   (225.0)                    (135.0)
 *            SSW          SSE
 *       (202.5)            (157.5)
 *                   S
 *                (180.0)
 */

typedef struct {
    float sumX;
    float sumY;
    uint32_t count;
    float meanDir;
} WindVaneData_t;

WindVaneData_t windVaneData = {0};

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

void wind_average_update(uint16_t adcRaw) {
    float dir = get_wind_direction(adcRaw);
    if (dir < 0.0f)
        return;

    printf("Direction: %.2f\n", RAD_TO_DEG(dir));

    windVaneData.sumX += cosf(dir);
    windVaneData.sumY += sinf(dir);
    windVaneData.count++;
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

uint16_t read_adc_windVaneData(size_t n) {
    uint32_t sum = 0;
    for (size_t i = 0; i < n; i++) {
        sum += adc_read();
    }
    return sum / n;
}

bool wind_vane_callback(__unused struct repeating_timer *t) {
    wind_average_compute();
    return true;
}

bool wind_vane_meas_callback(__unused struct repeating_timer *t) {
    uint16_t adcRaw = adc_read();
    wind_average_update(adcRaw);
    return true;
}

int main() {
    stdio_init_all();
    printf("Hello wind vane\n");

    adc_init();
    adc_gpio_init(WIND_VANE_PIN);
    adc_select_input(WIND_VANE_ADC_CHANNEL);

    struct repeating_timer timer;
    add_repeating_timer_ms(-MEAS_TIME_MS, wind_vane_callback, NULL, &timer);
    struct repeating_timer timer2;
    add_repeating_timer_ms(-SAMPLE_TIME_MS, wind_vane_meas_callback, NULL, &timer2);

    // Wait forever
    while (1) {
        // uint16_t adcRaw = adc_read();
        // uint16_t adcRaw = read_adc_windVaneData(6);
        // float direction = get_wind_direction(adcRaw);

        // printf("Voltage: %f V, direction: %.2fº\n", ADC_TO_VOLT(adcRaw), RAD_TO_DEG(direction));
        // sleep_ms(500);

        tight_loop_contents();
    }
}
