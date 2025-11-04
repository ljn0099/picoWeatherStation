#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define AVERAGE_TIME_MS 10000 // 10 sec
// #define AVERAGE_TIME_MS 600000 // 10 min
#define PEAK_TIME_MS 3000 // 3 sec
#define ANEMOMETER_PIN 22

#define AVERAGE_TIME_S (AVERAGE_TIME_MS / 1000)
#define PEAK_TIME_S (PEAK_TIME_MS / 1000)

typedef struct {
    volatile uint32_t interrupts;
    volatile uint32_t peakInterruptsBuf;
    volatile uint32_t peakInterrupts;

    float averageSpeed;
    float peakSpeed;
} anemometerData_t;

anemometerData_t anemometerData;

float get_wind_speed_kmh(uint32_t interruptions, uint32_t timeSec) {
    return 2.4 * ((float)interruptions / timeSec);
}

void gpio_callback(uint gpio, uint32_t events) {
    anemometerData.interrupts++;
    anemometerData.peakInterruptsBuf++;
}

bool anemometer_callback(__unused struct repeating_timer *t) {
    anemometerData.averageSpeed = get_wind_speed_kmh(anemometerData.interrupts, AVERAGE_TIME_S);
    anemometerData.peakSpeed = get_wind_speed_kmh(anemometerData.peakInterrupts, PEAK_TIME_S);

    printf("\n");
    printf("Interrupts: %ld\n", anemometerData.interrupts);
    printf("Average wind speed: %f\n", anemometerData.averageSpeed);
    printf("Peak wind speed: %f\n", anemometerData.peakSpeed);
    printf("\n");

    anemometerData.interrupts = 0;
    anemometerData.peakInterrupts = 0;
    return true;
}

bool anemometer_peak_callback(__unused struct repeating_timer *t) {
    if (anemometerData.peakInterruptsBuf > anemometerData.peakInterrupts)
        anemometerData.peakInterrupts = anemometerData.peakInterruptsBuf;
    anemometerData.peakInterruptsBuf = 0;
}

int main() {
    stdio_init_all();
    printf("Hello wind vane\n");

    anemometerData.interrupts = 0;
    anemometerData.peakInterrupts = 0;
    anemometerData.peakInterruptsBuf = 0;

    gpio_init(ANEMOMETER_PIN);
    gpio_set_dir(ANEMOMETER_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ANEMOMETER_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    struct repeating_timer timer;
    add_repeating_timer_ms(-AVERAGE_TIME_MS, anemometer_callback, NULL, &timer);
    struct repeating_timer timer2;
    add_repeating_timer_ms(-PEAK_TIME_MS, anemometer_peak_callback, NULL, &timer2);

    // Wait forever
    while (1) {
        tight_loop_contents();
    }
}
