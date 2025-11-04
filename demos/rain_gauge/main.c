#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define MEAS_TIME_MS 10000 // 10 sec

#define RAIN_GAUGE_PIN 21

typedef struct {
    volatile uint32_t interrupts;
    float rainMM;
} rainGauge_t;

rainGauge_t rainGaugeData;

float get_rain_mm(uint32_t interrupts) {
    return 0.2794 * interrupts;
}

void gpio_callback(uint gpio, uint32_t events) {
    rainGaugeData.interrupts++;
    printf("Interrupt: %ld\n", rainGaugeData.interrupts);
}

bool rain_gauge_callback(__unused struct repeating_timer *t) {
    rainGaugeData.rainMM = get_rain_mm(rainGaugeData.interrupts);

    printf("Interrupts: %ld\n", rainGaugeData.interrupts);
    printf("Rain mm: %f\n", rainGaugeData.rainMM);

    rainGaugeData.interrupts = 0;
    return true;
}

int main() {
    stdio_init_all();
    printf("Hello rain gauge\n");

    rainGaugeData.interrupts = 0;
    rainGaugeData.rainMM = 0.0f;

    gpio_init(RAIN_GAUGE_PIN);
    gpio_set_dir(RAIN_GAUGE_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(RAIN_GAUGE_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    struct repeating_timer timer;
    add_repeating_timer_ms(-MEAS_TIME_MS, rain_gauge_callback, NULL, &timer);

    // Wait forever
    while (1) {
        tight_loop_contents();
    }
}
