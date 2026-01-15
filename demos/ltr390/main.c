#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

#include "hardware/i2c.h"
#include "sensor/ltr390.h"

#define I2C_BUS i2c1
#define I2C_SDA 18
#define I2C_SCL 19
#define I2C_ADDRESS LTR390_DEFAULT_ADDR
#define I2C_SPEED 100000

#define MEAS_TIME_MS 10000 // 10 sec
#define SAMPLE_TIME_MS 3000 // 3 sec
#define MEAS_TIME_S (MEAS_TIME_MS / 1000)
#define SAMPLE_TIME_S (SAMPLE_TIME_MS / 1000)

#define LTR390_MEAS_RATE LTR390_MEAS_RATE_100MS
#define LTR390_GAIN LTR390_GAIN_3
#define LTR390_RESOLUTION LTR390_RES_18BIT
#define LTR390_INT_TIME_MS ((LTR390_RES_18BIT_INT_TIME_US / 1000.0f) + 20)

typedef struct {
    float sum;
    uint32_t count;
    float average;
} luxData_t;

typedef struct {
    float sum;
    uint32_t count;
    float average;
} uviData_t;

ltr390_t ltr390 = {0};
luxData_t luxData = {0};
uviData_t uviData = {0};

volatile bool readSample = false;
volatile bool readMeas = false;

bool sample_callback(__unused struct repeating_timer *t) {
    readSample = true;
}

bool meas_callback(__unused struct repeating_timer *t) {
    readMeas = true;
}

int main() {
    stdio_init_all();

    i2c_init(I2C_BUS, I2C_SPEED);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ltr390_init_struct(&ltr390, I2C_BUS, I2C_ADDRESS, I2C_SPEED, NULL, false);
    if (!ltr390_soft_reset(&ltr390))
        return -1;
    if (!ltr390_set_resolution(&ltr390, LTR390_RESOLUTION))
        return -1;
    if (!ltr390_set_gain(&ltr390, LTR390_GAIN))
        return -1;
    if (!ltr390_set_meas_rate(&ltr390, LTR390_MEAS_RATE))
        return -1;
    if (!ltr390_enable(&ltr390, true))
        return -1;

    struct repeating_timer measTimer;
    add_repeating_timer_ms(-MEAS_TIME_MS, meas_callback, NULL, &measTimer);
    struct repeating_timer sampleTimer;
    add_repeating_timer_ms(-SAMPLE_TIME_MS, sample_callback, NULL, &sampleTimer);

    while (1) {
        if (readSample) {
            readSample = false;

            float lux, uvi;
            
            if (!ltr390_set_mode(&ltr390, LTR390_ALS_MODE))
                return -1;
            sleep_ms(LTR390_INT_TIME_MS);
            if (!ltr390_read_lux(&ltr390, &lux))
                return -1;
            luxData.sum += lux;
            luxData.count++;
            printf("Lux: %f\n", lux);

            if (!ltr390_set_mode(&ltr390, LTR390_UVS_MODE))
                return -1;
            sleep_ms(LTR390_INT_TIME_MS);
            if (!ltr390_read_uvi(&ltr390, &uvi))
                return -1;
            uviData.sum += uvi;
            uviData.count++;
            printf("UV Index: %f\n", uvi);
        }
        if (readMeas) {
            readMeas = false;

            luxData.average = luxData.sum / luxData.count;
            uviData.average = uviData.sum / uviData.count;

            printf("Lux average: %f\n", luxData.average);
            printf("UV Index average: %f\n", uviData.average);

            luxData.sum = 0;
            luxData.count = 0;
            uviData.sum = 0;
            uviData.count = 0;
        }
    }
    return 0;
}
